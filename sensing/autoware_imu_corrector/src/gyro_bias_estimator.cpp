// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/imu_corrector/gyro_bias_estimator.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::imu_corrector
{
GyroBiasEstimator::GyroBiasEstimator(const rclcpp::NodeOptions & options)
: rclcpp::Node("gyro_bias_scale_validator", options),
  gyro_bias_threshold_(declare_parameter<double>("gyro_bias_threshold")),
  angular_velocity_offset_x_(declare_parameter<double>("angular_velocity_offset_x")),
  angular_velocity_offset_y_(declare_parameter<double>("angular_velocity_offset_y")),
  angular_velocity_offset_z_(declare_parameter<double>("angular_velocity_offset_z")),
  timer_callback_interval_sec_(declare_parameter<double>("timer_callback_interval_sec")),
  diagnostics_updater_interval_sec_(declare_parameter<double>("diagnostics_updater_interval_sec")),
  straight_motion_ang_vel_upper_limit_(
    declare_parameter<double>("straight_motion_ang_vel_upper_limit")),
  min_allowed_scale_(declare_parameter<double>("min_allowed_scale")),
  max_allowed_scale_(declare_parameter<double>("max_allowed_scale")),
  alpha_(declare_parameter<double>("alpha")),
  alpha_ndt_rate_(declare_parameter<double>("alpha_ndt_rate")),
  threshold_to_estimate_scale_(declare_parameter<double>("threshold_to_estimate_scale")),
  percentage_scale_rate_allow_correct_(
    declare_parameter<double>("percentage_scale_rate_allow_correct")),
  alpha_gyro_(declare_parameter<double>("alpha_gyro")),
  delay_gyro_ms_(declare_parameter<int>("delay_gyro_ms")),
  buffer_size_gyro_(declare_parameter<int>("buffer_size_gyro")),
  samples_filter_pose_rate_(declare_parameter<int>("samples_filter_pose_rate")),
  samples_filter_gyro_rate_(declare_parameter<int>("samples_filter_gyro_rate")),
  ndt_yaw_rate_(0.0),
  gyro_yaw_rate_(0.0),
  gyro_yaw_angle_(0.0),
  ndt_yaw_angle_(0.0),
  avg_rate_pose_(0.0),
  avg_rate_gyro_(0.0),
  updater_(this),
  gyro_bias_(std::nullopt)
{
  updater_.setHardwareID(get_name());
  updater_.add("gyro_bias_scale_validator", this, &GyroBiasEstimator::update_diagnostics);
  updater_.setPeriod(diagnostics_updater_interval_sec_);

  gyro_bias_estimation_module_ = std::make_unique<GyroBiasEstimationModule>();

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "~/input/imu_raw", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) { callback_imu(msg); });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odom", rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { callback_odom(msg); });
  gyro_bias_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "~/output/gyro_bias", rclcpp::SensorDataQoS());
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "~/input/pose_ndt", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
      callback_pose_msg(msg);
    });
  gyro_scale_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "~/output/gyro_scale", rclcpp::SensorDataQoS());
  imu_scaled_pub_ = create_publisher<sensor_msgs::msg::Imu>("~/output/imu_scaled", rclcpp::QoS{1});

  auto bound_timer_callback = std::bind(&GyroBiasEstimator::timer_callback, this);
  auto period_control = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timer_callback_interval_sec_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(bound_timer_callback)>>(
    this->get_clock(), period_control, std::move(bound_timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);

  transform_listener_ = std::make_shared<autoware_utils::TransformListener>(this);

  // initialize diagnostics_info_
  {
    diagnostics_info_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics_info_.summary_message = "Not initialized";
    diagnostics_info_.gyro_bias_x_for_imu_corrector = std::nan("");
    diagnostics_info_.gyro_bias_y_for_imu_corrector = std::nan("");
    diagnostics_info_.gyro_bias_z_for_imu_corrector = std::nan("");
    diagnostics_info_.estimated_gyro_bias_x = std::nan("");
    diagnostics_info_.estimated_gyro_bias_y = std::nan("");
    diagnostics_info_.estimated_gyro_bias_z = std::nan("");
    diagnostics_info_.estimated_gyro_scale_x = std::nan("");
    diagnostics_info_.estimated_gyro_scale_y = std::nan("");
    diagnostics_info_.estimated_gyro_scale_z = std::nan("");
  }
  // initialize gyro_info vehicle velocity converter
  {
    gyro_info_.bias_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
    gyro_info_.bias_status_summary = "OK";
    gyro_info_.bias_summary_message = "Not initialized";
    gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
    gyro_info_.scale_status_summary = "OK";
    gyro_info_.scale_summary_message = "Not initialized";
  }

  // EKF variables initialization
  double initial_scale = declare_parameter<double>("estimate_scale_init");
  double noise_ekf_r = declare_parameter<double>("ekf_rate.measurement_noise_r");
  ekf_rate_.max_variance_ = declare_parameter<double>("ekf_rate.max_variance_p");
  ekf_rate_.min_covariance_ = declare_parameter<double>("ekf_rate.min_covariance");
  ekf_rate_.estimated_scale_rate_ = initial_scale;
  ekf_rate_.p_ = ekf_rate_.max_variance_;
  ekf_rate_.q_ = declare_parameter<double>("ekf_rate.process_noise_q");
  ekf_rate_.variance_p_after_ = declare_parameter<double>("ekf_rate.variance_p_after");
  ekf_rate_.measurement_noise_r_after_ =
    declare_parameter<double>("ekf_rate.measurement_noise_r_after");
  ekf_rate_.r_ = noise_ekf_r * noise_ekf_r;
  ekf_rate_.filtered_scale_rate_ = initial_scale;
  ekf_rate_.process_noise_q_after_ = declare_parameter<double>("ekf_rate.process_noise_q_after");
  ekf_rate_.samples_to_init_ = declare_parameter<int>("ekf_rate.samples_to_init");
  ekf_rate_.filtered_scale_initialized_ = false;
  ekf_rate_.n_big_changes_detected_ = 0;

  ekf_angle_.initial_scale_ = initial_scale;
  ekf_angle_.x_state_(0) = 0.0;                        // angle
  ekf_angle_.x_state_(1) = ekf_angle_.initial_scale_;  // estimated scale
  ekf_angle_.estimated_scale_angle_ = ekf_angle_.initial_scale_;
  ekf_angle_.max_variance_p_angle_ = declare_parameter<double>("ekf_angle.variance_p_angle");
  ekf_angle_.noise_ekf_r_angle_ = declare_parameter<double>("ekf_angle.measurement_noise_r_angle");
  ekf_angle_.q_angle_ << 0, 0, 0, declare_parameter<double>("ekf_angle.process_noise_q_angle");
  ekf_angle_.p_angle_ << ekf_angle_.max_variance_p_angle_, 0, 0, ekf_angle_.max_variance_p_angle_;
  ekf_angle_.r_angle_ << ekf_angle_.noise_ekf_r_angle_ * ekf_angle_.noise_ekf_r_angle_;
  ekf_angle_.filtered_scale_angle_ = ekf_angle_.initial_scale_;
  ekf_angle_.has_gyro_yaw_angle_init_ = false;
  ekf_angle_.min_covariance_angle_ = declare_parameter<double>("ekf_angle.min_covariance_angle");
  ekf_angle_.decay_coefficient_ = declare_parameter<double>("ekf_angle.decay_coefficient");
  ekf_angle_.in_bounds_sample_count_ = 0;
  ekf_angle_.samples_in_bounds_to_init_ =
    declare_parameter<int>("ekf_angle.samples_in_bounds_to_init");
  ekf_angle_.filter_initialized_ = false;
  ekf_angle_.reinitialization_count_ = 0;
  ekf_angle_.max_reinitialization_retries_ =
    declare_parameter<int>("ekf_angle.max_reinitialization_retries");
  ekf_angle_.reinitialization_failed_ = false;

  scale_imu_.modify_imu_scale_ = declare_parameter<bool>("scale_imu_injection.modify_imu_scale");
  scale_imu_.drift_scale_ = declare_parameter<double>("scale_imu_injection.drift_scale");
  scale_imu_.drift_bias_ = declare_parameter<double>("scale_imu_injection.drift_bias");
  scale_imu_.scale_final_ = declare_parameter<double>("scale_imu_injection.scale_on_purpose");
  scale_imu_.bias_final_ = declare_parameter<double>("scale_imu_injection.bias_on_purpose");

  start_time_check_scale_ = this->now();
  last_time_rx_pose_ = this->now();
  last_time_rx_imu_ = this->now();
}

sensor_msgs::msg::Imu GyroBiasEstimator::modify_imu(
  sensor_msgs::msg::Imu & imu_msg, ScaleImuSignal & scale_imu, rclcpp::Time & time)
{
  // Modify the IMU data to inject bias and scale
  scale_imu.scale_final_ += scale_imu.drift_scale_;
  scale_imu.bias_final_ += scale_imu.drift_bias_;

  sensor_msgs::msg::Imu imu_mod = imu_msg;
  imu_mod.header.stamp = time;
  imu_mod.header.frame_id = imu_msg.header.frame_id;

  imu_mod.angular_velocity.x =
    scale_imu.scale_final_ * imu_msg.angular_velocity.x + scale_imu.bias_final_;
  imu_mod.angular_velocity.y =
    scale_imu.scale_final_ * imu_msg.angular_velocity.y + scale_imu.bias_final_;
  imu_mod.angular_velocity.z =
    scale_imu.scale_final_ * imu_msg.angular_velocity.z + scale_imu.bias_final_;

  return imu_mod;
}

void GyroBiasEstimator::callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  rclcpp::Time msg_time = imu_msg_ptr->header.stamp;

  double dt_imu = (msg_time - last_time_rx_imu_).seconds();
  last_time_rx_imu_ = msg_time;
  if (dt_imu == 0.0) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "DT_imu is zero");
    return;
  }

  imu_frame_ = imu_msg_ptr->header.frame_id;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
    transform_listener_->get_latest_transform(imu_frame_, output_frame_);
  if (!tf_imu2base_ptr) {
    RCLCPP_ERROR_ONCE(
      this->get_logger(), "Please publish TF %s to %s", output_frame_.c_str(),
      (imu_frame_).c_str());
    return;
  }

  geometry_msgs::msg::Vector3Stamped gyro;
  gyro.header.stamp = imu_msg_ptr->header.stamp;
  if (scale_imu_.modify_imu_scale_) {
    sensor_msgs::msg::Imu imu_msg = *imu_msg_ptr;
    rclcpp::Time now_time = this->now();
    auto imu_msg_mod = modify_imu(imu_msg, scale_imu_, now_time);
    // Publish modified IMU
    imu_scaled_pub_->publish(imu_msg_mod);
    // Transform gyro from modified IMU
    gyro.vector = transform_vector3(imu_msg_mod.angular_velocity, *tf_imu2base_ptr);
  } else {
    gyro.vector = transform_vector3(imu_msg_ptr->angular_velocity, *tf_imu2base_ptr);
  }

  gyro_all_.push_back(gyro);        // Used to update the gyro bias
  gyro_scale_buf_.push_back(gyro);  // Used to update the scale

  // Keep gyro buffer fixed size
  if (gyro_scale_buf_.size() > static_cast<size_t>(buffer_size_gyro_)) {
    gyro_scale_buf_.erase(gyro_scale_buf_.begin());
  }

  // Filter gyro data
  gyro_yaw_rate_ = alpha_gyro_ * gyro_yaw_rate_ + (1 - alpha_gyro_) * gyro.vector.z;

  // EKF always update p_
  ekf_rate_.p_ = ekf_rate_.p_ + ekf_rate_.q_;

  // Update diagnostics when necessary
  if (gyro_info_.scale_status != diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    if (ekf_rate_.p_ > ekf_rate_.max_variance_) {
      gyro_info_.scale_summary_message =
        "Covariance is high, scale estimation hasn't been updated for a while";
      gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      gyro_info_.scale_status_summary = "WARN";
    }
    if (ekf_rate_.p_ < ekf_rate_.min_covariance_) {
      gyro_info_.scale_summary_message = "limiting covariance";
      gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      gyro_info_.scale_status_summary = "WARN";
    }
  }

  // Limit covariance
  if (ekf_rate_.p_ > ekf_rate_.max_variance_) {
    ekf_rate_.p_ = ekf_rate_.max_variance_;
  }
  if (ekf_rate_.p_ < ekf_rate_.min_covariance_) {
    ekf_rate_.p_ = ekf_rate_.min_covariance_;
  }

  // Publish results for debugging
  if (gyro_bias_ != std::nullopt && gyro_bias_not_rotated_.has_value()) {
    // Angle is updated here but restarted when angle from pose is received
    gyro_yaw_angle_ +=
      (ekf_angle_.x_state_(1) * (gyro.vector.z) - gyro_bias_not_rotated_.value().z) * dt_imu;
    ekf_angle_.x_state_(1) = (ekf_angle_.x_state_(1) * ekf_angle_.decay_coefficient_);

    // EKF update
    ekf_angle_.x_state_(0) = gyro_yaw_angle_;
    Eigen::Matrix2d f_matrix;
    f_matrix << 1, dt_imu * (gyro.vector.z - gyro_bias_not_rotated_.value().z), 0,
      ekf_angle_.decay_coefficient_;
    ekf_angle_.p_angle_ =
      f_matrix * ekf_angle_.p_angle_ * f_matrix.transpose() + ekf_angle_.q_angle_;

    // Limit covariance
    ekf_angle_.p_angle_(0, 0) = std::min(
      std::max(ekf_angle_.p_angle_(0, 0), ekf_angle_.min_covariance_angle_),
      ekf_angle_.max_variance_p_angle_);
    ekf_angle_.p_angle_(1, 1) = std::min(
      std::max(ekf_angle_.p_angle_(1, 1), ekf_angle_.min_covariance_angle_),
      ekf_angle_.max_variance_p_angle_);

    if (gyro_yaw_angle_ < -M_PI) {
      gyro_yaw_angle_ += 2.0 * M_PI;
    } else if (gyro_yaw_angle_ > M_PI) {
      gyro_yaw_angle_ -= 2.0 * M_PI;
    }
    geometry_msgs::msg::Vector3Stamped gyro_bias_msg;
    gyro_bias_msg.header.stamp = this->now();
    gyro_bias_msg.vector = gyro_bias_.value();
    gyro_bias_pub_->publish(gyro_bias_msg);
  }
}

void GyroBiasEstimator::callback_odom(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_ptr)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom_msg_ptr->header;
  pose.pose = odom_msg_ptr->pose.pose;
  pose_buf_.push_back(pose);
}

void GyroBiasEstimator::callback_pose_msg(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg_ptr)
{
  estimate_scale_gyro(pose_msg_ptr);
}

double GyroBiasEstimator::extract_yaw_from_pose(
  const geometry_msgs::msg::Quaternion & quat_msg, tf2::Quaternion & quat_out)
{
  quat_out = tf2::Quaternion(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
  quat_out.normalize();

  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3(quat_out).getRPY(roll, pitch, yaw);
  return yaw;
}

double GyroBiasEstimator::compute_yaw_rate_from_quat(
  const tf2::Quaternion & quat, const tf2::Quaternion & prev_quat, double dt)
{
  tf2::Quaternion q_delta = quat * prev_quat.inverse();
  q_delta.normalize();

  tf2::Vector3 axis = q_delta.getAxis();
  double angle = q_delta.getAngle();
  if (angle > M_PI) angle -= 2 * M_PI;

  tf2::Vector3 angular_velocity = axis * (angle / dt);
  return angular_velocity.z();
}

void GyroBiasEstimator::reinitialize_angle_ekf(EKFEstimateScaleAngleVars & ekf_angle)
{
  ekf_angle.x_state_(0) = 0.0;                       // angle
  ekf_angle.x_state_(1) = ekf_angle.initial_scale_;  // estimated scale
  ekf_angle.estimated_scale_angle_ = ekf_angle.initial_scale_;
  ekf_angle.p_angle_ << ekf_angle.max_variance_p_angle_, 0, 0, ekf_angle.max_variance_p_angle_;
  ekf_angle.q_angle_ << 0, 0, 0, ekf_angle.process_noise_q_angle_;
  ekf_angle.r_angle_ << ekf_angle.noise_ekf_r_angle_ * ekf_angle.noise_ekf_r_angle_;
  ekf_angle.filtered_scale_angle_ = ekf_angle.initial_scale_;
  ekf_angle.has_gyro_yaw_angle_init_ = false;
  ekf_angle.in_bounds_sample_count_ = 0;
}

void GyroBiasEstimator::update_angle_ekf(double yaw_ndt, EKFEstimateScaleAngleVars & ekf_angle)
{
  const double angle_tolerance_eval = 0.1;  // radians

  // If reinitialization has failed (max retries exceeded), don't update EKF
  if (ekf_angle.reinitialization_failed_) {
    return;
  }

  // Check if estimated scale falls outside the allowed tolerances
  if (
    ekf_angle.estimated_scale_angle_ < min_allowed_scale_ ||
    ekf_angle.estimated_scale_angle_ > max_allowed_scale_) {
    // If filter is not yet initialized and scale goes out of bounds, reinitialize
    if (!ekf_angle.filter_initialized_) {
      ekf_angle.in_bounds_sample_count_ = 0;

      // Check if max retries exceeded
      if (ekf_angle.reinitialization_count_ >= ekf_angle.max_reinitialization_retries_) {
        ekf_angle.reinitialization_failed_ = true;
        gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        gyro_info_.scale_status_summary = "ERR";
        gyro_info_.scale_summary_message =
          "EKF angle scale reinitialization failed after maximum attempts";
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to initialize EKF angle after %d attempts. Scale: %.6f, limits: [%.6f, %.6f]",
          ekf_angle.max_reinitialization_retries_, ekf_angle.estimated_scale_angle_,
          min_allowed_scale_, max_allowed_scale_);
        return;
      }

      ekf_angle.reinitialization_count_++;
      reinitialize_angle_ekf(ekf_angle);
      gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      RCLCPP_WARN(
        this->get_logger(), "EKF angle scale out of bounds (%.6f). Reinitialization attempt %d/%d",
        ekf_angle.estimated_scale_angle_, ekf_angle.reinitialization_count_,
        ekf_angle.max_reinitialization_retries_);
      gyro_info_.scale_summary_message = "EKF angle scale out of bounds, reinitializing";
      return;
    }
    // If already initialized, continue updating EKF normally
  } else {
    // Scale is within bounds - track consecutive in-bounds samples for initialization
    if (!ekf_angle.filter_initialized_) {
      ekf_angle.in_bounds_sample_count_++;

      // Initialize filtered_scale_angle_ once we have enough consecutive in-bounds samples
      if (ekf_angle.in_bounds_sample_count_ >= ekf_angle.samples_in_bounds_to_init_) {
        ekf_angle.filtered_scale_angle_ = ekf_angle.estimated_scale_angle_;
        ekf_angle.filter_initialized_ = true;
        gyro_info_.scale_summary_message = "EKF angle scale successfully initialized";
        RCLCPP_INFO(
          this->get_logger(),
          "EKF angle filter successfully initialized. filtered_scale_angle = %.6f after %d "
          "consecutive in-bounds samples (reinitializations: %d)",
          ekf_angle.filtered_scale_angle_, ekf_angle.samples_in_bounds_to_init_,
          ekf_angle.reinitialization_count_);
      }
    }
  }

  // EKF update only if the angle difference is within tolerance
  if (std::abs(yaw_ndt - ekf_angle.x_state_(0)) < angle_tolerance_eval) {
    Eigen::Matrix<double, 1, 2> h_matrix;
    h_matrix << 1, 0;
    Eigen::Matrix<double, 1, 1> y;
    y << yaw_ndt - ekf_angle.x_state_(0);
    Eigen::Matrix<double, 1, 1> s_matrix =
      h_matrix * ekf_angle.p_angle_ * h_matrix.transpose() + ekf_angle.r_angle_;
    Eigen::Matrix<double, 2, 1> k_matrix =
      ekf_angle.p_angle_ * h_matrix.transpose() * s_matrix.inverse();
    ekf_angle.x_state_ = ekf_angle.x_state_ + k_matrix * y;
    ekf_angle.estimated_scale_angle_ = ekf_angle.x_state_(1);
    ekf_angle.p_angle_ = (Eigen::Matrix2d::Identity() - k_matrix * h_matrix) * ekf_angle.p_angle_;
    // Update filtered scale if initial value was in bounds
    if (ekf_angle.filter_initialized_) {
      ekf_angle.filtered_scale_angle_ = alpha_ * ekf_angle.filtered_scale_angle_ +
                                        (1.0 - alpha_) * ekf_angle.estimated_scale_angle_;
    }
  }
}

void GyroBiasEstimator::update_rate_ekf(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose_msg_ptr,
  EKFEstimateScaleRateVars & ekf_rate_state)
{
  // EKF update
  // Find in the buffer the delayed gyro data and average the last samples
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_local_buf = gyro_scale_buf_;
  rclcpp::Time pose_time(pose_msg_ptr->header.stamp);
  rclcpp::Time target_time =
    pose_time - rclcpp::Duration::from_nanoseconds(delay_gyro_ms_ * 1000 * 1000);
  auto closest_delayed_it = gyro_local_buf.end();

  for (auto it = gyro_local_buf.begin(); it != gyro_local_buf.end(); ++it) {
    rclcpp::Time buf_time(it->header.stamp);
    if (buf_time <= target_time) {
      closest_delayed_it = it;  // Found delayed element in the buffer
    }
  }

  // Average the last samples of the gyro rate
  size_t samples_window = samples_filter_gyro_rate_;
  if (closest_delayed_it != gyro_local_buf.end()) {
    auto distance = std::distance(gyro_local_buf.begin(), closest_delayed_it);
    auto start_it = gyro_local_buf.begin();

    if (distance >= static_cast<int64_t>(samples_window)) {
      start_it = closest_delayed_it - static_cast<int64_t>(samples_window);
    }
    double sum_gyro_delayed_rate = 0.0;
    for (auto it = start_it; it != closest_delayed_it; ++it) {
      sum_gyro_delayed_rate += it->vector.z;
    }
    avg_rate_gyro_ = sum_gyro_delayed_rate / samples_filter_gyro_rate_;
  } else {
    avg_rate_gyro_ = 0.0;
  }

  rate_pose_buff_.push_back(ndt_yaw_rate_);

  // Calculate the EKF if enough samples are available
  if (
    rate_pose_buff_.size() > static_cast<size_t>(samples_filter_pose_rate_) &&
    gyro_bias_not_rotated_.has_value()) {
    avg_rate_pose_ =
      std::accumulate(rate_pose_buff_.begin(), rate_pose_buff_.end(), 0.0) / rate_pose_buff_.size();
    // Keep the buffer size fixed
    rate_pose_buff_.erase(rate_pose_buff_.begin());

    auto h = avg_rate_pose_;
    // To avoid confusion with the bias sign we remove the bias from the gyro rate directly
    auto y = (avg_rate_gyro_ - gyro_bias_not_rotated_.value().z) -
             (ekf_rate_state.estimated_scale_rate_ * avg_rate_pose_);
    auto s = h * ekf_rate_state.p_ * h + ekf_rate_state.r_;
    auto k = ekf_rate_state.p_ * h / s;

    ekf_rate_state.estimated_scale_rate_ = ekf_rate_state.estimated_scale_rate_ + k * y;
    ekf_rate_state.p_ = (1 - k * h) * ekf_rate_state.p_;

    // Initialize the slow phase after the first samples
    if (!ekf_rate_state.filtered_scale_initialized_) {
      estimated_scale_buff_.push_back(ekf_rate_state.estimated_scale_rate_);
      if (estimated_scale_buff_.size() > static_cast<size_t>(ekf_rate_state.samples_to_init_)) {
        double average_est_scale =
          std::accumulate(estimated_scale_buff_.begin(), estimated_scale_buff_.end(), 0.0) /
          static_cast<double>(estimated_scale_buff_.size());
        ekf_rate_state.filtered_scale_rate_ = average_est_scale;
        estimated_scale_buff_.clear();
        // Update parameters
        ekf_rate_state.max_variance_ = ekf_rate_state.variance_p_after_;
        ekf_rate_state.r_ =
          ekf_rate_state.measurement_noise_r_after_ * ekf_rate_state.measurement_noise_r_after_;
        ekf_rate_state.q_ = ekf_rate_state.process_noise_q_after_;
        ekf_rate_state.filtered_scale_initialized_ = true;
      }
    }

    if (ekf_angle_.reinitialization_failed_) {
      gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      gyro_info_.scale_status_summary = "ERR";
      gyro_info_.scale_summary_message =
        "EKF angle scale reinitialization failed after maximum attempts. Check IMU, NDT device or "
        "TF.";
    } else if (
      ekf_angle_.filtered_scale_angle_ < min_allowed_scale_ ||
      ekf_angle_.filtered_scale_angle_ > max_allowed_scale_) {
      gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      gyro_info_.scale_status_summary = "ERR";
      gyro_info_.scale_summary_message =
        "Scale is out of the limits, check the IMU, NDT device or TF.";
    } else if (ekf_rate_state.n_big_changes_detected_ == 0) {
      gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
    }

    // Check if the estimated scale is within the allowed range or if a big change is detected
    // only after the angle EKF initialized successfully
    if (ekf_angle_.filter_initialized_) {
      if (
        ekf_rate_state.estimated_scale_rate_ >=
          ekf_rate_state.filtered_scale_rate_ * (1 - percentage_scale_rate_allow_correct_) &&
        ekf_rate_state.estimated_scale_rate_ <=
          ekf_rate_state.filtered_scale_rate_ * (1 + percentage_scale_rate_allow_correct_)) {
        ekf_rate_state.filtered_scale_rate_ = alpha_ * ekf_rate_state.filtered_scale_rate_ +
                                              (1 - alpha_) * ekf_rate_state.estimated_scale_rate_;
        ekf_rate_state.n_big_changes_detected_ = 0;
      } else {
        ekf_rate_state.n_big_changes_detected_++;
        const int n_iterations_until_big_change_error = 10;  // 10 iterations approx 1 seconds
        if (ekf_rate_state.n_big_changes_detected_ >= n_iterations_until_big_change_error) {
          gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          gyro_info_.scale_status_summary = "ERR";
          gyro_info_.scale_summary_message =
            "Large gyro scale change detected in short period of time, check the IMU, NDT device "
            "or TF.";
        }
      }
    }

    geometry_msgs::msg::Vector3Stamped vector_scale;

    vector_scale.header.stamp = this->now();

    // Scale on x , y axis is not estimated, but set to 1.0 for consistency
    vector_scale.vector.x = 1.0;
    vector_scale.vector.y = 1.0;
    if (std::abs(ekf_angle_.filtered_scale_angle_) <= std::numeric_limits<double>::epsilon()) {
      vector_scale.vector.z = 1.0;
    } else {
      vector_scale.vector.z = 1 / ekf_angle_.filtered_scale_angle_;
    }
    gyro_scale_pub_->publish(vector_scale);

    diagnostics_info_.estimated_gyro_scale_z = 1 / ekf_angle_.estimated_scale_angle_;
  }
}

bool GyroBiasEstimator::should_skip_update(double gyro_yaw_rate)
{
  if (std::abs(gyro_yaw_rate) < threshold_to_estimate_scale_) {
    if (gyro_info_.scale_status < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
      gyro_info_.scale_status_summary = "OK";
      gyro_info_.scale_summary_message = "Skipped scale update (yaw rate too small)";
    }

    geometry_msgs::msg::Vector3Stamped msg;
    msg.header.stamp = this->now();
    msg.vector.x = 1.0;
    msg.vector.y = 1.0;
    if (std::abs(ekf_angle_.filtered_scale_angle_) <= std::numeric_limits<double>::epsilon()) {
      msg.vector.z = 1.0;
    } else {
      msg.vector.z = 1 / ekf_angle_.filtered_scale_angle_;
    }
    gyro_scale_pub_->publish(msg);

    ekf_rate_.n_big_changes_detected_ = 0;
    rate_pose_buff_.clear();
    return true;
  }
  return false;
}

void GyroBiasEstimator::estimate_scale_gyro(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg_ptr)
{
  const rclcpp::Time msg_time = pose_msg_ptr->header.stamp;
  const double dt_pose = (msg_time - last_time_rx_pose_).seconds();
  last_time_rx_pose_ = msg_time;
  if (dt_pose == 0.0) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "DT_pose is zero");
    return;
  }

  const auto pose_frame = pose_msg_ptr->header.frame_id;
  auto tf_base2pose_ptr = transform_listener_->get_latest_transform(pose_frame, output_frame_);
  if (!tf_base2pose_ptr) {
    RCLCPP_ERROR_ONCE(
      this->get_logger(), "Please publish TF %s to %s", pose_frame.c_str(), output_frame_.c_str());
    diagnostics_info_.summary_message =
      "Skipped update (tf between base and pose is not available)";
    return;
  }

  tf2::Quaternion quat;
  double yaw_ndt = extract_yaw_from_pose(tf_base2pose_ptr->transform.rotation, quat);
  ndt_yaw_angle_ = yaw_ndt;

  // Initialize EKF with yaw if first time
  if (!ekf_angle_.has_gyro_yaw_angle_init_) {
    ekf_angle_.x_state_(0) = yaw_ndt;
    gyro_yaw_angle_ = yaw_ndt;
    previous_quat_ndt_ = quat;
    ekf_angle_.has_gyro_yaw_angle_init_ = true;
  }

  // Angular velocity from pose
  const double ndt_yaw_rate = compute_yaw_rate_from_quat(quat, previous_quat_ndt_, dt_pose);
  ndt_yaw_rate_ = alpha_ndt_rate_ * ndt_yaw_rate_ + (1.0 - alpha_ndt_rate_) * ndt_yaw_rate;
  previous_quat_ndt_ = quat;

  // Reset gyro yaw angle to the NDT yaw angle
  gyro_yaw_angle_ = ndt_yaw_angle_;

  if (should_skip_update(gyro_yaw_rate_)) {
    return;
  }

  // EKF rate update
  if (
    gyro_bias_.has_value() && gyro_bias_not_rotated_.has_value() &&
    gyro_scale_buf_.size() > static_cast<size_t>(buffer_size_gyro_ - 2)) {
    update_rate_ekf(pose_msg_ptr, ekf_rate_);
    update_angle_ekf(yaw_ndt, ekf_angle_);
  }
}

void GyroBiasEstimator::timer_callback()
{
  if (pose_buf_.empty()) {
    gyro_info_.bias_summary_message = "Skipped update (pose_buf is empty).";
    return;
  }

  // Copy data
  const std::vector<geometry_msgs::msg::PoseStamped> pose_buf = pose_buf_;
  const std::vector<geometry_msgs::msg::Vector3Stamped> gyro_all = gyro_all_;
  pose_buf_.clear();
  gyro_all_.clear();

  // Check time
  const rclcpp::Time t0_rclcpp_time = rclcpp::Time(pose_buf.front().header.stamp);
  const rclcpp::Time t1_rclcpp_time = rclcpp::Time(pose_buf.back().header.stamp);
  if (t1_rclcpp_time <= t0_rclcpp_time) {
    gyro_info_.bias_summary_message = "Skipped update (pose_buf is not in chronological order).";
    return;
  }

  // Filter gyro data
  std::vector<geometry_msgs::msg::Vector3Stamped> gyro_filtered;
  for (const auto & gyro : gyro_all) {
    const rclcpp::Time t = rclcpp::Time(gyro.header.stamp);
    if (t0_rclcpp_time <= t && t < t1_rclcpp_time) {
      gyro_filtered.push_back(gyro);
    }
  }

  // Check gyro data size
  // Data size must be greater than or equal to 2 since the time difference will be taken later
  if (gyro_filtered.size() <= 1) {
    gyro_info_.bias_summary_message = "Skipped update (gyro_filtered size is less than 2).";
    return;
  }

  // Check if the vehicle is moving straight
  const geometry_msgs::msg::Vector3 rpy_0 =
    autoware_utils::get_rpy(pose_buf.front().pose.orientation);
  const geometry_msgs::msg::Vector3 rpy_1 =
    autoware_utils::get_rpy(pose_buf.back().pose.orientation);
  const double yaw_diff = std::abs(autoware_utils::normalize_radian(rpy_1.z - rpy_0.z));
  const double time_diff = (t1_rclcpp_time - t0_rclcpp_time).seconds();
  const double yaw_vel = yaw_diff / time_diff;
  const bool is_straight = (yaw_vel < straight_motion_ang_vel_upper_limit_);
  if (!is_straight) {
    gyro_info_.bias_summary_message =
      "Skipped update (yaw angular velocity is greater than straight_motion_ang_vel_upper_limit).";
    return;
  }

  // Calculate gyro bias
  gyro_bias_estimation_module_->update_bias(pose_buf, gyro_filtered);

  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_base2imu_ptr =
    transform_listener_->get_latest_transform(output_frame_, imu_frame_);
  if (!tf_base2imu_ptr) {
    RCLCPP_ERROR_ONCE(
      this->get_logger(), "Please publish TF %s to %s", imu_frame_.c_str(), output_frame_.c_str());
    gyro_info_.bias_summary_message = "Skipped update (tf between base and imu is not available).";
    return;
  }
  gyro_bias_not_rotated_ = gyro_bias_estimation_module_->get_bias_base_link();
  gyro_bias_ =
    transform_vector3(gyro_bias_estimation_module_->get_bias_base_link(), *tf_base2imu_ptr);

  validate_gyro_bias();
}

void GyroBiasEstimator::validate_gyro_bias()
{
  // Calculate diagnostics key-values
  diagnostics_info_.gyro_bias_x_for_imu_corrector = gyro_bias_.value().x;
  diagnostics_info_.gyro_bias_y_for_imu_corrector = gyro_bias_.value().y;
  diagnostics_info_.gyro_bias_z_for_imu_corrector = gyro_bias_.value().z;
  diagnostics_info_.estimated_gyro_bias_x = gyro_bias_.value().x - angular_velocity_offset_x_;
  diagnostics_info_.estimated_gyro_bias_y = gyro_bias_.value().y - angular_velocity_offset_y_;
  diagnostics_info_.estimated_gyro_bias_z = gyro_bias_.value().z - angular_velocity_offset_z_;

  // Validation
  const bool is_bias_small_enough =
    std::abs(diagnostics_info_.estimated_gyro_bias_x) < gyro_bias_threshold_ &&
    std::abs(diagnostics_info_.estimated_gyro_bias_y) < gyro_bias_threshold_ &&
    std::abs(diagnostics_info_.estimated_gyro_bias_z) < gyro_bias_threshold_;

  // Update diagnostics
  if (is_bias_small_enough) {
    gyro_info_.bias_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
    gyro_info_.bias_status_summary = "OK";
    gyro_info_.bias_summary_message = "Successfully updated";
  } else {
    gyro_info_.bias_status = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    gyro_info_.bias_status_summary = "WARN";
    gyro_info_.bias_summary_message =
      "Gyro bias may be incorrect. Please calibrate IMU and reflect the result in imu_corrector. "
      "You may also use the output of gyro_bias_estimator.";
  }
}

geometry_msgs::msg::Vector3 GyroBiasEstimator::transform_vector3(
  const geometry_msgs::msg::Vector3 & vec, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::Vector3Stamped vec_stamped;
  vec_stamped.vector = vec;

  geometry_msgs::msg::Vector3Stamped vec_stamped_transformed;
  tf2::doTransform(vec_stamped, vec_stamped_transformed, transform);
  return vec_stamped_transformed.vector;
}

void GyroBiasEstimator::update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  auto f = [](const double & value) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(8) << value;
    return ss.str();
  };

  // Check for the highest status priority
  if (gyro_info_.scale_status >= gyro_info_.bias_status) {
    diagnostics_info_.level = gyro_info_.scale_status;
    diagnostics_info_.summary_message = gyro_info_.scale_summary_message;
  } else {
    diagnostics_info_.level = gyro_info_.bias_status;
    diagnostics_info_.summary_message = gyro_info_.bias_summary_message;
  }

  stat.summary(diagnostics_info_.level, diagnostics_info_.summary_message);
  stat.add("gyro_bias_x_for_imu_corrector", f(diagnostics_info_.gyro_bias_x_for_imu_corrector));
  stat.add("gyro_bias_y_for_imu_corrector", f(diagnostics_info_.gyro_bias_y_for_imu_corrector));
  stat.add("gyro_bias_z_for_imu_corrector", f(diagnostics_info_.gyro_bias_z_for_imu_corrector));

  stat.add("estimated_gyro_bias_x", f(diagnostics_info_.estimated_gyro_bias_x));
  stat.add("estimated_gyro_bias_y", f(diagnostics_info_.estimated_gyro_bias_y));
  stat.add("estimated_gyro_bias_z", f(diagnostics_info_.estimated_gyro_bias_z));

  stat.add("estimated_gyro_scale_z", f(diagnostics_info_.estimated_gyro_scale_z));

  stat.add("gyro_bias_status", gyro_info_.bias_status_summary);
  stat.add("gyro_scale_status", gyro_info_.scale_status_summary);

  stat.add("min_allowed_scale", (min_allowed_scale_));
  stat.add("max_allowed_scale", (max_allowed_scale_));

  if (gyro_bias_.has_value()) {
    stat.add("gyro_yaw_rate_", f(gyro_yaw_rate_ - gyro_bias_not_rotated_.value().z));
    stat.add("ndt_yaw_rate_", f(ndt_yaw_rate_));
  }

  stat.add("covariance_rate_estimation", f(ekf_rate_.p_ * 1e10));
  stat.add("covariance_angle_estimation", f(ekf_angle_.p_angle_(1, 1) * 1e10));
}

}  // namespace autoware::imu_corrector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::imu_corrector::GyroBiasEstimator)
