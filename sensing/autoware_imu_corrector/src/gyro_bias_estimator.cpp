// Copyright 2023 TIER IV, Inc.
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

#include "gyro_bias_estimator.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::imu_corrector
{
GyroBiasEstimator::GyroBiasEstimator(const rclcpp::NodeOptions & options)
: rclcpp::Node(
    "gyro_bias_validator_scale",
    options),  // Todo(SergioReyesSan): Change node name to "gyro_bias_validator"
  gyro_bias_threshold_(declare_parameter<double>("gyro_bias_threshold")),
  angular_velocity_offset_x_(declare_parameter<double>("angular_velocity_offset_x")),
  angular_velocity_offset_y_(declare_parameter<double>("angular_velocity_offset_y")),
  angular_velocity_offset_z_(declare_parameter<double>("angular_velocity_offset_z")),
  timer_callback_interval_sec_(declare_parameter<double>("timer_callback_interval_sec")),
  diagnostics_updater_interval_sec_(declare_parameter<double>("diagnostics_updater_interval_sec")),
  straight_motion_ang_vel_upper_limit_(
    declare_parameter<double>("straight_motion_ang_vel_upper_limit")),
  estimate_scale_init_(declare_parameter<double>("estimate_scale_init")),
  ekf_variance_p_(declare_parameter<double>("ekf_variance_p")),
  ekf_variance_p_after_(declare_parameter<double>("ekf_variance_p_after")),
  ekf_process_noise_q_(declare_parameter<double>("ekf_process_noise_q")),
  ekf_process_noise_q_after_(declare_parameter<double>("ekf_process_noise_q_after")),
  ekf_measurement_noise_r_(declare_parameter<double>("ekf_measurement_noise_r")),
  ekf_measurement_noise_r_after_(declare_parameter<double>("ekf_measurement_noise_r_after")),
  min_allowed_scale_(declare_parameter<double>("min_allowed_scale")),
  max_allowed_scale_(declare_parameter<double>("max_allowed_scale")),
  scale_on_purpose_(declare_parameter<double>("scale_on_purpose")),
  bias_on_purpose_(declare_parameter<double>("bias_on_purpose")),
  drift_scale_(declare_parameter<double>("drift_scale")),
  drift_bias_(declare_parameter<double>("drift_bias")),
  alpha_(declare_parameter<double>("alpha")),
  alpha_ndt_rate_(declare_parameter<double>("alpha_ndt_rate")),
  threshold_to_estimate_scale_(declare_parameter<double>("threshold_to_estimate_scale")),
  percentage_scale_rate_allow_correct_(
    declare_parameter<double>("percentage_scale_rate_allow_correct")),
  warning_covariance_(declare_parameter<double>("warning_covariance")),
  min_covariance_(declare_parameter<double>("min_covariance")),
  alpha_gyro_(declare_parameter<double>("alpha_gyro")),
  ekf_process_noise_q_angle_(declare_parameter<double>("ekf_process_noise_q_angle")),
  ekf_variance_p_angle_(declare_parameter<double>("ekf_variance_p_angle")),
  ekf_measurement_noise_r_angle_(declare_parameter<double>("ekf_measurement_noise_r_angle")),
  decay_coefficient_(declare_parameter<double>("decay_coefficient")),
  delay_gyro_ms_(declare_parameter<int>("delay_gyro_ms")),
  samples_to_init_(declare_parameter<int>("samples_to_init")),
  buffer_size_gyro_(declare_parameter<int>("buffer_size_gyro")),
  samples_to_average_delta_(declare_parameter<int>("samples_to_average_delta")),
  samples_filter_pose_rate_(declare_parameter<int>("samples_filter_pose_rate")),
  samples_filter_gyro_rate_(declare_parameter<int>("samples_filter_gyro_rate")),
  filtered_scale_angle_(1.0),
  big_change_scale_rate_(1.0),
  ndt_yaw_rate_(0.0),
  gyro_yaw_rate_(0.0),
  gyro_yaw_angle_(0.0),
  ndt_yaw_angle_(0.0),
  avg_rate_pose_(0.0),
  avg_rate_gyro_(0.0),
  big_change_detect_(0.0),
  has_gyro_yaw_angle_init_(false),
  filtered_scale_initialized_(false),
  updater_(this),
  gyro_bias_(std::nullopt)
{
  updater_.setHardwareID(get_name());
  updater_.add("gyro_bias_validator", this, &GyroBiasEstimator::update_diagnostics);
  updater_.setPeriod(diagnostics_updater_interval_sec_);

  gyro_bias_estimation_module_ = std::make_unique<GyroBiasEstimationModule>();

  imu_sub_ = create_subscription<Imu>(
    "~/input/imu_raw", rclcpp::SensorDataQoS(),
    [this](const Imu::ConstSharedPtr msg) { callback_imu(msg); });
  odom_sub_ = create_subscription<Odometry>(
    "~/input/odom", rclcpp::SensorDataQoS(),
    [this](const Odometry::ConstSharedPtr msg) { callback_odom(msg); });
  gyro_bias_pub_ = create_publisher<Vector3Stamped>("~/output/gyro_bias", rclcpp::SensorDataQoS());
  pose_sub_ = create_subscription<PoseWithCovarianceStamped>(
    "~/input/pose_ndt", rclcpp::SensorDataQoS(),
    [this](const PoseWithCovarianceStamped::ConstSharedPtr msg) { callback_pose_msg(msg); });
  gyro_scale_pub_ =
    create_publisher<Vector3Stamped>("~/output/gyro_scale", rclcpp::SensorDataQoS());
  gyro_debug_pub_ =
    create_publisher<Vector3Stamped>("~/output/gyro_vector_debug", rclcpp::SensorDataQoS());
  scale_debug_pub_ =
    create_publisher<Vector3Stamped>("~/output/scale_vector_debug", rclcpp::SensorDataQoS());
  new_scale_debug_pub_ =
    create_publisher<Vector3Stamped>("~/output/new_scale_vector_debug", rclcpp::SensorDataQoS());

  imu_scaled_pub_ = create_publisher<Imu>("~/output/imu_scaled", rclcpp::QoS{1});

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
  estimated_scale_ = estimate_scale_init_;
  p_ = ekf_variance_p_;
  q_ = ekf_process_noise_q_;
  r_ = ekf_measurement_noise_r_ * ekf_measurement_noise_r_;
  x_state_(0) = 0.0;
  x_state_(1) = estimate_scale_init_;
  estimated_scale_angle_ = estimate_scale_init_;

  p_angle_ << ekf_variance_p_angle_, 0, 0, ekf_variance_p_angle_;
  q_angle_ << 0, 0, 0, ekf_process_noise_q_angle_;
  r_angle_ << ekf_measurement_noise_r_angle_ * ekf_measurement_noise_r_angle_;

  ekf_variance_ = ekf_variance_p_;
  start_time_check_scale_ = this->now();
  final_scale_on_purpose_ = scale_on_purpose_;
  final_bias_on_purpose_ = bias_on_purpose_;
  scale_final_ = scale_on_purpose_;
  bias_final_ = bias_on_purpose_;
  last_time_rx_pose_ = this->now();
  filtered_scale_rate_ = estimate_scale_init_;
  last_time_rx_imu_ = this->now();
}

void GyroBiasEstimator::callback_imu(const Imu::ConstSharedPtr imu_msg_ptr)
{
  // Replace again the original code after testing (SergioReyesSan):
  // imu_frame_ = imu_msg_ptr->header.frame_id;
  // geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
  //   transform_listener_->get_latest_transform(imu_frame_, output_frame_);
  // if (!tf_imu2base_ptr) {
  //   RCLCPP_ERROR(
  //     this->get_logger(), "Please publish TF %s to %s", output_frame_.c_str(),
  //     (imu_frame_).c_str());
  //   return;
  // }

  // geometry_msgs::msg::Vector3Stamped gyro;
  // gyro.header.stamp = imu_msg_ptr->header.stamp;
  // gyro.vector = transform_vector3(imu_msg_ptr->angular_velocity, *tf_imu2base_ptr);

  // gyro_all_.push_back(gyro);

  // Imu imu_msg = *imu_msg_ptr;

  rclcpp::Time msg_time = imu_msg_ptr->header.stamp;

  double dt_imu2 = (msg_time - last_time_rx_imu_).seconds();
  last_time_rx_imu_ = msg_time;
  if (dt_imu2 == 0.0) {
    RCLCPP_ERROR(this->get_logger(), "DT_imu is zero");
    return;
  }

  imu_frame_ = imu_msg_ptr->header.frame_id;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
    transform_listener_->get_latest_transform(imu_frame_, output_frame_);
  if (!tf_imu2base_ptr) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", output_frame_.c_str(),
      (imu_frame_).c_str());
    return;
  }

  // Add bias and scale on purpose
  scale_final_ += drift_scale_;
  bias_final_ += drift_bias_;
  final_scale_on_purpose_ = scale_final_;
  final_bias_on_purpose_ = bias_final_;
  Imu imu_msg_mod = *imu_msg_ptr;

  imu_msg_mod.header.stamp = this->now();
  imu_msg_mod.header.frame_id = imu_msg_ptr->header.frame_id;
  imu_msg_mod.angular_velocity.x = scale_final_ * imu_msg_ptr->angular_velocity.x + bias_final_;
  imu_msg_mod.angular_velocity.y = scale_final_ * imu_msg_ptr->angular_velocity.y + bias_final_;
  imu_msg_mod.angular_velocity.z = scale_final_ * imu_msg_ptr->angular_velocity.z + bias_final_;

  imu_scaled_pub_->publish(imu_msg_mod);

  geometry_msgs::msg::Vector3Stamped gyro;
  gyro.header.stamp = imu_msg_ptr->header.stamp;
  gyro.vector = transform_vector3(imu_msg_mod.angular_velocity, *tf_imu2base_ptr);

  gyro_all_.push_back(gyro);  // Used to update the gyro bias
  gyro_buf_.push_back(gyro);  // Used to update the scale

  // Keep gyro buffer fixed size
  if (gyro_buf_.size() > static_cast<size_t>(buffer_size_gyro_)) {
    gyro_buf_.erase(gyro_buf_.begin());
  }

  // Filter gyro data
  gyro_yaw_rate_ = alpha_gyro_ * gyro_yaw_rate_ + (1 - alpha_gyro_) * gyro.vector.z;

  // EKF always update p_
  p_ = p_ + q_;

  if (gyro_info_.scale_status != diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    if (p_ > ekf_variance_) {
      p_ = ekf_variance_;
      gyro_info_.scale_summary_message =
        "Covariance is high, scale estimation hasn't been updated for a while";
      gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      gyro_info_.scale_status_summary = "WARN";
    }
    if (p_ < min_covariance_) {
      p_ = min_covariance_;
      gyro_info_.scale_summary_message = "limiting covariance";
      gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      gyro_info_.scale_status_summary = "WARN";
    }
  }

  // Publish results for debugging
  if (gyro_bias_ != std::nullopt) {
    if (dt_imu2 != 0) {
      // Angle is updated here but restarted when angle from pose is received
      gyro_yaw_angle_ +=
        (x_state_(1) * (gyro.vector.z) - gyro_bias_not_rotated_.value().z) * dt_imu2;
      x_state_(1) = (x_state_(1) * decay_coefficient_);

      // EKF update
      x_state_(0) = gyro_yaw_angle_;
      Eigen::Matrix2d f_matrix;
      f_matrix << 1, dt_imu2 * (gyro.vector.z - gyro_bias_not_rotated_.value().z), 0,
        decay_coefficient_;
      p_angle_ = f_matrix * p_angle_ * f_matrix.transpose() + q_angle_;
    }
    if (gyro_yaw_angle_ < -M_PI) {
      gyro_yaw_angle_ += 2.0 * M_PI;
    } else if (gyro_yaw_angle_ > M_PI) {
      gyro_yaw_angle_ -= 2.0 * M_PI;
    }
    Vector3Stamped gyro_bias_msg;
    gyro_bias_msg.header.stamp = this->now();
    gyro_bias_msg.vector = gyro_bias_.value();
    gyro_bias_pub_->publish(gyro_bias_msg);
  }
}

void GyroBiasEstimator::callback_odom(const Odometry::ConstSharedPtr odom_msg_ptr)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom_msg_ptr->header;
  pose.pose = odom_msg_ptr->pose.pose;
  pose_buf_.push_back(pose);
}

void GyroBiasEstimator::callback_pose_msg(
  const PoseWithCovarianceStamped::ConstSharedPtr pose_msg_ptr)
{
  estimate_scale_gyro(pose_msg_ptr);
}

void GyroBiasEstimator::estimate_scale_gyro(
  const PoseWithCovarianceStamped::ConstSharedPtr pose_msg_ptr)
{
  rclcpp::Time msg_time = pose_msg_ptr->header.stamp;
  double dt_pose = (msg_time - last_time_rx_pose_).seconds();
  last_time_rx_pose_ = msg_time;
  auto pose_frame = pose_msg_ptr->header.frame_id;
  if (dt_pose == 0.0) {
    RCLCPP_ERROR(this->get_logger(), "DT_pose is zero");
    return;
  }

  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_base2pose_ptr =
    transform_listener_->get_latest_transform(pose_frame, output_frame_);
  if (!tf_base2pose_ptr) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", pose_frame.c_str(), output_frame_.c_str());

    diagnostics_info_.summary_message =
      "Skipped update (tf between base and pose is not available)";
    return;
  }

  auto quat_rotated = tf_base2pose_ptr->transform.rotation;

  // Convert to tf2 quaternion
  tf2::Quaternion quat(quat_rotated.x, quat_rotated.y, quat_rotated.z, quat_rotated.w);

  // Convert to Euler angles
  double roll_ndt = 0.0;
  double pitch_ndt = 0.0;
  double yaw_ndt = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll_ndt, pitch_ndt, yaw_ndt);
  ndt_yaw_angle_ = yaw_ndt;

  if (!has_gyro_yaw_angle_init_) {
    gyro_yaw_angle_ = yaw_ndt;
    x_state_(0) = yaw_ndt;
    previous_quat_ndt_ = quat;
    has_gyro_yaw_angle_init_ = true;
  }

  quat.normalize();
  previous_quat_ndt_.normalize();

  // Compute relative rotation quaternion
  tf2::Quaternion q_delta = quat * previous_quat_ndt_.inverse();
  // Convert relative quaternion to axis-angle
  tf2::Vector3 axis = q_delta.getAxis();
  double angle = q_delta.getAngle();

  // Handle angle wrap
  if (angle > M_PI) {
    angle -= 2 * M_PI;
  }

  tf2::Vector3 angular_velocity = axis * (angle / dt_pose);
  previous_quat_ndt_ = quat;

  // Filter angular velocity
  ndt_yaw_rate_ = alpha_ndt_rate_ * ndt_yaw_rate_ + (1 - alpha_ndt_rate_) * angular_velocity.z();

  // Publish vectors for debugging
  geometry_msgs::msg::Vector3Stamped new_vector_scale;
  new_vector_scale.header.stamp = this->now();
  new_vector_scale.vector.x = final_scale_on_purpose_;
  new_vector_scale.vector.y = estimated_scale_;
  new_vector_scale.vector.z = 1 / estimated_scale_angle_;
  new_scale_debug_pub_->publish(new_vector_scale);

  geometry_msgs::msg::Vector3Stamped scale_debug;
  scale_debug.header.stamp = this->now();
  scale_debug.vector.x = final_scale_on_purpose_;
  scale_debug.vector.y = estimated_scale_;
  scale_debug.vector.z = filtered_scale_rate_;
  scale_debug_pub_->publish(scale_debug);

  const double angle_tolerance_eval = 0.1;
  // New implementation EKF using angle instead of yaw rate
  if (
    std::abs(gyro_yaw_rate_) > threshold_to_estimate_scale_ && gyro_bias_.has_value() &&
    std::abs(ndt_yaw_angle_ - x_state_(0)) < angle_tolerance_eval) {  // Avoid large jumps in angle
    // Update the EKF state with the new angle
    Eigen::Matrix<double, 1, 2> h_matrix;
    h_matrix << 1, 0;
    Eigen::Matrix<double, 1, 1> y;

    y << ndt_yaw_angle_ - x_state_(0);

    Eigen::Matrix<double, 1, 1> s_matrix = h_matrix * p_angle_ * h_matrix.transpose() + r_angle_;
    Eigen::Matrix<double, 2, 1> k_matrix = p_angle_ * h_matrix.transpose() * s_matrix.inverse();
    x_state_ = x_state_ + k_matrix * y;
    estimated_scale_angle_ = x_state_(1);
    p_angle_ = (Eigen::Matrix2d::Identity() - k_matrix * h_matrix) * p_angle_;
  }
  gyro_yaw_angle_ = ndt_yaw_angle_;  // reset the gyro yaw angle to the NDT yaw angle

  if (std::abs(gyro_yaw_rate_) < threshold_to_estimate_scale_) {
    gyro_info_.scale_summary_message = "Skipped scale update (yaw rate is too small)";
    if (
      gyro_info_.scale_status != diagnostic_msgs::msg::DiagnosticStatus::ERROR &&
      gyro_info_.scale_status != diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
      gyro_info_.scale_status_summary = "OK";
    }
    geometry_msgs::msg::Vector3Stamped vector_scale_skipped;
    vector_scale_skipped.header.stamp = this->now();
    // Scale on x , y axis is not estimated, but set to 1.0 for consistency
    vector_scale_skipped.vector.x = 1.0;
    vector_scale_skipped.vector.y = 1.0;
    vector_scale_skipped.vector.z = filtered_scale_rate_;
    gyro_scale_pub_->publish(vector_scale_skipped);
    big_change_detect_ = 0;
    rate_pose_buff_.clear();

    return;
  }

  if (gyro_bias_.has_value() && gyro_buf_.size() > static_cast<size_t>(buffer_size_gyro_ - 2)) {
    // EKF update
    // Find in the buffer the delayed gyro data and average the last samples
    const std::vector<geometry_msgs::msg::Vector3Stamped> gyro_local_buf = gyro_buf_;
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
      } else {
        start_it = gyro_local_buf.begin();
      }
      double sum_gyro_delayed_rate = 0.0;
      for (auto it = start_it; it != closest_delayed_it; ++it) {
        sum_gyro_delayed_rate += it->vector.z;
      }
      avg_rate_gyro_ = std::abs(sum_gyro_delayed_rate / samples_filter_gyro_rate_);
    } else {
      avg_rate_gyro_ = 0.0;
    }

    rate_pose_buff_.push_back(ndt_yaw_rate_);

    // Calculate the EKF if enough samples are available
    if (rate_pose_buff_.size() > static_cast<size_t>(samples_filter_pose_rate_)) {
      avg_rate_pose_ = std::abs(
        std::accumulate(rate_pose_buff_.begin(), rate_pose_buff_.end(), 0.0) /
        rate_pose_buff_.size());
      // Keep the buffer size fixed
      rate_pose_buff_.erase(rate_pose_buff_.begin());

      h_ = avg_rate_pose_;
      y_ = avg_rate_gyro_ - (estimated_scale_ * avg_rate_pose_ + gyro_bias_.value().z);
      s_ = h_ * p_ * h_ + r_;
      k_ = p_ * h_ / s_;
      estimated_scale_ = estimated_scale_ + k_ * y_;
      p_ = (1 - k_ * h_) * p_;

      Vector3Stamped gyro_debug_msg;
      gyro_debug_msg.header.stamp = this->now();
      gyro_debug_msg.vector.x = avg_rate_pose_;
      gyro_debug_msg.vector.y = avg_rate_gyro_;
      gyro_debug_msg.vector.z = 1.0;
      gyro_debug_pub_->publish(gyro_debug_msg);

      // Initialize the slow phase after the first samples
      if (!filtered_scale_initialized_) {
        estimated_scale_buff_.push_back(estimated_scale_);
        if (estimated_scale_buff_.size() > static_cast<size_t>(samples_to_init_)) {
          double average_est_scale =
            std::accumulate(estimated_scale_buff_.begin(), estimated_scale_buff_.end(), 0.0) /
            static_cast<double>(estimated_scale_buff_.size());
          filtered_scale_rate_ = average_est_scale;
          estimated_scale_buff_.clear();
          // Update parameters
          ekf_variance_ = ekf_variance_p_after_;
          r_ = ekf_measurement_noise_r_after_ * ekf_measurement_noise_r_after_;
          q_ = ekf_process_noise_q_after_;
          filtered_scale_initialized_ = true;
        }
      }

      // Check if the estimated scale is within the allowed range or if a big change is detected
      if (
        estimated_scale_ >= filtered_scale_rate_ * (1 - percentage_scale_rate_allow_correct_) &&
        estimated_scale_ <= filtered_scale_rate_ * (1 + percentage_scale_rate_allow_correct_)) {
        filtered_scale_rate_ = (alpha_)*filtered_scale_rate_ + (1 - alpha_) * estimated_scale_;
        big_change_detect_ = 0;
      } else {
        big_change_detect_++;
        int counter_correct_big_change = 30;  // 30 iterations approx 3 seconds
        if (big_change_detect_ >= counter_correct_big_change) {
          big_change_detect_ = 0;
          gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          gyro_info_.scale_status_summary = "ERR";
          gyro_info_.scale_summary_message =
            "Scale estimated is over the maximum allowed, check the IMU, NDT device or TF.";
        }
      }

      geometry_msgs::msg::Vector3Stamped vector_scale;

      if (estimated_scale_ < min_allowed_scale_) {
        gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        gyro_info_.scale_status_summary = "ERR";
        gyro_info_.scale_summary_message =
          "Scale is under the minimum, check the IMU, NDT device or TF.";
      } else if (estimated_scale_ > max_allowed_scale_) {
        gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        gyro_info_.scale_status_summary = "ERR";
        gyro_info_.scale_summary_message =
          "Scale is over the maximum, check the IMU, NDT device or TF.";
      } else {
        gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
      }

      vector_scale.header.stamp = this->now();

      // Scale on x , y axis is not estimated, but set to 1.0 for consistency
      vector_scale.vector.x = 1.0;
      vector_scale.vector.y = 1.0;
      vector_scale.vector.z = filtered_scale_rate_;
      gyro_scale_pub_->publish(vector_scale);

      diagnostics_info_.estimated_gyro_scale_z = estimated_scale_;
      scale_list_all_.push_back(filtered_scale_rate_);
    }
  }
}

void GyroBiasEstimator::timer_callback()
{
  if (pose_buf_.empty()) {
    gyro_info_.bias_summary_message = "Skipped update (pose_buf is empty).";
    diagnostics_info_.summary_message = "Skipped update (pose_buf is empty)";
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
    diagnostics_info_.summary_message = "Skipped update (pose_buf is not in chronological order)";
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
    diagnostics_info_.summary_message = "Skipped update (gyro_filtered size is less than 2)";
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
    diagnostics_info_.summary_message =
      "Skipped update (yaw angular velocity is greater than straight_motion_ang_vel_upper_limit)";
    return;
  }

  // Calculate gyro bias
  gyro_bias_estimation_module_->update_bias(pose_buf, gyro_filtered);

  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_base2imu_ptr =
    transform_listener_->get_latest_transform(output_frame_, imu_frame_);
  if (!tf_base2imu_ptr) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", imu_frame_.c_str(), output_frame_.c_str());
    gyro_info_.bias_summary_message = "Skipped update (tf between base and imu is not available).";
    diagnostics_info_.summary_message = "Skipped update (tf between base and imu is not available)";
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
  stat.add("samples_to_average_delta", (samples_to_average_delta_));
  stat.add("samples_to_init", (samples_to_init_));
  stat.add("samples_to_average_delta", (samples_to_average_delta_));

  if (gyro_bias_.has_value()) {
    stat.add("gyro_yaw_rate_", f(gyro_yaw_rate_));  // for testing removed bias
    stat.add("ndt_yaw_rate_", f(ndt_yaw_rate_));
    stat.add("gyro_angle", f(gyro_yaw_angle_));
    stat.add("lidar_angle", f(ndt_yaw_angle_));

    stat.add("h_", f(h_));
    stat.add("s_", f(s_));
    stat.add("k_", f(k_));
    stat.add("y_", f(y_));
  }

  stat.add("bias_on_purpose", f(final_bias_on_purpose_));
  stat.add("scale_on_purpose", f(final_scale_on_purpose_));

  stat.add("gyro_bias_threshold", f(gyro_bias_threshold_));
  stat.add("p_", f(p_ * 1e10));
  stat.add("q_", f(q_ * 1e10));
}

}  // namespace autoware::imu_corrector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::imu_corrector::GyroBiasEstimator)
