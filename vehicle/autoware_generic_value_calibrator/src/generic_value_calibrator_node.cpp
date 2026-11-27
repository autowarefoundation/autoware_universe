//
// Copyright 2025 TIER IV, Inc. All rights reserved.
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
//

#include "autoware_generic_value_calibrator/generic_value_calibrator_node.hpp"

#include "rclcpp/logging.hpp"
#include "tf2/utils.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace autoware::generic_value_calibrator
{

GenericValueCalibrator::GenericValueCalibrator(const rclcpp::NodeOptions & node_options)
: Node("generic_value_calibrator", node_options)
{
  RCLCPP_INFO(get_logger(), "=== Generic Value Calibrator Initializing ===");

  // get parameter
  update_hz_ = declare_parameter<double>("update_hz", 10.0);
  covariance_ = declare_parameter<double>("initial_covariance", 0.05);
  velocity_min_threshold_ = declare_parameter<double>("velocity_min_threshold", 0.1);
  velocity_diff_threshold_ = declare_parameter<double>("velocity_diff_threshold", 0.556);
  value_diff_threshold_ = declare_parameter<double>("value_diff_threshold", 0.03);
  max_steer_threshold_ = declare_parameter<double>("max_steer_threshold", 0.2);
  max_pitch_threshold_ = declare_parameter<double>("max_pitch_threshold", 0.02);
  max_jerk_threshold_ = declare_parameter<double>("max_jerk_threshold", 0.7);
  value_velocity_thresh_ = declare_parameter<double>("value_velocity_thresh", 0.15);
  max_accel_ = declare_parameter<double>("max_accel", 5.0);
  min_accel_ = declare_parameter<double>("min_accel", -5.0);
  value_to_accel_delay_ = declare_parameter<double>("value_to_accel_delay", 0.3);
  max_data_count_ = static_cast<int>(declare_parameter("max_data_count", 200));
  progress_file_output_ = declare_parameter<bool>("progress_file_output", false);
  precision_ = static_cast<int>(declare_parameter("precision", 3));

  const auto get_pitch_method_str = declare_parameter("get_pitch_method", std::string("tf"));
  if (get_pitch_method_str == std::string("tf")) {
    get_pitch_method_ = GET_PITCH_METHOD::TF;
    // Only initialize transform_listener when needed
    transform_listener_ = std::make_shared<autoware_utils::TransformListener>(this);
  } else if (get_pitch_method_str == std::string("none")) {
    get_pitch_method_ = GET_PITCH_METHOD::NONE;
  } else {
    RCLCPP_ERROR_STREAM(get_logger(), "get_pitch_method is wrong. (available method: tf, none)");
    return;
  }

  update_suggest_thresh_ = declare_parameter<double>("update_suggest_thresh", 0.7);
  csv_calibrated_map_dir_ = declare_parameter("csv_calibrated_map_dir", std::string(""));
  output_map_file_ = csv_calibrated_map_dir_ + "/value_map.csv";

  const std::string update_method_str =
    declare_parameter("update_method", std::string("update_offset_each_cell"));
  if (update_method_str == std::string("update_offset_each_cell")) {
    update_method_ = UPDATE_METHOD::UPDATE_OFFSET_EACH_CELL;
  } else if (update_method_str == std::string("update_offset_total")) {
    update_method_ = UPDATE_METHOD::UPDATE_OFFSET_TOTAL;
  } else {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "update_method is wrong. (available method: update_offset_each_cell, update_offset_total)");
    return;
  }

  // QoS setup
  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS durable_qos(queue_size);

  /* Diagnostic Updater */
  updater_ptr_ = std::make_shared<diagnostic_updater::Updater>(this, 1.0 / update_hz_);
  updater_ptr_->setHardwareID("generic_value_calibrator");
  updater_ptr_->add(
    "generic_value_calibrator", this, &GenericValueCalibrator::check_update_suggest);

  // Initialize map from CSV or create default map
  csv_default_map_dir_ = declare_parameter("csv_default_map_dir", std::string(""));

  // Get map index parameters (used if no CSV is provided)
  const double value_min = declare_parameter("value_min", -1.0);
  const double value_max = declare_parameter("value_max", 1.0);
  const int value_num = declare_parameter("value_num", 11);
  const double velocity_min = declare_parameter("velocity_min", 0.0);
  const double velocity_max = declare_parameter("velocity_max", 20.0);
  const int velocity_num = declare_parameter("velocity_num", 11);

  bool map_loaded_from_csv = false;

  // Try to load map from CSV if path is provided
  if (!csv_default_map_dir_.empty()) {
    std::ifstream csv_file(csv_default_map_dir_);
    if (csv_file.is_open()) {
      std::vector<std::vector<std::string>> csv_table;
      std::string line;
      while (std::getline(csv_file, line)) {
        std::istringstream iss(line);
        std::string cell;
        std::vector<std::string> row;
        while (std::getline(iss, cell, ',')) {
          row.push_back(cell);
        }
        csv_table.push_back(row);
      }
      csv_file.close();

      if (!csv_table.empty() && csv_table.size() > 1) {
        // Extract column indices (velocities) from first row
        vel_index_.clear();
        for (size_t i = 1; i < csv_table[0].size(); ++i) {
          vel_index_.push_back(std::stod(csv_table[0][i]));
        }

        // Extract row indices (values) from first column
        value_index_.clear();
        for (size_t i = 1; i < csv_table.size(); ++i) {
          value_index_.push_back(std::stod(csv_table[i][0]));
        }

        // Extract map data
        value_map_.clear();
        for (size_t i = 1; i < csv_table.size(); ++i) {
          std::vector<double> row;
          for (size_t j = 1; j < csv_table[i].size(); ++j) {
            row.push_back(std::stod(csv_table[i][j]));
          }
          value_map_.push_back(row);
        }

        map_loaded_from_csv = true;
        RCLCPP_INFO(get_logger(), "Loaded map from CSV: %s", csv_default_map_dir_.c_str());
        RCLCPP_INFO(
          get_logger(), "  Value range from CSV: [%.3f, %.3f] with %zu points",
          value_index_.front(), value_index_.back(), value_index_.size());
        RCLCPP_INFO(
          get_logger(), "  Velocity range from CSV: [%.3f, %.3f] with %zu points",
          vel_index_.front(), vel_index_.back(), vel_index_.size());
      } else {
        RCLCPP_WARN(get_logger(), "CSV file is empty or invalid: %s", csv_default_map_dir_.c_str());
      }
    } else {
      RCLCPP_WARN(get_logger(), "Failed to open CSV file: %s", csv_default_map_dir_.c_str());
    }
  }

  // Generate indices from parameters if CSV was not loaded
  if (!map_loaded_from_csv) {
    // Generate value_index from parameters
    value_index_.clear();
    if (value_num == 1) {
      value_index_.push_back(value_min);
    } else {
      const double value_step = (value_max - value_min) / (value_num - 1);
      for (int i = 0; i < value_num; ++i) {
        value_index_.push_back(value_min + i * value_step);
      }
    }

    // Generate vel_index from parameters
    vel_index_.clear();
    if (velocity_num == 1) {
      vel_index_.push_back(velocity_min);
    } else {
      const double vel_step = (velocity_max - velocity_min) / (velocity_num - 1);
      for (int i = 0; i < velocity_num; ++i) {
        vel_index_.push_back(velocity_min + i * vel_step);
      }
    }

    // Initialize map with default acceleration values (simple linear relationship)
    value_map_.clear();
    value_map_.resize(value_index_.size());
    for (size_t i = 0; i < value_index_.size(); ++i) {
      value_map_[i].resize(vel_index_.size());
      // Default: acceleration = input_value (simple linear mapping)
      const double default_accel = value_index_[i];
      for (size_t j = 0; j < vel_index_.size(); ++j) {
        value_map_[i][j] = default_accel;
      }
    }
    RCLCPP_INFO(get_logger(), "Generated default map from parameters");
  }

  // Initialize update map and covariance matrices
  update_value_map_.resize(value_index_.size());
  for (auto & m : update_value_map_) {
    m.resize(vel_index_.size(), 0.0);
  }

  value_offset_covariance_value_.resize(value_index_.size());
  for (auto & m : value_offset_covariance_value_) {
    m.resize(vel_index_.size(), covariance_);
  }

  // Copy initial map to update map
  std::copy(value_map_.begin(), value_map_.end(), update_value_map_.begin());

  // initialize matrix for covariance calculation
  {
    const auto gen_const_mat = [](const Map & map, const auto val) {
      return Eigen::MatrixXd::Constant(map.size(), map.at(0).size(), val);
    };
    data_mean_mat_ = gen_const_mat(value_map_, map_offset_);
    data_covariance_mat_ = gen_const_mat(value_map_, covariance_);
    data_num_ = gen_const_mat(value_map_, 1);
  }

  // publisher
  update_suggest_pub_ =
    create_publisher<std_msgs::msg::Bool>("~/output/update_suggest", durable_qos);
  current_map_error_pub_ =
    create_publisher<Float64Stamped>("~/output/current_map_error", durable_qos);
  updated_map_error_pub_ =
    create_publisher<Float64Stamped>("~/output/updated_map_error", durable_qos);
  map_error_ratio_pub_ = create_publisher<Float64Stamped>("~/output/map_error_ratio", durable_qos);

  // Debug/Visualization publishers
  original_map_occ_pub_ = create_publisher<OccupancyGrid>("~/debug/original_occ_map", durable_qos);
  update_map_occ_pub_ = create_publisher<OccupancyGrid>("~/debug/update_occ_map", durable_qos);
  data_ave_pub_ = create_publisher<OccupancyGrid>("~/debug/data_average_occ_map", durable_qos);
  data_std_pub_ = create_publisher<OccupancyGrid>("~/debug/data_std_dev_occ_map", durable_qos);
  data_count_pub_ = create_publisher<OccupancyGrid>("~/debug/data_count_occ_map", durable_qos);
  data_count_with_self_pose_pub_ =
    create_publisher<OccupancyGrid>("~/debug/data_count_self_pose_occ_map", durable_qos);
  index_pub_ = create_publisher<MarkerArray>("~/debug/occ_index", durable_qos);
  original_map_raw_pub_ =
    create_publisher<Float32MultiArray>("~/debug/original_raw_map", durable_qos);
  update_map_raw_pub_ = create_publisher<Float32MultiArray>("~/output/update_raw_map", durable_qos);

  // output log file
  std::string output_log_file = declare_parameter("output_log_file", std::string(""));
  if (!output_log_file.empty()) {
    output_log_.open(output_log_file);
    if (output_log_.is_open()) {
      add_index_to_csv(&output_log_);
      RCLCPP_INFO(get_logger(), "Logging calibration data to: %s", output_log_file.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to open log file: %s", output_log_file.c_str());
    }
  }

  // timer
  init_timer(1.0 / update_hz_);
  init_output_csv_timer(30.0);

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);

  // Print important parameters
  RCLCPP_INFO(get_logger(), "=== Map Index Configuration ===");
  RCLCPP_INFO(
    get_logger(), "  Input value range: [%.3f, %.3f] with %d points", value_min, value_max,
    value_num);
  RCLCPP_INFO(
    get_logger(), "  Velocity range: [%.3f, %.3f] m/s with %d points", velocity_min, velocity_max,
    velocity_num);
  RCLCPP_INFO(
    get_logger(), "  Map size: %zu x %zu (value x velocity)", value_index_.size(),
    vel_index_.size());
  RCLCPP_INFO(get_logger(), "=== Calibration Parameters ===");
  RCLCPP_INFO(get_logger(), "  Update Hz: %.1f", update_hz_);
  RCLCPP_INFO(get_logger(), "  Velocity min threshold: %.3f m/s", velocity_min_threshold_);
  RCLCPP_INFO(get_logger(), "  Max steer threshold: %.3f rad", max_steer_threshold_);
  RCLCPP_INFO(get_logger(), "  Max pitch threshold: %.3f rad", max_pitch_threshold_);
  RCLCPP_INFO(get_logger(), "  Max jerk threshold: %.3f m/s^3", max_jerk_threshold_);
  RCLCPP_INFO(get_logger(), "  Value to accel delay: %.3f s", value_to_accel_delay_);
  RCLCPP_INFO(get_logger(), "=== Subscribed Topics ===");
  RCLCPP_INFO(get_logger(), "  Input value: ~/input/value (Float64Stamped)");
  RCLCPP_INFO(get_logger(), "  Velocity: ~/input/velocity (VelocityReport)");
  RCLCPP_INFO(get_logger(), "  Steering: ~/input/steer (SteeringReport)");
  RCLCPP_INFO(get_logger(), "=== Output ===");
  RCLCPP_INFO(get_logger(), "  Map file: %s", output_map_file_.c_str());
  if (!output_log_file.empty()) {
    RCLCPP_INFO(get_logger(), "  Log file: %s", output_log_file.c_str());
  }
  RCLCPP_INFO(get_logger(), "=== GenericValueCalibrator Ready! ===");
  RCLCPP_INFO(get_logger(), "Waiting for input topics...");
}

void GenericValueCalibrator::init_output_csv_timer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_output_csv_ = rclcpp::create_timer(
    this, get_clock(), period_ns,
    std::bind(&GenericValueCalibrator::timer_callback_output_csv, this));
}

void GenericValueCalibrator::init_timer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&GenericValueCalibrator::fetch_data, this));
}

bool GenericValueCalibrator::get_current_pitch_from_tf(double * pitch)
{
  if (get_pitch_method_ == GET_PITCH_METHOD::NONE) {
    *pitch = 0.0;
    return true;
  }

  const auto transform = transform_listener_->get_transform(
    "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
  if (!transform) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 5000, "cannot get map to base_link transform. ");
    return false;
  }
  double roll = 0.0;
  double raw_pitch = 0.0;
  double yaw = 0.0;
  tf2::getEulerYPR(transform->transform.rotation, yaw, raw_pitch, roll);
  *pitch = lowpass(*pitch, raw_pitch, 0.2);
  return true;
}

bool GenericValueCalibrator::take_data()
{
  // take input value data
  Float64Stamped::ConstSharedPtr input_value_msg = input_value_sub_.take_data();
  if (!input_value_msg) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 10000, "No input_value message received. Topic: ~/input/value");
    return false;
  }
  take_input_value(input_value_msg);

  // take velocity data
  VelocityReport::ConstSharedPtr velocity_ptr = velocity_sub_.take_data();
  if (!velocity_ptr) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 10000, "No velocity message received. Topic: ~/input/velocity");
    return false;
  }
  take_velocity(velocity_ptr);

  // take steer data
  steer_ptr_ = steer_sub_.take_data();
  if (!steer_ptr_) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 10000, "No steer message received. Topic: ~/input/steer");
  }

  /* valid check */
  if (!twist_ptr_ || !steer_ptr_ || !input_value_ptr_ || !delayed_input_value_ptr_) {
    std::stringstream ss;
    ss << "Lack of required data - ";
    if (!twist_ptr_) ss << "twist ";
    if (!steer_ptr_) ss << "steer ";
    if (!input_value_ptr_) ss << "input_value ";
    if (!delayed_input_value_ptr_) ss << "delayed_input_value ";
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, ss.str());
    lack_of_data_count_++;
    return false;
  }

  RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 30000, "All data successfully received");
  return true;
}

void GenericValueCalibrator::fetch_data()
{
  update_count_++;

  RCLCPP_DEBUG_STREAM_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "map updating... count: " << update_success_count_ << " / " << update_count_);

  // if cannot get data, return this callback
  if (!take_data()) return;

  // data check - timeout
  if (
    is_timeout(twist_ptr_->header.stamp, timeout_sec_) ||
    is_timeout(steer_ptr_->stamp, timeout_sec_) || is_timeout(input_value_ptr_, timeout_sec_)) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 5000, "timeout of topics (twist, steer, input_value)");
    lack_of_data_count_++;
    return;
  }

  // get pitch
  if (!get_current_pitch_from_tf(&pitch_)) {
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 5000, "cannot get pitch");
    failed_to_get_pitch_count_++;
    return;
  }

  /* write data to log if enabled */
  if (output_log_.is_open() && delayed_input_value_ptr_) {
    output_log_ << rclcpp::Time(twist_ptr_->header.stamp).seconds() << ","
                << twist_ptr_->twist.linear.x << "," << acceleration_ << ","
                << get_pitch_compensated_acceleration() << "," << delayed_input_value_ptr_->data
                << "," << input_value_speed_ << "," << pitch_ << ","
                << steer_ptr_->steering_tire_angle << "," << jerk_ << "," << part_original_rmse_
                << "," << new_rmse_ << ","
                << (part_original_rmse_ != 0.0 ? new_rmse_ / part_original_rmse_ : 1.0)
                << std::endl;
  }

  /* publish error metrics */
  publish_update_suggest_flag();
  publish_float64("current_map_error", part_original_rmse_);
  publish_float64("updated_map_error", new_rmse_);
  publish_float64(
    "map_error_ratio", part_original_rmse_ != 0.0 ? new_rmse_ / part_original_rmse_ : 1.0);

  /* initialize */
  update_success_ = false;

  // velocity check
  if (twist_ptr_->twist.linear.x < velocity_min_threshold_) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 10000, "Velocity too low: %.3f < %.3f m/s",
      twist_ptr_->twist.linear.x, velocity_min_threshold_);
    too_low_speed_count_++;
    return;
  }

  // evaluation
  execute_evaluation();

  // pitch check
  if (std::fabs(pitch_) > max_pitch_threshold_) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 10000, "Pitch too large: %.3f > %.3f rad", std::fabs(pitch_),
      max_pitch_threshold_);
    too_large_pitch_count_++;
    return;
  }

  // steer check
  if (std::fabs(steer_ptr_->steering_tire_angle) > max_steer_threshold_) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 10000, "Steering angle too large: %.3f > %.3f rad",
      std::fabs(steer_ptr_->steering_tire_angle), max_steer_threshold_);
    too_large_steer_count_++;
    return;
  }

  // jerk check
  if (std::fabs(jerk_) > max_jerk_threshold_) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 10000, "Jerk too large: %.3f > %.3f m/s^3", std::fabs(jerk_),
      max_jerk_threshold_);
    too_large_jerk_count_++;
    return;
  }

  // value speed check
  if (std::fabs(input_value_speed_) > value_velocity_thresh_) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 10000, "Input value speed too large: %.3f > %.3f",
      std::fabs(input_value_speed_), value_velocity_thresh_);
    too_large_value_spd_count_++;
    return;
  }

  RCLCPP_DEBUG_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "All checks passed! Attempting map update with vel=%.3f m/s, value=%.3f, accel=%.3f m/s^2",
    twist_ptr_->twist.linear.x, delayed_input_value_ptr_->data,
    get_pitch_compensated_acceleration());

  /* update map */
  if (update_value_map()) {
    update_success_count_++;
    update_success_ = true;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "✓ Map update SUCCESS! Total: %d/%d (%.1f%%)",
      update_success_count_, update_count_,
      100.0 * update_success_count_ / std::max(1, update_count_));
  } else {
    update_fail_count_++;
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 10000, "Map update failed (cell might be out of bounds)");
  }
}

void GenericValueCalibrator::timer_callback_output_csv()
{
  write_map_to_csv(vel_index_, value_index_, update_value_map_, output_map_file_);

  // Publish visualization data
  publish_map(value_map_, "original");  // value_map_ is the default/original map
  publish_map(update_value_map_, "update");
  publish_count_map();
  publish_index();
}

void GenericValueCalibrator::take_velocity(const VelocityReport::ConstSharedPtr msg)
{
  auto twist_msg = std::make_shared<TwistStamped>();
  twist_msg->header = msg->header;
  twist_msg->twist.linear.x = msg->longitudinal_velocity;

  if (!twist_vec_.empty()) {
    const auto past_msg = get_nearest_time_data_from_vec(twist_msg, dif_twist_time_, twist_vec_);
    const double raw_acceleration = get_accel(past_msg, twist_msg);
    acceleration_ = lowpass(acceleration_, raw_acceleration, 0.25);
    acceleration_time_ = rclcpp::Time(msg->header.stamp).seconds();

    // calculate jerk
    if (
      this->now().seconds() - pre_acceleration_time_ > timeout_sec_ ||
      (acceleration_time_ - pre_acceleration_time_) <= std::numeric_limits<double>::epsilon()) {
      // does not update jerk
    } else {
      const double raw_jerk = get_jerk();
      jerk_ = lowpass(jerk_, raw_jerk, 0.5);
    }
    pre_acceleration_ = acceleration_;
    pre_acceleration_time_ = acceleration_time_;
  }

  twist_ptr_ = twist_msg;
  push_data_to_vec(twist_msg, twist_vec_max_size_, &twist_vec_);
}

void GenericValueCalibrator::take_input_value(const Float64Stamped::ConstSharedPtr msg)
{
  input_value_ptr_ = std::make_shared<DataStamped>(msg->data, rclcpp::Time(msg->stamp));

  if (!input_value_vec_.empty()) {
    const auto past_value_ptr =
      get_nearest_time_data_from_vec(input_value_ptr_, dif_value_time_, input_value_vec_);
    const double raw_value_speed =
      get_value_speed(past_value_ptr, input_value_ptr_, input_value_speed_);
    input_value_speed_ = lowpass(input_value_speed_, raw_value_speed, 0.5);
  }

  push_data_to_vec(input_value_ptr_, value_vec_max_size_, &input_value_vec_);
  delayed_input_value_ptr_ =
    get_nearest_time_data_from_vec(input_value_ptr_, value_to_accel_delay_, input_value_vec_);
}

double GenericValueCalibrator::lowpass(
  const double original, const double current, const double gain)
{
  return current * gain + original * (1.0 - gain);
}

double GenericValueCalibrator::get_value_speed(
  const DataStampedPtr & prev_value, const DataStampedPtr & current_value,
  const double prev_value_speed)
{
  const double dt = (current_value->data_time - prev_value->data_time).seconds();
  if (dt < 1e-03) {
    return prev_value_speed;
  }

  const double d_value = current_value->data - prev_value->data;
  return d_value / dt;
}

double GenericValueCalibrator::get_accel(
  const TwistStamped::ConstSharedPtr & prev_twist,
  const TwistStamped::ConstSharedPtr & current_twist) const
{
  const double dt =
    (rclcpp::Time(current_twist->header.stamp) - rclcpp::Time(prev_twist->header.stamp)).seconds();
  if (dt < 1e-03) {
    return acceleration_;
  }
  const double dv = current_twist->twist.linear.x - prev_twist->twist.linear.x;
  return std::min(std::max(min_accel_, dv / dt), max_accel_);
}

double GenericValueCalibrator::get_jerk()
{
  const double jerk =
    (acceleration_ - pre_acceleration_) / (acceleration_time_ - pre_acceleration_time_);
  const double max_jerk = 5.0;
  return std::min(std::max(-max_jerk, jerk), max_jerk);
}

bool GenericValueCalibrator::index_value_search(
  const std::vector<double> & value_index, const double value, const double value_thresh,
  int * searched_index) const
{
  for (std::size_t i = 0; i < value_index.size(); i++) {
    const double diff_value = std::fabs(value_index.at(i) - value);
    if (diff_value <= value_thresh) {
      *searched_index = static_cast<int>(i);
      return true;
    }
  }
  return false;
}

int GenericValueCalibrator::nearest_value_search(
  const std::vector<double> & value_index, const double value)
{
  double max_dist = std::numeric_limits<double>::max();
  int nearest_idx = 0;

  for (std::size_t i = 0; i < value_index.size(); i++) {
    const double dist = std::fabs(value - value_index.at(i));
    if (max_dist > dist) {
      nearest_idx = static_cast<int>(i);
      max_dist = dist;
    }
  }
  return nearest_idx;
}

void GenericValueCalibrator::take_consistency_of_value_map()
{
  const double bit = std::pow(1e-01, precision_);
  for (std::size_t val_idx = 0; val_idx < update_value_map_.size() - 1; val_idx++) {
    for (std::size_t vel_idx = update_value_map_.at(0).size() - 1;; vel_idx--) {
      if (vel_idx == 0) break;

      const double current_acc = update_value_map_.at(val_idx).at(vel_idx);
      const double next_val_acc = update_value_map_.at(val_idx + 1).at(vel_idx);
      const double prev_vel_acc = update_value_map_.at(val_idx).at(vel_idx - 1);

      // Ensure monotonic relationship with velocity
      if (current_acc + bit >= prev_vel_acc) {
        update_value_map_.at(val_idx).at(vel_idx - 1) = current_acc + bit;
      }

      // Ensure monotonic relationship with value
      if (current_acc + bit >= next_val_acc) {
        update_value_map_.at(val_idx + 1).at(vel_idx) = current_acc + bit;
      }
    }
  }
}

bool GenericValueCalibrator::update_value_map()
{
  int value_index = 0;
  int vel_index = 0;

  if (!index_value_search(
        value_index_, delayed_input_value_ptr_->data, value_diff_threshold_, &value_index)) {
    return false;
  }

  if (!index_value_search(
        vel_index_, twist_ptr_->twist.linear.x, velocity_diff_threshold_, &vel_index)) {
    return false;
  }

  // update map
  execute_update(value_index, vel_index);

  // take consistency of map
  take_consistency_of_value_map();

  return true;
}

void GenericValueCalibrator::execute_update(const int value_index, const int vel_index)
{
  const double measured_acc = acceleration_ - get_pitch_compensated_acceleration();
  const double map_acc = update_value_map_.at(value_index).at(vel_index);

  if (update_method_ == UPDATE_METHOD::UPDATE_OFFSET_EACH_CELL) {
    update_each_val_offset(value_index, vel_index, measured_acc, map_acc);
  } else if (update_method_ == UPDATE_METHOD::UPDATE_OFFSET_TOTAL) {
    update_total_map_offset(measured_acc, map_acc);
    // Still update statistics for the current cell to track data coverage
    update_statistics(value_index, vel_index, measured_acc);
  }
}

bool GenericValueCalibrator::update_each_val_offset(
  const int value_index, const int vel_index, const double measured_acc, const double map_acc)
{
  // pre-defined static variables
  static Map map_offset_vec(
    value_map_.size(), std::vector<double>(value_map_.at(0).size(), map_offset_));
  static Map covariance_vec(
    value_map_.size(), std::vector<double>(value_map_.at(0).size(), covariance_));

  double map_offset = map_offset_vec.at(value_index).at(vel_index);
  double covariance = covariance_vec.at(value_index).at(vel_index);

  /* RLS update */
  const double phi = 1.0;
  covariance = (covariance - (covariance * phi * phi * covariance) /
                               (forgetting_factor_ + phi * covariance * phi)) /
               forgetting_factor_;

  const double coef = (covariance * phi) / (forgetting_factor_ + phi * covariance * phi);

  const double error_map_offset = measured_acc - map_acc;
  map_offset = map_offset + coef * error_map_offset;

  /* Update statistics using Welford's online algorithm */
  const double current_count = data_num_(value_index, vel_index);
  const double pre_mean = data_mean_mat_(value_index, vel_index);
  const double pre_variance = data_covariance_mat_(value_index, vel_index);

  // Update mean
  const double new_mean = (current_count * pre_mean + measured_acc) / (current_count + 1);

  // Update variance using Welford's method
  const double new_variance =
    (current_count * (pre_variance + pre_mean * pre_mean) + measured_acc * measured_acc) /
      (current_count + 1) -
    new_mean * new_mean;

  // Update count
  data_num_(value_index, vel_index) = current_count + 1;
  data_mean_mat_(value_index, vel_index) = new_mean;
  data_covariance_mat_(value_index, vel_index) = new_variance;

  /* update map */
  map_offset_vec.at(value_index).at(vel_index) = map_offset;
  covariance_vec.at(value_index).at(vel_index) = covariance;
  update_value_map_.at(value_index).at(vel_index) =
    value_map_.at(value_index).at(vel_index) + map_offset;

  return true;
}

void GenericValueCalibrator::update_total_map_offset(
  const double measured_acc, const double map_acc)
{
  /* RLS update */
  const double phi = 1.0;
  covariance_ = (covariance_ - (covariance_ * phi * phi * covariance_) /
                                 (forgetting_factor_ + phi * covariance_ * phi)) /
                forgetting_factor_;

  const double coef = (covariance_ * phi) / (forgetting_factor_ + phi * covariance_ * phi);
  const double error_map_offset = measured_acc - map_acc;
  map_offset_ = map_offset_ + coef * error_map_offset;

  /* update entire map */
  for (std::size_t val_idx = 0; val_idx < update_value_map_.size(); val_idx++) {
    for (std::size_t vel_idx = 0; vel_idx < update_value_map_.at(0).size(); vel_idx++) {
      update_value_map_.at(val_idx).at(vel_idx) = value_map_.at(val_idx).at(vel_idx) + map_offset_;
    }
  }
}

void GenericValueCalibrator::update_statistics(
  const int value_index, const int vel_index, const double measured_acc)
{
  /* Update statistics using Welford's online algorithm */
  const double current_count = data_num_(value_index, vel_index);
  const double pre_mean = data_mean_mat_(value_index, vel_index);
  const double pre_variance = data_covariance_mat_(value_index, vel_index);

  // Update mean
  const double new_mean = (current_count * pre_mean + measured_acc) / (current_count + 1);

  // Update variance using Welford's method
  const double new_variance =
    (current_count * (pre_variance + pre_mean * pre_mean) + measured_acc * measured_acc) /
      (current_count + 1) -
    new_mean * new_mean;

  // Update count
  data_num_(value_index, vel_index) = current_count + 1;
  data_mean_mat_(value_index, vel_index) = new_mean;
  data_covariance_mat_(value_index, vel_index) = new_variance;
}

double GenericValueCalibrator::get_pitch_compensated_acceleration() const
{
  constexpr double gravity = 9.80665;
  return gravity * std::sin(pitch_);
}

void GenericValueCalibrator::execute_evaluation()
{
  const double part_orig_sq_error =
    calculate_value_squared_error(delayed_input_value_ptr_->data, twist_ptr_->twist.linear.x);
  push_data_to_vec(part_orig_sq_error, part_mse_que_size_, &part_original_mse_que_);
  part_original_rmse_ = std::sqrt(get_average(part_original_mse_que_));

  // Calculate error using updated map
  const double new_sq_error = calculate_updated_value_squared_error(
    delayed_input_value_ptr_->data, twist_ptr_->twist.linear.x);
  push_data_to_vec(new_sq_error, part_mse_que_size_, &new_mse_que_);
  new_rmse_ = std::sqrt(get_average(new_mse_que_));
}

double GenericValueCalibrator::calculate_value_squared_error(const double value, const double vel)
{
  // Find nearest indices
  const int val_idx = nearest_value_search(value_index_, value);
  const int vel_idx = nearest_value_search(vel_index_, vel);

  const double estimated_acc = value_map_.at(val_idx).at(vel_idx);
  const double measured_acc = acceleration_ - get_pitch_compensated_acceleration();
  const double dif_acc = measured_acc - estimated_acc;
  return dif_acc * dif_acc;
}

double GenericValueCalibrator::calculate_updated_value_squared_error(
  const double value, const double vel)
{
  // Find nearest indices
  const int val_idx = nearest_value_search(value_index_, value);
  const int vel_idx = nearest_value_search(vel_index_, vel);

  const double estimated_acc = update_value_map_.at(val_idx).at(vel_idx);
  const double measured_acc = acceleration_ - get_pitch_compensated_acceleration();
  const double dif_acc = measured_acc - estimated_acc;
  return dif_acc * dif_acc;
}

template <class T>
void GenericValueCalibrator::push_data_to_vec(
  const T data, const std::size_t max_size, std::vector<T> * vec)
{
  vec->emplace_back(data);
  while (vec->size() > max_size) {
    vec->erase(vec->begin());
  }
}

template <class T>
T GenericValueCalibrator::get_nearest_time_data_from_vec(
  const T base_data, const double back_time, const std::vector<T> & vec)
{
  double nearest_time = std::numeric_limits<double>::max();
  const double target_time = rclcpp::Time(base_data->header.stamp).seconds() - back_time;
  T nearest_time_data;
  for (const auto & data : vec) {
    const double data_time = rclcpp::Time(data->header.stamp).seconds();
    const auto delta_time = std::abs(target_time - data_time);
    if (nearest_time > delta_time) {
      nearest_time_data = data;
      nearest_time = delta_time;
    }
  }
  return nearest_time_data;
}

DataStampedPtr GenericValueCalibrator::get_nearest_time_data_from_vec(
  DataStampedPtr base_data, const double back_time, const std::vector<DataStampedPtr> & vec)
{
  double nearest_time = std::numeric_limits<double>::max();
  const double target_time = base_data->data_time.seconds() - back_time;
  DataStampedPtr nearest_time_data;
  for (const auto & data : vec) {
    const double data_time = data->data_time.seconds();
    const auto delta_time = std::abs(target_time - data_time);
    if (nearest_time > delta_time) {
      nearest_time_data = data;
      nearest_time = delta_time;
    }
  }
  return nearest_time_data;
}

double GenericValueCalibrator::get_average(const std::vector<double> & vec)
{
  if (vec.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  for (const auto num : vec) {
    sum += num;
  }
  return sum / static_cast<double>(vec.size());
}

bool GenericValueCalibrator::is_timeout(
  const builtin_interfaces::msg::Time & stamp, const double timeout_sec)
{
  const double dt = this->now().seconds() - rclcpp::Time(stamp).seconds();
  return dt > timeout_sec;
}

bool GenericValueCalibrator::is_timeout(
  const DataStampedPtr & data_stamped, const double timeout_sec)
{
  const double dt = (this->now() - data_stamped->data_time).seconds();
  return dt > timeout_sec;
}

void GenericValueCalibrator::check_update_suggest(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  int8_t level = DiagStatus::OK;
  std::string msg = "OK";

  if (!is_default_map_) {
    level = DiagStatus::ERROR;
    msg = "Default map is not found in " + csv_default_map_dir_;
  }

  if (new_mse_que_.size() < part_mse_que_size_ / 2) {
    stat.summary(level, msg);
    return;
  }

  const double rmse_rate = new_rmse_ / part_original_rmse_;
  if (rmse_rate < update_suggest_thresh_) {
    level = DiagStatus::WARN;
    msg = "Value map calibration is required.";
  }

  stat.summary(level, msg);
}

void GenericValueCalibrator::publish_float64(const std::string & publish_type, const double val)
{
  Float64Stamped msg;
  msg.stamp = this->now();
  msg.data = val;

  if (publish_type == "current_map_error") {
    current_map_error_pub_->publish(msg);
  } else if (publish_type == "updated_map_error") {
    updated_map_error_pub_->publish(msg);
  } else {
    map_error_ratio_pub_->publish(msg);
  }
}

void GenericValueCalibrator::publish_update_suggest_flag()
{
  std_msgs::msg::Bool update_suggest;

  if (new_mse_que_.size() < part_mse_que_size_ / 2) {
    update_suggest.data = false;
  } else {
    const double rmse_rate = new_rmse_ / part_original_rmse_;
    update_suggest.data = (rmse_rate < update_suggest_thresh_);
    if (update_suggest.data) {
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "suggest to update value map. evaluation score = " << rmse_rate);
    }
  }

  update_suggest_pub_->publish(update_suggest);
}

bool GenericValueCalibrator::write_map_to_csv(
  std::vector<double> vel_index, std::vector<double> value_index, Map value_map,
  std::string filename)
{
  if (update_success_count_ == 0) {
    return false;
  }

  std::ofstream csv_file(filename);

  if (!csv_file.is_open()) {
    RCLCPP_WARN(get_logger(), "Failed to open csv file : %s", filename.c_str());
    return false;
  }

  csv_file << "default,";
  for (std::size_t v = 0; v < vel_index.size(); v++) {
    csv_file << vel_index.at(v);
    if (v != vel_index.size() - 1) {
      csv_file << ",";
    }
  }
  csv_file << "\n";

  for (std::size_t p = 0; p < value_index.size(); p++) {
    csv_file << value_index.at(p) << ",";
    for (std::size_t v = 0; v < vel_index.size(); v++) {
      csv_file << std::fixed << std::setprecision(precision_) << value_map.at(p).at(v);
      if (v != vel_index.size() - 1) {
        csv_file << ",";
      }
    }
    csv_file << "\n";
  }
  csv_file.close();
  RCLCPP_DEBUG_STREAM(get_logger(), "output map to " << filename);
  return true;
}

void GenericValueCalibrator::add_index_to_csv(std::ofstream * csv_file)
{
  *csv_file << "timestamp,velocity,accel,pitch_comp_accel,input_value,input_value_speed,"
            << "pitch,steer,jerk,part_original_rmse,new_rmse,rmse_rate" << std::endl;
}

/* Debug/Visualization Functions */

OccupancyGrid GenericValueCalibrator::get_occ_msg(
  const std::string & frame_id, const double height, const double width, const double resolution,
  const std::vector<int8_t> & map_value)
{
  OccupancyGrid occ;
  occ.header.frame_id = frame_id;
  occ.header.stamp = this->now();
  occ.info.height = static_cast<uint32_t>(height);
  occ.info.width = static_cast<uint32_t>(width);
  occ.info.map_load_time = this->now();
  occ.info.origin.position.x = 0;
  occ.info.origin.position.y = 0;
  occ.info.origin.position.z = 0;
  occ.info.origin.orientation.x = 0;
  occ.info.origin.orientation.y = 0;
  occ.info.origin.orientation.z = 0;
  occ.info.origin.orientation.w = 1;
  occ.info.resolution = static_cast<float>(resolution);
  occ.data = map_value;
  return occ;
}

void GenericValueCalibrator::publish_map(const Map & value_map, const std::string & publish_type)
{
  const double h = static_cast<double>(value_map.size());        // value index size
  const double w = static_cast<double>(value_map.at(0).size());  // velocity index size

  // Publish occupancy map
  const int8_t max_occ_value = 100;
  std::vector<int8_t> int_map_value;
  int_map_value.resize(static_cast<std::size_t>(h * w));

  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      const double value = value_map.at(i).at(j);
      // Convert value to 0~100 int value
      int8_t int_value =
        static_cast<int8_t>(max_occ_value * ((value - min_accel_) / (max_accel_ - min_accel_)));
      int_map_value.at(static_cast<std::size_t>(i * w + j)) =
        std::max(std::min(max_occ_value, int_value), static_cast<int8_t>(0));
    }
  }

  const double resolution = 0.1;  // meters per cell
  if (publish_type == "original") {
    original_map_occ_pub_->publish(get_occ_msg("base_link", h, w, resolution, int_map_value));
  } else {
    update_map_occ_pub_->publish(get_occ_msg("base_link", h, w, resolution, int_map_value));
  }

  // Publish raw map
  Float32MultiArray float_map;
  float_map.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  float_map.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  float_map.layout.dim[0].label = "height";
  float_map.layout.dim[1].label = "width";
  float_map.layout.dim[0].size = static_cast<uint32_t>(h);
  float_map.layout.dim[1].size = static_cast<uint32_t>(w);
  float_map.layout.dim[0].stride = static_cast<uint32_t>(h * w);
  float_map.layout.dim[1].stride = static_cast<uint32_t>(w);
  float_map.layout.data_offset = 0;

  std::vector<float> vec(static_cast<std::size_t>(h * w), 0);
  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      vec[i * w + j] = static_cast<float>(value_map.at(i).at(j));
    }
  }
  float_map.data = vec;

  if (publish_type == "original") {
    original_map_raw_pub_->publish(float_map);
  } else {
    update_map_raw_pub_->publish(float_map);
  }
}

void GenericValueCalibrator::publish_count_map()
{
  const double h = static_cast<double>(value_map_.size());
  const double w = static_cast<double>(value_map_.at(0).size());

  // Publish average map (using data_mean_mat_)
  std::vector<int8_t> average_map(static_cast<std::size_t>(h * w), 0);
  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      const double value = data_mean_mat_(i, j);  // Use Eigen matrix accessor
      int8_t int_value =
        static_cast<int8_t>(100 * ((value - min_accel_) / (max_accel_ - min_accel_)));
      average_map.at(static_cast<std::size_t>(i * w + j)) =
        std::max(std::min(static_cast<int8_t>(100), int_value), static_cast<int8_t>(0));
    }
  }
  data_ave_pub_->publish(get_occ_msg("base_link", h, w, 0.1, average_map));

  // Publish std dev map (computed from covariance)
  std::vector<int8_t> std_map(static_cast<std::size_t>(h * w), 0);
  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      const double variance = data_covariance_mat_(i, j);
      const double std_dev = std::sqrt(std::max(0.0, variance));  // Ensure non-negative
      int8_t int_value =
        static_cast<int8_t>(100 * std_dev / 5.0);  // Scale stddev, max expected ~5 m/s^2
      std_map.at(static_cast<std::size_t>(i * w + j)) =
        std::max(std::min(static_cast<int8_t>(100), int_value), static_cast<int8_t>(0));
    }
  }
  data_std_pub_->publish(get_occ_msg("base_link", h, w, 0.1, std_map));

  // Publish count map
  std::vector<int8_t> count_map(static_cast<std::size_t>(h * w), 0);
  for (int i = 0; i < h; i++) {
    for (int j = 0; j < w; j++) {
      const double count = data_num_(i, j);                            // Use Eigen matrix accessor
      int8_t int_value = static_cast<int8_t>(std::min(count, 100.0));  // Cap at 100
      count_map.at(static_cast<std::size_t>(i * w + j)) = int_value;
    }
  }
  data_count_pub_->publish(get_occ_msg("base_link", h, w, 0.1, count_map));

  // Publish count map with self pose
  int nearest_value_idx = nearest_value_index_search();
  int nearest_vel_idx = 0;
  if (!vel_index_.empty() && twist_ptr_) {  // Use twist_ptr_ instead of velocity_ptr_
    nearest_vel_idx = nearest_value_search(vel_index_, twist_ptr_->twist.linear.x);
  }

  // Mark current position
  std::vector<int8_t> count_with_self_pose = count_map;
  if (
    nearest_value_idx >= 0 && nearest_value_idx < h && nearest_vel_idx >= 0 &&
    nearest_vel_idx < w) {
    count_with_self_pose.at(static_cast<std::size_t>(nearest_value_idx * w + nearest_vel_idx)) =
      update_success_ ? std::numeric_limits<int8_t>::max() : std::numeric_limits<int8_t>::min();
  }
  data_count_with_self_pose_pub_->publish(
    get_occ_msg("base_link", h, w, 0.1, count_with_self_pose));
}

int GenericValueCalibrator::nearest_value_index_search() const
{
  if (!delayed_input_value_ptr_ || value_index_.empty()) {
    return 0;
  }
  return nearest_value_search(value_index_, delayed_input_value_ptr_->data);
}

void GenericValueCalibrator::publish_index()
{
  MarkerArray markers;
  const double h = static_cast<double>(value_map_.size());
  const double w = static_cast<double>(value_map_.at(0).size());
  const double resolution = 0.1;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = this->now();
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;

  std_msgs::msg::ColorRGBA color;
  color.a = 0.999;
  color.b = 0.1;
  color.g = 0.1;
  color.r = 0.1;
  marker.color = color;
  marker.frame_locked = true;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Value index labels (Y-axis)
  for (int value_idx = 0; value_idx < h; value_idx++) {
    const double value = value_index_.at(value_idx);
    marker.ns = "occ_value_index";
    marker.id = value_idx;
    marker.pose.position.x = -resolution * 0.5;
    marker.pose.position.y = resolution * (0.5 + value_idx);
    marker.scale.z = 0.03;

    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << value;
    marker.text = stream.str();
    markers.markers.push_back(marker);
  }

  // Value index name label
  marker.ns = "occ_value_index";
  marker.id = static_cast<int>(h);
  marker.pose.position.x = -resolution * 0.5;
  marker.pose.position.y = resolution * (0.5 + h);
  marker.scale.z = 0.03;
  marker.text = "(input_value)";
  markers.markers.push_back(marker);

  // Velocity index labels (X-axis)
  for (int vel_idx = 0; vel_idx < w; vel_idx++) {
    const double vel_value = vel_index_.at(vel_idx);
    marker.ns = "occ_vel_index";
    marker.id = vel_idx;
    marker.pose.position.x = resolution * (0.5 + vel_idx);
    marker.pose.position.y = -resolution * 0.2;
    marker.scale.z = 0.02;

    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << vel_value;
    marker.text = stream.str() + "m/s";
    markers.markers.push_back(marker);
  }

  // Velocity index name label
  marker.ns = "occ_vel_index";
  marker.id = static_cast<int>(w);
  marker.pose.position.x = resolution * (0.5 + w);
  marker.pose.position.y = -resolution * 0.2;
  marker.scale.z = 0.02;
  marker.text = "(velocity)";
  markers.markers.push_back(marker);

  index_pub_->publish(markers);
}

// Destructor to ensure log file is properly closed
GenericValueCalibrator::~GenericValueCalibrator()
{
  if (output_log_.is_open()) {
    output_log_.close();
    RCLCPP_INFO(get_logger(), "Calibration log file closed");
  }
}

}  // namespace autoware::generic_value_calibrator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::generic_value_calibrator::GenericValueCalibrator)
