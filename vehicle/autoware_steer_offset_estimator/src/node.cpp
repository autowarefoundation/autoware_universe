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

#include "node.hpp"

#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>

#include <fmt/format.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::steer_offset_estimator
{

SteerOffsetEstimatorParameters load_estimator_parameters(rclcpp::Node * node)
{
  SteerOffsetEstimatorParameters parameters;
  parameters.initial_covariance = node->declare_parameter<double>("initial_covariance");
  parameters.initial_offset = node->declare_parameter<double>("initial_offset");
  parameters.wheel_base =
    autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo().wheel_base_m;
  parameters.min_velocity = node->declare_parameter<double>("min_velocity");
  parameters.max_steer = node->declare_parameter<double>("max_steer");
  parameters.max_steer_rate = node->declare_parameter<double>("max_steer_rate");
  parameters.max_ang_velocity = node->declare_parameter<double>("max_ang_velocity");
  parameters.process_noise_covariance = node->declare_parameter<double>("process_noise_covariance");
  parameters.measurement_noise_covariance =
    node->declare_parameter<double>("measurement_noise_covariance");
  parameters.denominator_floor = node->declare_parameter<double>("denominator_floor");
  parameters.covariance_floor = node->declare_parameter<double>("covariance_floor");
  parameters.max_steer_buffer = node->declare_parameter<double>("max_steer_buffer");
  parameters.max_pose_lag = node->declare_parameter<double>("max_pose_lag");
  return parameters;
}

SteerOffsetEstimatorNode::SteerOffsetEstimatorNode(const rclcpp::NodeOptions & node_options)
: Node("steer_offset_estimator", node_options), estimator_(load_estimator_parameters(this))
{
  // Subscribers
  sub_pose_ =
    PollingSubscriber<PoseStamped>::create_subscription(this, "~/input/pose", rclcpp::QoS{10});
  sub_steer_ =
    PollingSubscriber<SteeringReport>::create_subscription(this, "~/input/steer", rclcpp::QoS{10});

  // Publishers
  pub_steer_offset_ = this->create_publisher<Float32Stamped>("~/output/steering_offset", 1);
  pub_steer_offset_covariance_ =
    this->create_publisher<Float32Stamped>("~/output/steering_offset_covariance", 1);
  pub_steer_offset_error_ =
    this->create_publisher<Float32Stamped>("~/output/steering_offset_error", 1);
  pub_debug_info_ = this->create_publisher<StringStamped>("~/output/debug_info", 1);
  pub_steer_offset_update_ = this->create_publisher<Float32Stamped>(
    "~/output/steering_offset_update", rclcpp::QoS{1}.transient_local());

  set_calibration_parameters();

  // get current registered steering offset
  current_steering_offset_ =
    this->declare_parameter<double>(calibration_params_.steer_offset_param_name);

  // Create timer
  auto update_hz = this->declare_parameter<double>("update_hz", 10.0);
  const auto period = rclcpp::Rate(update_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period, std::bind(&SteerOffsetEstimatorNode::on_timer, this));

  srv_update_offset_ = create_service<std_srvs::srv::Trigger>(
    "~/trigger_steer_offset_calibration", std::bind(
                                            &SteerOffsetEstimatorNode::on_update_offset_request,
                                            this, std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(
    this->get_logger(), "Calibration service initialized in %s mode.",
    SteerOffsetCalibrationParameters::mode_to_str_map.at(calibration_params_.mode).c_str());

  // publish initial steer offset value
  auto initial_msg = std::make_unique<autoware_internal_debug_msgs::msg::Float32Stamped>();
  initial_msg->stamp = this->now();
  initial_msg->data = static_cast<float>(current_steering_offset_);
  pub_steer_offset_update_->publish(std::move(initial_msg));

  last_calibration_time_ = this->now();
  last_no_result_time_ = this->now();
}

void SteerOffsetEstimatorNode::set_calibration_parameters()
{
  const auto mode_str = this->declare_parameter<std::string>("calibration.mode");
  if (SteerOffsetCalibrationParameters::mode_map.count(mode_str)) {
    calibration_params_.mode = SteerOffsetCalibrationParameters::mode_map.at(mode_str);
  } else {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid calibration mode: %s. Defaulting to OFF.", mode_str.c_str());
    calibration_params_.mode = CalibrationMode::OFF;
  }

  calibration_params_.error_threshold =
    this->declare_parameter<double>("calibration.error_threshold");
  calibration_params_.covariance_threshold =
    this->declare_parameter<double>("calibration.covariance_threshold");
  calibration_params_.min_steady_duration =
    this->declare_parameter<double>("calibration.min_steady_duration");
  calibration_params_.min_velocity = this->declare_parameter<double>("calibration.min_velocity");
  calibration_params_.max_offset_limit =
    this->declare_parameter<double>("calibration.max_offset_limit");
  calibration_params_.min_update_interval =
    this->declare_parameter<double>("calibration.min_update_interval");
  calibration_params_.enable_parameter_file_overwrite =
    this->declare_parameter<bool>("calibration.enable_parameter_file_overwrite");
  calibration_params_.steer_offset_param_path =
    this->declare_parameter<std::string>("steer_offset_param_path");
  calibration_params_.steer_offset_param_name =
    this->declare_parameter<std::string>("steer_offset_param_name");
}

void SteerOffsetEstimatorNode::on_timer()
{
  std::vector<PoseStamped::ConstSharedPtr> pose_ptrs = sub_pose_->take_data();
  std::vector<SteeringReport::ConstSharedPtr> steer_ptrs = sub_steer_->take_data();

  std::vector<PoseStamped> poses;
  poses.reserve(pose_ptrs.size());
  for (const auto & pose : pose_ptrs) {
    poses.emplace_back(*pose);
  }
  // Convert to vector of values
  std::vector<SteeringReport> steers;
  steers.reserve(steer_ptrs.size());
  for (const auto & steer : steer_ptrs) {
    steers.emplace_back(*steer);
  }

  auto result = estimator_.update(poses, steers);
  if (result) {
    latest_result_ = result.value();
    publish_data(result.value());
    check_auto_calibration();
  } else {
    RCLCPP_DEBUG(
      this->get_logger(), "Failed to update steer offset estimator: %s",
      result.error().reason.c_str());
    autoware_internal_debug_msgs::msg::StringStamped debug_info;
    debug_info.stamp = this->now();
    debug_info.data =
      fmt::format("Failed to update steer offset estimator:\n{}", result.error().reason);
    pub_debug_info_->publish(debug_info);
    last_no_result_time_ = this->now();
  }
}

void SteerOffsetEstimatorNode::publish_data(const SteerOffsetEstimationUpdated & result) const
{
  auto pub_float = [this](const auto & publisher, const double value) {
    autoware_internal_debug_msgs::msg::Float32Stamped msg;
    msg.stamp = this->now();
    msg.data = static_cast<float>(value);
    publisher->publish(msg);
  };

  pub_float(pub_steer_offset_, result.offset);
  pub_float(pub_steer_offset_covariance_, result.covariance);

  const double offset_error = result.offset - current_steering_offset_;
  pub_float(pub_steer_offset_error_, offset_error);

  autoware_internal_debug_msgs::msg::StringStamped debug_info;
  debug_info.stamp = this->now();
  debug_info.data = fmt::format(
    "offset: {:.5f}, std_dev: {:.5f},\n"
    "velocity: {:.5f}, angular_velocity: {:.5f},\n"
    "steering_angle: {:.5f},\n"
    "kalman_gain: {:.5f}, residual: {:.5f}",
    result.offset, std::sqrt(result.covariance), result.velocity, result.angular_velocity,
    result.steering_angle, result.kalman_gain, result.residual);
  pub_debug_info_->publish(debug_info);
}

void SteerOffsetEstimatorNode::check_auto_calibration()
{
  if (calibration_params_.mode != CalibrationMode::AUTO) return;

  const auto now = this->now();
  if (
    (now - last_calibration_time_).seconds() < calibration_params_.min_update_interval ||
    (now - last_no_result_time_).seconds() < calibration_params_.min_steady_duration) {
    return;
  }

  if (!latest_result_) return;

  const double offset_error = std::abs(latest_result_->offset - current_steering_offset_);

  if (offset_error < calibration_params_.error_threshold) {
    return;
  }

  if (
    latest_result_->velocity < calibration_params_.min_velocity ||
    latest_result_->covariance > calibration_params_.covariance_threshold ||
    std::abs(latest_result_->offset) > calibration_params_.max_offset_limit) {
    return;
  }

  if (execute_calibration_update(latest_result_->offset)) {
    last_calibration_time_ = now;
    RCLCPP_INFO(
      this->get_logger(), "Auto calibration updated offset to %.4f rad.", latest_result_->offset);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Auto calibration failed to write to YAML path.");
  }
}

void SteerOffsetEstimatorNode::on_update_offset_request(
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  auto reject = [&](const auto & msg) {
    response->success = false;
    response->message = msg;
    return;
  };

  if (calibration_params_.mode == CalibrationMode::OFF)
    return reject("Rejected: calibration is in OFF mode");

  if (!latest_result_) return reject("Rejected: estimation result not available");

  if (latest_result_->covariance > calibration_params_.covariance_threshold)
    return reject(
      "Rejected: High uncertainty. Covariance: " + std::to_string(latest_result_->covariance));

  if (std::abs(latest_result_->offset) > calibration_params_.max_offset_limit)
    return reject(
      "Rejected: Offset value exceeds limit. Offset: " + std::to_string(latest_result_->offset));

  if (execute_calibration_update(latest_result_->offset)) {
    response->success = true;
    response->message =
      "Success: Offset updated to " + std::to_string(latest_result_->offset) + " rad.";
    last_calibration_time_ = this->now();
  } else {
    response->success = false;
    response->message = "Error: Failed to write to YAML path.";
  }
}

std::string get_timestamp_string()
{
  auto now = std::chrono::system_clock::now();
  auto timestamp = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&timestamp), "%Y-%m-%d %H:%M:%S");
  return ss.str();
}

bool SteerOffsetEstimatorNode::execute_calibration_update(const double steer_offset)
{
  const auto & param_path = calibration_params_.steer_offset_param_path;
  const auto & param_name = calibration_params_.steer_offset_param_name;

  current_steering_offset_ = steer_offset;
  autoware_internal_debug_msgs::msg::Float32Stamped msg;
  msg.stamp = this->now();
  msg.data = static_cast<float>(steer_offset);
  pub_steer_offset_update_->publish(msg);

  if (!calibration_params_.enable_parameter_file_overwrite) return true;

  try {
    YAML::Node config = YAML::LoadFile(param_path);
    auto node = config["/**"]["ros__parameters"];

    if (node) {
      node[param_name] = steer_offset;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not find 'ros__parameters' key in YAML.");
      return false;
    }

    std::ofstream fout(param_path);
    if (!fout.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s", param_path.c_str());
      return false;
    }

    fout << "# " << param_name << " was AUTOMATICALLY UPDATED BY steer_offset_estimator "
         << get_timestamp_string() << "\n";
    fout << config;
    fout.close();

    RCLCPP_INFO(
      this->get_logger(), "Saved %.4f to %s as '%s'", steer_offset, param_path.c_str(),
      param_name.c_str());

    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "YAML update exception: %s", e.what());
    return false;
  }
}

}  // namespace autoware::steer_offset_estimator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::steer_offset_estimator::SteerOffsetEstimatorNode)
