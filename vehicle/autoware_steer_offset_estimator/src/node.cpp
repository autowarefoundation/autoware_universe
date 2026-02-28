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
#include <rclcpp/logging.hpp>

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
  pub_debug_info_ = this->create_publisher<StringStamped>("~/output/debug_info", 1);
  pub_steer_offset_update_ = this->create_publisher<Float32Stamped>(
    "~/output/steering_offset_update", rclcpp::QoS{1}.transient_local());

  set_calibration_parameters();

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
  // get current registered steering offset
  last_offset_update_ =
    this->declare_parameter<double>(calibration_params_.steer_offset_param_name);
  auto initial_msg = std::make_unique<autoware_internal_debug_msgs::msg::Float32Stamped>();
  initial_msg->stamp = this->now();
  initial_msg->data = static_cast<float>(last_offset_update_);
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

  calibration_params_.update_offset_th =
    this->declare_parameter<double>("calibration.update_offset_th");
  calibration_params_.covariance_th = this->declare_parameter<double>("calibration.covariance_th");
  calibration_params_.warning_offset_th =
    this->declare_parameter<double>("calibration.warning_offset_th");
  calibration_params_.min_steady_duration =
    this->declare_parameter<double>("calibration.min_steady_duration");
  calibration_params_.min_velocity = this->declare_parameter<double>("calibration.min_velocity");
  calibration_params_.max_offset_limit =
    this->declare_parameter<double>("calibration.max_offset_limit");
  calibration_params_.min_update_interval =
    this->declare_parameter<double>("calibration.min_update_interval");
  calibration_params_.steer_offset_param_path =
    this->declare_parameter<std::string>("steer_offset_param_path");
  calibration_params_.steer_offset_param_name =
    this->declare_parameter<std::string>("steer_offset_param_name");
  calibration_params_.calibration_file_path =
    this->declare_parameter<std::string>("calibration_file_path");
  calibration_params_.calibration_param_name =
    this->declare_parameter<std::string>("calibration_param_name");
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
    set_latest_reliable_result(result.value());
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

void SteerOffsetEstimatorNode::set_latest_reliable_result(
  const SteerOffsetEstimationUpdated & result)
{
  if (result.covariance > calibration_params_.covariance_th) {
    return;
  }

  const auto now = this->now();
  if ((now - last_no_result_time_).seconds() < calibration_params_.min_steady_duration) {
    return;
  }

  latest_reliable_result_ = result;
}

void SteerOffsetEstimatorNode::publish_data(const SteerOffsetEstimationUpdated & result)
{
  auto pub_float = [this](const auto & publisher, const double value) {
    autoware_internal_debug_msgs::msg::Float32Stamped msg;
    msg.stamp = this->now();
    msg.data = static_cast<float>(value);
    publisher->publish(msg);
  };

  pub_float(pub_steer_offset_, result.offset);
  pub_float(pub_steer_offset_covariance_, result.covariance);

  if (is_publish_update(result)) {
    pub_float(pub_steer_offset_update_, result.offset);
    last_offset_update_ = result.offset;
    log_offset_update(result);
  }

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

bool SteerOffsetEstimatorNode::is_publish_update(const SteerOffsetEstimationUpdated & result) const
{
  const double diff = std::abs(result.offset - last_offset_update_);
  return diff > calibration_params_.update_offset_th &&
         result.covariance < calibration_params_.covariance_th;
}

void SteerOffsetEstimatorNode::log_offset_update(const SteerOffsetEstimationUpdated & result) const
{
  const auto & param_path = calibration_params_.steer_offset_param_path;
  const auto & param_name = calibration_params_.steer_offset_param_name;

  auto write_result = write_to_yaml(param_path, param_name, result.offset);
  if (!write_result) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to write steer offset to YAML: %s", write_result.error().c_str());
  }
}

void SteerOffsetEstimatorNode::check_auto_calibration()
{
  if (!latest_reliable_result_) return;

  if (calibration_params_.mode != CalibrationMode::AUTO) {
    if (
      std::abs(latest_reliable_result_->offset) > calibration_params_.warning_offset_th &&
      latest_reliable_result_->covariance < calibration_params_.covariance_th) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *get_clock(), 1000,
        "Estimated steering offset %.4f exceeds warning threshold %.4f. Consider calibrating the "
        "steering offset.",
        latest_reliable_result_->offset, calibration_params_.warning_offset_th);
    }
    return;
  }

  const auto now = this->now();
  if (
    (now - last_calibration_time_).seconds() < calibration_params_.min_update_interval ||
    std::abs(latest_reliable_result_->offset) < calibration_params_.update_offset_th ||
    std::abs(latest_reliable_result_->offset) > calibration_params_.max_offset_limit) {
    return;
  }

  if (execute_calibration_update(latest_reliable_result_->offset)) {
    last_calibration_time_ = now;
    RCLCPP_INFO(this->get_logger(), "Auto calibration updated offset successfully.");
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

  if (!latest_reliable_result_) return reject("Rejected: latest reliable result not available");

  if (std::abs(latest_reliable_result_->offset) > calibration_params_.max_offset_limit)
    return reject(
      "Rejected: Offset value exceeds limit. Offset: " +
      std::to_string(latest_reliable_result_->offset));

  const auto calibration_result = execute_calibration_update(latest_reliable_result_->offset);
  if (calibration_result) {
    response->success = true;
    response->message =
      "Success: Offset updated to " + std::to_string(calibration_result.value()) + " rad.";
    last_calibration_time_ = this->now();
  } else {
    response->success = false;
    response->message = "Error: " + calibration_result.error();
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

tl::expected<double, std::string> SteerOffsetEstimatorNode::execute_calibration_update(
  const double steer_offset)
{
  const auto & param_path = calibration_params_.calibration_file_path;
  const auto & param_name = calibration_params_.calibration_param_name;

  return write_to_yaml(param_path, param_name, steer_offset, true);
}

tl::expected<double, std::string> SteerOffsetEstimatorNode::write_to_yaml(
  const std::string & file_path, const std::string & param_name, double value,
  const bool accumulate) const
{
  try {
    YAML::Node config = YAML::LoadFile(file_path);
    auto node = config["/**"]["ros__parameters"];

    if (!node) {
      return tl::make_unexpected("Could not find 'ros__parameters' key in YAML.");
    }

    auto write_value =
      accumulate ? value + (node[param_name] ? node[param_name].as<double>() : 0.0) : value;
    node[param_name] = write_value;

    std::ofstream fout(file_path);
    if (!fout.is_open()) {
      return tl::make_unexpected(fmt::format("Failed to open file for writing: {}", file_path));
    }

    fout << "# " << param_name << " was AUTOMATICALLY SET BY steer_offset_estimator at "
         << get_timestamp_string() << "\n";
    fout << config;
    fout.close();

    RCLCPP_INFO(
      this->get_logger(), "Saved %.4f to %s as '%s'", write_value, file_path.c_str(),
      param_name.c_str());

    return write_value;
  } catch (const std::exception & e) {
    return tl::make_unexpected(fmt::format("YAML update exception: {}", e.what()));
  }
}

}  // namespace autoware::steer_offset_estimator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::steer_offset_estimator::SteerOffsetEstimatorNode)
