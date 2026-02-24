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

#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace autoware::steer_offset_estimator
{

SteerOffsetEstimatorParameters load_parameters(rclcpp::Node * node)
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
: Node("steer_offset_estimator", node_options), estimator_(load_parameters(this))
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

  // get current registered steering offset
  auto initial_steer_offset_param_name =
    this->declare_parameter<std::string>("initial_steer_offset_param_name");
  current_steering_offset_ = this->declare_parameter<double>(initial_steer_offset_param_name);

  // Create timer
  auto update_hz = this->declare_parameter<double>("update_hz", 10.0);
  const auto period = rclcpp::Rate(update_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period, std::bind(&SteerOffsetEstimatorNode::on_timer, this));
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
    publish_data(result.value());
  } else {
    RCLCPP_DEBUG(
      this->get_logger(), "Failed to update steer offset estimator: %s",
      result.error().reason.c_str());
    autoware_internal_debug_msgs::msg::StringStamped debug_info;
    debug_info.stamp = this->now();
    debug_info.data =
      fmt::format("Failed to update steer offset estimator:\n{}", result.error().reason);
    pub_debug_info_->publish(debug_info);
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

}  // namespace autoware::steer_offset_estimator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::steer_offset_estimator::SteerOffsetEstimatorNode)
