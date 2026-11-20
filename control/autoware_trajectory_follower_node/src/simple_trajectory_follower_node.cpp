//  Copyright 2022 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "autoware/trajectory_follower_node/simple_trajectory_follower_node.hpp"

#include "autoware/trajectory_follower_node/simple_trajectory_follower.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tl/expected.hpp>

#include <chrono>
#include <functional>

namespace simple_trajectory_follower
{

namespace
{
// Reads the ROS parameters in the node layer so the core never touches them.
SimpleTrajectoryFollowerConfig create_config(rclcpp::Node & node)
{
  SimpleTrajectoryFollowerConfig config;
  config.use_external_target_vel = node.declare_parameter<bool>("use_external_target_vel");
  config.external_target_vel = node.declare_parameter<float>("external_target_vel");
  config.lateral_deviation = node.declare_parameter<float>("lateral_deviation");
  return config;
}
}  // namespace

SimpleTrajectoryFollowerNode::SimpleTrajectoryFollowerNode(const rclcpp::NodeOptions & options)
: Node("simple_trajectory_follower", options), follower_(create_config(*this))
{
  pub_cmd_ = create_publisher<Control>("~/output/control_cmd", 1);

  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 30ms, std::bind(&SimpleTrajectoryFollowerNode::on_timer, this));
}

void SimpleTrajectoryFollowerNode::on_timer()
{
  if (!process_data()) {
    RCLCPP_INFO(get_logger(), "data not ready");
    return;
  }

  const auto result =
    follower_.compute_control_command(*trajectory_, *odometry_, get_clock()->now());
  if (!result) {
    RCLCPP_WARN(get_logger(), "cannot compute control command: %s", result.error().c_str());
    return;
  }
  const auto & cmd = result->command;

  const auto & debug = result->debug;
  RCLCPP_DEBUG(
    get_logger(), "kp = %f, lat_err = %f, kd = %f, yaw_err = %f, steer = %f", debug.steer_kp,
    debug.lateral_error, debug.steer_kd, debug.yaw_error, cmd.lateral.steering_tire_angle);
  RCLCPP_DEBUG(
    get_logger(), "vel_err = %f, acc = %f", debug.velocity_error, cmd.longitudinal.acceleration);

  pub_cmd_->publish(cmd);
}

bool SimpleTrajectoryFollowerNode::process_data()
{
  bool is_ready = true;
  const auto & get_data = [](auto & dest, auto & sub) {
    const auto temp = sub.take_data();
    if (!temp) return false;
    dest = temp;
    return true;
  };
  is_ready &= get_data(odometry_, sub_kinematics_);
  is_ready &= get_data(trajectory_, sub_trajectory_);
  return is_ready;
}

}  // namespace simple_trajectory_follower

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(simple_trajectory_follower::SimpleTrajectoryFollowerNode)
