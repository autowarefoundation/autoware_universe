// Copyright 2026 TIER IV, Inc.
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

#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params_ros.hpp"

#include <string>

namespace autoware::mppi_optimizer
{
namespace
{

std::string param_name(const std::string & prefix, const std::string & name)
{
  return prefix.empty() ? name : prefix + name;
}

}  // namespace

void declare_first_order_dubins_mppi_cost_params(rclcpp::Node & node, const std::string & prefix)
{
  const FirstOrderDubinsMppiCostParams defaults;
  node.declare_parameter(param_name(prefix, "desired_speed"), defaults.desired_speed);
  node.declare_parameter(param_name(prefix, "speed_coeff"), defaults.speed_coeff);
  node.declare_parameter(param_name(prefix, "track_coeff"), defaults.track_coeff);
  node.declare_parameter(param_name(prefix, "heading_coeff"), defaults.heading_coeff);
  node.declare_parameter(param_name(prefix, "crash_coeff"), defaults.crash_coeff);
  node.declare_parameter(param_name(prefix, "boundary_threshold"), defaults.boundary_threshold);
  node.declare_parameter(
    param_name(prefix, "boundary_threshold_left"), defaults.boundary_threshold_left);
  node.declare_parameter(
    param_name(prefix, "boundary_threshold_right"), defaults.boundary_threshold_right);
  node.declare_parameter(param_name(prefix, "accel_cmd_coeff"), defaults.accel_cmd_coeff);
  node.declare_parameter(param_name(prefix, "steer_cmd_coeff"), defaults.steer_cmd_coeff);
  node.declare_parameter(
    param_name(prefix, "lateral_acceleration_coeff"), defaults.lateral_acceleration_coeff);
  node.declare_parameter(param_name(prefix, "lateral_jerk_coeff"), defaults.lateral_jerk_coeff);
  node.declare_parameter(
    param_name(prefix, "longitudinal_jerk_coeff"), defaults.longitudinal_jerk_coeff);
  node.declare_parameter(
    param_name(prefix, "obstacle_collision_margin"), defaults.obstacle_collision_margin);
  node.declare_parameter(param_name(prefix, "goal_pos_coeff"), defaults.goal_pos_coeff);
  node.declare_parameter(param_name(prefix, "goal_speed_coeff"), defaults.goal_speed_coeff);
  node.declare_parameter(param_name(prefix, "goal_yaw_coeff"), defaults.goal_yaw_coeff);
  node.declare_parameter(param_name(prefix, "goal_terminal_scale"), defaults.goal_terminal_scale);
}

FirstOrderDubinsMppiCostParams get_first_order_dubins_mppi_cost_params(
  const rclcpp::Node & node, const std::string & prefix)
{
  FirstOrderDubinsMppiCostParams params;
  params.desired_speed =
    static_cast<float>(node.get_parameter(param_name(prefix, "desired_speed")).as_double());
  params.speed_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "speed_coeff")).as_double());
  params.track_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "track_coeff")).as_double());
  params.heading_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "heading_coeff")).as_double());
  params.crash_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "crash_coeff")).as_double());
  params.boundary_threshold =
    static_cast<float>(node.get_parameter(param_name(prefix, "boundary_threshold")).as_double());
  params.boundary_threshold_left = static_cast<float>(
    node.get_parameter(param_name(prefix, "boundary_threshold_left")).as_double());
  params.boundary_threshold_right = static_cast<float>(
    node.get_parameter(param_name(prefix, "boundary_threshold_right")).as_double());
  params.accel_cmd_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "accel_cmd_coeff")).as_double());
  params.steer_cmd_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "steer_cmd_coeff")).as_double());
  params.lateral_acceleration_coeff = static_cast<float>(
    node.get_parameter(param_name(prefix, "lateral_acceleration_coeff")).as_double());
  params.lateral_jerk_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "lateral_jerk_coeff")).as_double());
  params.longitudinal_jerk_coeff = static_cast<float>(
    node.get_parameter(param_name(prefix, "longitudinal_jerk_coeff")).as_double());
  params.obstacle_collision_margin = static_cast<float>(
    node.get_parameter(param_name(prefix, "obstacle_collision_margin")).as_double());
  params.goal_pos_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "goal_pos_coeff")).as_double());
  params.goal_speed_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "goal_speed_coeff")).as_double());
  params.goal_yaw_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "goal_yaw_coeff")).as_double());
  params.goal_terminal_scale =
    static_cast<float>(node.get_parameter(param_name(prefix, "goal_terminal_scale")).as_double());
  return params;
}

}  // namespace autoware::mppi_optimizer
