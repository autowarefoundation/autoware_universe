// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_dry_steering_avoidance_module/manager.hpp"

#include "autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

void DrySteeringAvoidanceModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {""});

  RCLCPP_WARN(node->get_logger(), "[DrySteeringAvoidance] Manager::init() called - Module registered!");

  DrySteeringAvoidanceParameters parameters = DrySteeringAvoidanceParameters::init(*node);
  parameters_ = std::make_shared<DrySteeringAvoidanceParameters>(parameters);

  RCLCPP_WARN(node->get_logger(), "[DrySteeringAvoidance] Parameters loaded successfully");
}

void DrySteeringAvoidanceModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  auto & p = parameters_;

  const std::string ns = "dry_steering_avoidance.";

  // activation condition
  update_param<double>(parameters, ns + "th_stopped_velocity", p->th_stopped_velocity);
  update_param<double>(parameters, ns + "th_stopped_time", p->th_stopped_time);
  update_param<double>(parameters, ns + "th_static_object_velocity", p->th_static_object_velocity);

  // object detection
  update_param<double>(
    parameters, ns + "object_search_forward_distance", p->object_search_forward_distance);

  // goal pose sampling
  update_param<std::string>(parameters, ns + "avoidance_direction", p->avoidance_direction);
  update_param<double>(parameters, ns + "longitudinal_search_min", p->longitudinal_search_min);
  update_param<double>(parameters, ns + "longitudinal_search_max", p->longitudinal_search_max);
  update_param<double>(
    parameters, ns + "longitudinal_search_interval", p->longitudinal_search_interval);
  update_param<double>(parameters, ns + "lateral_search_min", p->lateral_search_min);
  update_param<double>(parameters, ns + "lateral_search_max", p->lateral_search_max);
  update_param<double>(parameters, ns + "lateral_search_interval", p->lateral_search_interval);

  // geometric pull out parameters
  update_param<double>(
    parameters, ns + "geometric_pull_out_velocity",
    p->parallel_parking_parameters.pull_out_velocity);
  update_param<double>(
    parameters, ns + "arc_path_interval",
    p->parallel_parking_parameters.pull_out_arc_path_interval);
  update_param<double>(
    parameters, ns + "lane_departure_margin",
    p->parallel_parking_parameters.pull_out_lane_departure_margin);
  update_param<double>(
    parameters, ns + "center_line_path_interval",
    p->parallel_parking_parameters.center_line_path_interval);
  update_param<double>(
    parameters, ns + "geometric_pull_out_max_steer_angle_margin_scale",
    p->parallel_parking_parameters.geometric_pull_out_max_steer_angle_margin_scale);
  update_param<double>(
    parameters, ns + "geometric_collision_check_distance_from_end",
    p->geometric_collision_check_distance_from_end);
  update_param<bool>(parameters, ns + "divide_pull_out_path", p->divide_pull_out_path);
  update_param<double>(
    parameters, ns + "lane_departure_check_expansion_margin",
    p->lane_departure_check_expansion_margin);

  // debug end pose offset parameters
  update_param<double>(
    parameters, ns + "debug_end_pose_longitudinal_offset",
    p->parallel_parking_parameters.debug_end_pose_longitudinal_offset);
  update_param<double>(
    parameters, ns + "debug_end_pose_lateral_offset",
    p->parallel_parking_parameters.debug_end_pose_lateral_offset);
  update_param<double>(
    parameters, ns + "center_line_path_extension",
    p->parallel_parking_parameters.center_line_path_extension);

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::DrySteeringAvoidanceModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
