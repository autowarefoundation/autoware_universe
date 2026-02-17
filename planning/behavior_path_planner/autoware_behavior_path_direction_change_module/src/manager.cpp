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

#include "autoware/behavior_path_direction_change_module/manager.hpp"

#include "autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

void DirectionChangeModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {});

  DirectionChangeParameters p{};

  const std::string ns = "direction_change.";

  // Cusp detection parameters
  p.cusp_detection_distance_threshold =
    node->declare_parameter<double>(ns + "cusp_detection_distance_threshold");
  p.cusp_detection_angle_threshold_deg =
    node->declare_parameter<double>(ns + "cusp_detection_angle_threshold_deg");
  p.reverse_initial_speed = node->declare_parameter<double>(ns + "reverse_initial_speed");

  // State transition parameters
  p.cusp_detection_distance_start_approaching =
    node->declare_parameter<double>(ns + "cusp_detection_distance_start_approaching");
  p.stop_velocity_threshold =
    node->declare_parameter<double>(ns + "stop_velocity_threshold");
  p.th_stopped_time = node->declare_parameter<double>(ns + "th_stopped_time");

  // Reverse lane following parameters
  p.reverse_speed_limit = node->declare_parameter<double>(ns + "reverse_speed_limit");

  // Path densification parameters
  p.reverse_path_densify_max_yaw_step_deg =
    node->declare_parameter<double>(ns + "reverse_path_densify_max_yaw_step_deg");
  p.reverse_path_densify_max_distance_step =
    node->declare_parameter<double>(ns + "reverse_path_densify_max_distance_step");

  // General parameters
  p.enable_cusp_detection = node->declare_parameter<bool>(ns + "enable_cusp_detection");
  p.enable_reverse_following = node->declare_parameter<bool>(ns + "enable_reverse_following");
  p.publish_debug_marker = node->declare_parameter<bool>(ns + "publish_debug_marker");
  p.th_arrived_distance = node->declare_parameter<double>(ns + "th_arrived_distance");

  parameters_ = std::make_shared<DirectionChangeParameters>(p);
}

void DirectionChangeModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  [[maybe_unused]] auto p = parameters_;

  [[maybe_unused]] const std::string ns = "direction_change.";
  // update_param<bool>(parameters, ns + ..., ...);

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::DirectionChangeModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)

