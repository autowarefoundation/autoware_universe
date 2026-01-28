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

#include "autoware/behavior_path_side_shift_module/manager.hpp"

#include "autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

void SideShiftModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {});

  SideShiftParameters p{};

  const std::string ns = "side_shift.";
  p.min_distance_to_start_shifting =
    node->declare_parameter<double>(ns + "min_distance_to_start_shifting");
  p.time_to_start_shifting = node->declare_parameter<double>(ns + "time_to_start_shifting");
  p.shifting_lateral_jerk = node->declare_parameter<double>(ns + "shifting_lateral_jerk");
  p.min_shifting_distance = node->declare_parameter<double>(ns + "min_shifting_distance");
  p.min_shifting_speed = node->declare_parameter<double>(ns + "min_shifting_speed");
  p.shift_request_time_limit = node->declare_parameter<double>(ns + "shift_request_time_limit");
  p.drivable_area_check_mode = node->declare_parameter<int>(ns + "drivable_area_check_mode");
  p.min_drivable_area_margin = node->declare_parameter<double>(ns + "min_drivable_area_margin");
  p.publish_debug_marker = node->declare_parameter<bool>(ns + "publish_debug_marker");

  parameters_ = std::make_shared<SideShiftParameters>(p);
}

void SideShiftModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  auto p = parameters_;

  const std::string ns = "side_shift.";
  update_param<int>(parameters, ns + "drivable_area_check_mode", p->drivable_area_check_mode);
  update_param<double>(parameters, ns + "min_drivable_area_margin", p->min_drivable_area_margin);
  update_param<double>(
    parameters, ns + "min_distance_to_start_shifting", p->min_distance_to_start_shifting);
  update_param<double>(parameters, ns + "time_to_start_shifting", p->time_to_start_shifting);
  update_param<double>(parameters, ns + "shifting_lateral_jerk", p->shifting_lateral_jerk);
  update_param<double>(parameters, ns + "min_shifting_distance", p->min_shifting_distance);
  update_param<double>(parameters, ns + "min_shifting_speed", p->min_shifting_speed);
  update_param<bool>(parameters, ns + "publish_debug_marker", p->publish_debug_marker);

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::SideShiftModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
