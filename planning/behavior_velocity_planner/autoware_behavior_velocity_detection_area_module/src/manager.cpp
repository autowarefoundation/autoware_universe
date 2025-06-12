// Copyright 2020 Tier IV, Inc.
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

#include "manager.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils/ros/parameter.hpp>

#include <tf2/utils.h>

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_utils::get_or_declare_parameter;
using lanelet::autoware::DetectionArea;

DetectionAreaModuleManager::DetectionAreaModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(
    node, getModuleName(), getEnableRTC(node, std::string(getModuleName()) + ".enable_rtc"))
{
  const std::string ns(DetectionAreaModuleManager::getModuleName());
  planner_param_.stop_margin = get_or_declare_parameter<double>(node, ns + ".stop_margin");
  planner_param_.use_dead_line = get_or_declare_parameter<bool>(node, ns + ".use_dead_line");
  planner_param_.dead_line_margin =
    get_or_declare_parameter<double>(node, ns + ".dead_line_margin");
  planner_param_.use_pass_judge_line =
    get_or_declare_parameter<bool>(node, ns + ".use_pass_judge_line");
  planner_param_.state_clear_time =
    get_or_declare_parameter<double>(node, ns + ".state_clear_time");
  planner_param_.hold_stop_margin_distance =
    get_or_declare_parameter<double>(node, ns + ".hold_stop_margin_distance");
  planner_param_.distance_to_judge_over_stop_line =
    get_or_declare_parameter<double>(node, ns + ".distance_to_judge_over_stop_line");
  planner_param_.suppress_pass_judge_when_stopping =
    get_or_declare_parameter<bool>(node, ns + ".suppress_pass_judge_when_stopping");
  planner_param_.enable_detected_obstacle_logging =
    get_or_declare_parameter<bool>(node, ns + ".enable_detected_obstacle_logging");
}

void DetectionAreaModuleManager::launchNewModules(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  for (const auto & detection_area_with_lane_id :
       planning_utils::getRegElemMapOnPath<DetectionArea>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    // Use lanelet_id to unregister module when the route is changed
    const auto lane_id = detection_area_with_lane_id.second.id();
    const auto module_id = detection_area_with_lane_id.first->id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<DetectionAreaModule>(
        module_id, lane_id, *detection_area_with_lane_id.first, planner_param_,
        logger_.get_child("detection_area_module"), clock_, time_keeper_,
        planning_factor_interface_));
      generate_uuid(module_id);
      updateRTCStatus(
        getUUID(module_id), true, State::WAITING_FOR_EXECUTION,
        std::numeric_limits<double>::lowest(), path.header.stamp);
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
DetectionAreaModuleManager::getModuleExpiredFunction(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto detection_area_id_set = planning_utils::getRegElemIdSetOnPath<DetectionArea>(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return
    [detection_area_id_set](const std::shared_ptr<SceneModuleInterfaceWithRTC> & scene_module) {
      return detection_area_id_set.count(scene_module->getModuleId()) == 0;
    };
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::DetectionAreaModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
