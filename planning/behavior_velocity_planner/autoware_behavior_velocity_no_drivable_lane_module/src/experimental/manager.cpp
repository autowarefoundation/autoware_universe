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

#include "manager.hpp"

#include <memory>
#include <string>

namespace autoware::behavior_velocity_planner::experimental
{
NoDrivableLaneModuleManager::NoDrivableLaneModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(NoDrivableLaneModuleManager::getModuleName());
  planner_param_.stop_margin = get_or_declare_parameter<double>(node, ns + ".stop_margin");
  planner_param_.print_debug_info = get_or_declare_parameter<bool>(node, ns + ".print_debug_info");
}

void NoDrivableLaneModuleManager::launchNewModules(
  const Trajectory & path, [[maybe_unused]] const rclcpp::Time & stamp,
  const PlannerData & planner_data)
{
  PathWithLaneId path_msg;
  path_msg.points = path.restore();

  for (const auto & ll : planning_utils::getLaneletsOnPath(
         path_msg, planner_data.route_handler_->getLaneletMapPtr(),
         planner_data.current_odometry->pose)) {
    const auto lane_id = ll.id();
    const auto module_id = lane_id;

    if (isModuleRegistered(module_id)) {
      continue;
    }

    const std::string no_drivable_lane_attribute = ll.attributeOr("no_drivable_lane", "no");
    if (no_drivable_lane_attribute != "yes") {
      continue;
    }

    registerModule(
      std::make_shared<NoDrivableLaneModule>(
        module_id, lane_id, planner_param_, logger_.get_child("no_drivable_lane_module"), clock_,
        time_keeper_, planning_factor_interface_),
      planner_data);
  }
}

std::function<bool(const std::shared_ptr<experimental::SceneModuleInterface> &)>
NoDrivableLaneModuleManager::getModuleExpiredFunction(
  const Trajectory & path, const PlannerData & planner_data)
{
  PathWithLaneId path_msg;
  path_msg.points = path.restore();

  const auto lane_id_set = planning_utils::getLaneIdSetOnPath(
    path_msg, planner_data.route_handler_->getLaneletMapPtr(), planner_data.current_odometry->pose);

  return [lane_id_set](const std::shared_ptr<experimental::SceneModuleInterface> & scene_module) {
    return lane_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace autoware::behavior_velocity_planner::experimental

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::experimental::NoDrivableLaneModulePlugin,
  autoware::behavior_velocity_planner::experimental::PluginInterface)
