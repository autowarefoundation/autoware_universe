// Copyright 2025 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
using lanelet::autoware::SpeedBump;

SpeedBumpModuleManager::SpeedBumpModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  std::string ns(SpeedBumpModuleManager::getModuleName());
  planner_param_.slow_start_margin =
    get_or_declare_parameter<double>(node, ns + ".slow_start_margin");
  planner_param_.slow_end_margin = get_or_declare_parameter<double>(node, ns + ".slow_end_margin");
  planner_param_.print_debug_info = get_or_declare_parameter<bool>(node, ns + ".print_debug_info");

  // limits for speed bump height and slow down speed
  ns += ".speed_calculation";
  planner_param_.speed_calculation_min_height =
    static_cast<float>(get_or_declare_parameter<double>(node, ns + ".min_height"));
  planner_param_.speed_calculation_max_height =
    static_cast<float>(get_or_declare_parameter<double>(node, ns + ".max_height"));
  planner_param_.speed_calculation_min_speed =
    static_cast<float>(get_or_declare_parameter<double>(node, ns + ".min_speed"));
  planner_param_.speed_calculation_max_speed =
    static_cast<float>(get_or_declare_parameter<double>(node, ns + ".max_speed"));
}

void SpeedBumpModuleManager::launchNewModules(
  const Trajectory & path, [[maybe_unused]] const rclcpp::Time & stamp,
  const PlannerData & planner_data)
{
  PathWithLaneId path_msg;
  path_msg.points = path.restore();

  for (const auto & speed_bump_with_lane_id : planning_utils::getRegElemMapOnPath<SpeedBump>(
         path_msg, planner_data.route_handler_->getLaneletMapPtr(),
         planner_data.current_odometry->pose)) {
    const auto lane_id = speed_bump_with_lane_id.second.id();
    const auto module_id = speed_bump_with_lane_id.first->id();
    if (!isModuleRegistered(module_id)) {
      registerModule(
        std::make_shared<SpeedBumpModule>(
          module_id, lane_id, *speed_bump_with_lane_id.first, planner_param_,
          logger_.get_child("speed_bump_module"), clock_, time_keeper_, planning_factor_interface_),
        planner_data);
    }
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
SpeedBumpModuleManager::getModuleExpiredFunction(
  const Trajectory & path, const PlannerData & planner_data)
{
  PathWithLaneId path_msg;
  path_msg.points = path.restore();

  const auto speed_bump_id_set = planning_utils::getRegElemIdSetOnPath<SpeedBump>(
    path_msg, planner_data.route_handler_->getLaneletMapPtr(), planner_data.current_odometry->pose);

  return [speed_bump_id_set](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return speed_bump_id_set.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace autoware::behavior_velocity_planner::experimental

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::experimental::SpeedBumpModulePlugin,
  autoware::behavior_velocity_planner::experimental::PluginInterface)
