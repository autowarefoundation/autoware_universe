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

#include "manager.hpp"

#include <memory>
#include <string>
#include <utility>

namespace autoware::behavior_velocity_planner
{
TemplateModuleManager::TemplateModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  std::string ns(TemplateModuleManager::getModuleName());
  dummy_parameter_ = get_or_declare_parameter<double>(node, ns + ".dummy");
}

void TemplateModuleManager::launchNewModules(
  [[maybe_unused]] const Trajectory & path, [[maybe_unused]] const rclcpp::Time & stamp,
  const PlannerData & planner_data)
{
  int64_t module_id = 0;
  if (!isModuleRegistered(module_id)) {
    registerModule(
      std::make_shared<TemplateModule>(
        module_id, logger_.get_child(getModuleName()), clock_, time_keeper_,
        planning_factor_interface_),
      planner_data);
  }
}

std::function<bool(const std::shared_ptr<experimental::SceneModuleInterface> &)>
TemplateModuleManager::getModuleExpiredFunction(
  [[maybe_unused]] const Trajectory & path, [[maybe_unused]] const PlannerData & planner_data)
{
  return
    []([[maybe_unused]] const std::shared_ptr<experimental::SceneModuleInterface> & scene_module)
      -> bool { return false; };
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::TemplateModulePlugin,
  autoware::behavior_velocity_planner::experimental::PluginInterface)
