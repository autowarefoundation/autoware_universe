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

#include "scene.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

TemplateModule::TemplateModule(
  const lanelet::Id module_id, const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface)
{
}

visualization_msgs::msg::MarkerArray TemplateModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray ma;
  return ma;
};

autoware::motion_utils::VirtualWalls TemplateModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls vw;
  return vw;
}

bool TemplateModule::modifyPathVelocity(
  [[maybe_unused]] Trajectory & path,
  [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & left_bound,
  [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & right_bound,
  [[maybe_unused]] const PlannerData & planner_data)
{
  RCLCPP_INFO_ONCE(logger_, "Template Module is executing!");
  return false;
}

}  // namespace autoware::behavior_velocity_planner::experimental
