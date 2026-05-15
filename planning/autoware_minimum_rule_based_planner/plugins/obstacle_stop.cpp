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

#include "obstacle_stop.hpp"

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>

#include <memory>

namespace autoware::minimum_rule_based_planner::plugin
{

void ObstacleStop::on_initialize([[maybe_unused]] const TrajectoryModifierParams & params)
{
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      get_node_ptr(), "obstacle_stop");
}

bool ObstacleStop::is_trajectory_modification_required(
  [[maybe_unused]] const TrajectoryPoints & traj_points)
{
  return false;
}

void ObstacleStop::modify_trajectory([[maybe_unused]] TrajectoryPoints & traj_points)
{
  // TODO(odashima): implement logic
}

void ObstacleStop::update_params([[maybe_unused]] const TrajectoryModifierParams & params)
{
  // TODO(odashima): implement logic
}

}  // namespace autoware::minimum_rule_based_planner::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::minimum_rule_based_planner::plugin::ObstacleStop,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
