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

#ifndef PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__OBSTACLE_STOP_HPP_
#define PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__OBSTACLE_STOP_HPP_

#include <autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp>

#include <vector>

namespace autoware::minimum_rule_based_planner::plugin
{

using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase;
using TrajectoryModifierParams = trajectory_modifier_params::Params;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

class ObstacleStop : public TrajectoryModifierPluginBase
{
public:
  ObstacleStop() = default;

  void modify_trajectory(TrajectoryPoints & traj_points) override;

  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points) override;

  void update_params(const TrajectoryModifierParams & params) override;

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;
};

}  // namespace autoware::minimum_rule_based_planner::plugin

#endif  // PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__OBSTACLE_STOP_HPP_
