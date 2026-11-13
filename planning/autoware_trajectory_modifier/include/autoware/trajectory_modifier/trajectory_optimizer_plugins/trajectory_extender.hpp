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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_EXTENDER_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_EXTENDER_HPP_
#include "autoware/trajectory_modifier/trajectory_modifier_plugin_base.hpp"

#include <autoware_utils/system/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier::plugin
{
using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::TrajectoryModifierParams;
using autoware::trajectory_modifier::plugin::ProcessingResult;
using autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

class TrajectoryExtender : public TrajectoryModifierPluginBase
{
public:
  TrajectoryExtender() = default;
  ~TrajectoryExtender() = default;
  ProcessingResult process(TrajectoryPoints & traj_points, TrajectoryModifierData & data) override;
  void update_params(const TrajectoryModifierParams & params) override;

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;

private:
  Trajectory past_ego_state_trajectory_;
  trajectory_modifier_params::Params::TrajectoryExtender extender_params_;
};
}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_EXTENDER_HPP_
