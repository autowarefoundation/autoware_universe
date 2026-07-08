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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_MODIFIER_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_MODIFIER_HPP_

#include "autoware/trajectory_processor/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp"
#include "autoware/trajectory_processor/trajectory_modifier_utils/utils.hpp"

#include <rclcpp/rclcpp.hpp>

namespace autoware::trajectory_modifier::plugin
{

class VelocityModifier : public TrajectoryModifierPluginBase
{
public:
  VelocityModifier() = default;

  bool modify_trajectory(TrajectoryPoints & traj_points, const InputData & input) override;

  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, [[maybe_unused]] const InputData & input);

  void update_params(const TrajectoryModifierParams & params) override
  {
    enabled_ = params.use_velocity_modifier;
    trajectory_time_step_ = params.trajectory_time_step;
    params_ = params.stopping_constraints;
  };

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;

private:
  TrajectoryModifierParams::StoppingConstraints params_;

  size_t update_velocities(
    TrajectoryPoints & trajectory, const double jerk, const double decel) const;
};

}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_MODIFIER_HPP_
