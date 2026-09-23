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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__STOP_POINT_FIXER_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__STOP_POINT_FIXER_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugin_base.hpp"

namespace autoware::trajectory_modifier::plugin
{
using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::TrajectoryModifierParams;
using autoware::trajectory_modifier::plugin::ProcessingResult;
using autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using ModifierParams = trajectory_modifier_params::Params;

class StopPointFixer : public TrajectoryModifierPluginBase
{
public:
  StopPointFixer() = default;

  ProcessingResult process(TrajectoryPoints & traj_points, TrajectoryModifierData & input) override;

  bool is_long_stop_trajectory(const TrajectoryPoints & traj_points) const;
  bool is_stop_point_close_to_ego(
    const TrajectoryPoints & traj_points, const TrajectoryModifierData & input) const;
  [[nodiscard]] bool is_trajectory_modification_required(
    const TrajectoryPoints & traj_points, const TrajectoryModifierData & input);

  void update_params(const TrajectoryModifierParams & params) override
  {
    params_ = params.stop_point_fixer;
    enabled_ = params.use_stop_point_fixer;
  }

  const ModifierParams::StopPointFixer & get_params() const { return params_; }

protected:
  void on_initialize(const TrajectoryModifierParams & params) override;

private:
  ModifierParams::StopPointFixer params_;
};

}  // namespace autoware::trajectory_modifier::plugin

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__STOP_POINT_FIXER_HPP_
