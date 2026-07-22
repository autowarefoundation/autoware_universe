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

#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_extender.hpp"

#include "autoware/trajectory_processor/trajectory_optimizer_plugins/plugin_utils/trajectory_extender_utils.hpp"

#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils_math/unit_conversion.hpp>

#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
void TrajectoryExtender::optimize_trajectory(
  TrajectoryPoints & traj_points, TrajectoryOptimizerData & data)
{
  if (!enabled_) {
    return;
  }
  // Note: This function adds the current ego state to a history trajectory. Note that it is ok to
  // call this function several times with the same ego state, since there is a check inside the
  // function to avoid adding the same state multiple times.
  trajectory_extender_utils::add_ego_state_to_trajectory(
    past_ego_state_trajectory_.points, data.current_odometry,
    extender_params_.nearest_dist_threshold_m,
    autoware_utils_math::deg2rad(extender_params_.nearest_yaw_threshold_deg),
    extender_params_.backward_trajectory_extension_m);
  trajectory_extender_utils::expand_trajectory_with_ego_history(
    traj_points, past_ego_state_trajectory_.points, data.current_odometry);
}

void TrajectoryExtender::on_initialize(const TrajectoryOptimizerParams & params)
{
  enabled_ = params.use_trajectory_extender;
  extender_params_ = params.trajectory_extender;
}

void TrajectoryExtender::update_params(const TrajectoryOptimizerParams & params)
{
  enabled_ = params.use_trajectory_extender;
  extender_params_ = params.trajectory_extender;
}

}  // namespace autoware::trajectory_optimizer::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_optimizer::plugin::TrajectoryExtender,
  autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase)
