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

#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_spline_smoother.hpp"

#include "autoware/trajectory_processor/trajectory_optimizer_plugins/plugin_utils/trajectory_spline_smoother_utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>

#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
void TrajectorySplineSmoother::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  TrajectoryOptimizerData & data)
{
  if (!params.use_akima_spline_interpolation) {
    return;
  }
  trajectory_spline_smoother_utils::apply_spline(
    traj_points, spline_params_.interpolation_resolution_m,
    spline_params_.max_distance_discrepancy_m,
    spline_params_.preserve_input_trajectory_orientation);
  // TODO(Daniel): The spline should recalculate time_from_start based on the new trajectory points
  // and the current vehicle position. This is necessary to ensure that the time_from_start values
  // are consistent with the new trajectory. For now, we will use the motion_utils function.
  autoware::motion_utils::calculate_time_from_start(
    traj_points, data.current_odometry.pose.pose.position);
}

void TrajectorySplineSmoother::on_initialize(const TrajectoryOptimizerParams & params)
{
  spline_params_ = params.trajectory_spline_smoother;
}

void TrajectorySplineSmoother::update_params(const TrajectoryOptimizerParams & params)
{
  spline_params_ = params.trajectory_spline_smoother;
}

}  // namespace autoware::trajectory_optimizer::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_optimizer::plugin::TrajectorySplineSmoother,
  autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase)
