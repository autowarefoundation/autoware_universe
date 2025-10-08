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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_spline_smoother.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/trajectory_optimizer/utils.hpp"

#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
void TrajectorySplineSmoother::optimize_trajectory(
  TrajectoryPoints & traj_points, [[maybe_unused]] const TrajectoryOptimizerParams & params)
{
  // Apply spline to smooth the trajectory
  if (params.use_akima_spline_interpolation) {
    // utils::apply_spline(traj_points, params);
    autoware_planning_msgs::msg::Trajectory dummy_traj;
    dummy_traj.points = traj_points;

    autoware::motion_utils::resampleTrajectory(
      dummy_traj, params.spline_interpolation_resolution_m, true, false, true, false);
    traj_points = dummy_traj.points;
  }
}

void TrajectorySplineSmoother::set_up_params()
{
}

rcl_interfaces::msg::SetParametersResult TrajectorySplineSmoother::on_parameter(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_optimizer::plugin
