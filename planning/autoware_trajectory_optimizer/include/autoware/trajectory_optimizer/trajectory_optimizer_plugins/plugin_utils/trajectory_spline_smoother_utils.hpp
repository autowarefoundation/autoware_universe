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

// NOLINTNEXTLINE
#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_SPLINE_SMOOTHER_UTILS_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_SPLINE_SMOOTHER_UTILS_HPP_

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace autoware::trajectory_optimizer::plugin::trajectory_spline_smoother_utils
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

/**
 * @brief Interpolates trajectory using Akima spline interpolation.
 *
 * Applies Akima spline interpolation to smooth the trajectory path with configurable
 * resolution. Optionally preserves the original trajectory's orientation by copying
 * from nearest points using copy_trajectory_orientation from common utils.
 *
 * @param traj_points The trajectory points to be interpolated (modified in place)
 * @param interpolation_resolution_m Interpolation resolution for Akima spline
 * @param max_distance_discrepancy_m Maximum position deviation for orientation copying
 * @param preserve_original_orientation If true, copies orientation from original trajectory
 */
void apply_spline(
  TrajectoryPoints & traj_points, double interpolation_resolution_m,
  double max_distance_discrepancy_m, bool preserve_original_orientation);

}  // namespace autoware::trajectory_optimizer::plugin::trajectory_spline_smoother_utils

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_SPLINE_SMOOTHER_UTILS_HPP_
