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
#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_EXTENDER_UTILS_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_EXTENDER_UTILS_HPP_

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>

namespace autoware::trajectory_optimizer::plugin::trajectory_extender_utils
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using nav_msgs::msg::Odometry;

/**
 * @brief Adds the current ego vehicle state to the trajectory history.
 *
 * This function adds the current ego state to a history trajectory. It includes
 * checks to avoid adding duplicate states and limits the trajectory length
 * based on the backward extension parameter.
 *
 * @param traj_points The trajectory points to be updated (modified in place)
 * @param current_odometry The current vehicle odometry data
 * @param nearest_dist_threshold_m Distance threshold for considering states different
 * @param nearest_yaw_threshold_rad Yaw threshold for considering states different
 * @param backward_trajectory_extension_m Maximum length of backward trajectory to maintain
 */
void add_ego_state_to_trajectory(
  TrajectoryPoints & traj_points, const Odometry & current_odometry,
  const double nearest_dist_threshold_m, const double nearest_yaw_threshold_rad,
  const double backward_trajectory_extension_m);

/**
 * @brief Extends the forward trajectory with the ego history points.
 *
 * This function prepends historical ego states to the beginning of the trajectory
 * to extend it backward in time. It filters out history points that are already
 * present in the trajectory or are ahead of the first trajectory point.
 *
 * @param traj_points The forward trajectory points to be extended (modified in place)
 * @param ego_history_points The historical ego state points to prepend
 * @param current_odometry The current vehicle odometry data
 */
void expand_trajectory_with_ego_history(
  TrajectoryPoints & traj_points, const TrajectoryPoints & ego_history_points,
  const Odometry & current_odometry);

}  // namespace autoware::trajectory_optimizer::plugin::trajectory_extender_utils

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_EXTENDER_UTILS_HPP_
