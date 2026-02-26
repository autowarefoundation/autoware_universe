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
#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_VELOCITY_OPTIMIZER_UTILS_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_VELOCITY_OPTIMIZER_UTILS_HPP_

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/continuous_jerk_smoother.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <vector>

namespace autoware::trajectory_optimizer::plugin::trajectory_velocity_optimizer_utils
{
using autoware::trajectory_optimizer::InitialMotion;
using autoware::trajectory_optimizer::plugin::ContinuousJerkSmoother;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using nav_msgs::msg::Odometry;

/**
 * @brief Clamps the velocities of trajectory points to specified minimum values.
 *
 * Ensures all points have at least the minimum velocity and acceleration.
 *
 * @param input_trajectory_array The trajectory points to be clamped (modified in place)
 * @param min_velocity The minimum velocity to enforce
 * @param min_acceleration The minimum acceleration to enforce
 */
void clamp_velocities(
  TrajectoryPoints & input_trajectory_array, const float min_velocity,
  const float min_acceleration);

/**
 * @brief Sets the maximum velocity for trajectory points while preserving dynamics.
 *
 * This function identifies segments where velocity exceeds max_velocity and caps them
 * while preserving time intervals computed from velocity/acceleration relationships.
 * Interior accelerations in capped segments are set to 0 (constant velocity), and
 * boundary accelerations are recalculated for smooth transitions.
 *
 * @param input_trajectory_array The trajectory points to be updated (modified in place)
 * @param max_velocity The maximum velocity to enforce
 */
void set_max_velocity(TrajectoryPoints & input_trajectory_array, const float max_velocity);

/**
 * @brief Limits lateral acceleration by reducing velocity at high curvature points.
 *
 * Calculates lateral acceleration from yaw rate and velocity, then reduces velocity
 * where lateral acceleration exceeds the limit. Updates max_velocity_per_point constraints
 * rather than directly modifying trajectory velocities, allowing the QP smoother to handle
 * optimization systematically.
 *
 * @param input_trajectory_array The trajectory points (used for reading geometry/time, not
 * modified)
 * @param max_velocity_per_point Per-point velocity upper bounds (modified in place)
 * @param max_lateral_accel_mps2 Maximum allowed lateral acceleration
 * @param min_limited_speed_mps Minimum speed when applying lateral acceleration limit
 * @param current_odometry Current vehicle odometry for time calculation
 * @param inplace If true, directly modify trajectory velocities; if false, only update
 */
void limit_lateral_acceleration(
  TrajectoryPoints & input_trajectory_array, std::vector<double> & max_velocity_per_point,
  const double max_lateral_accel_mps2, const double min_limited_speed_mps,
  const Odometry & current_odometry, const bool inplace = false);

/**
 * @brief Filters velocity profile using jerk-constrained smoothing.
 *
 * Applies jerk-filtered velocity optimization using the ContinuousJerkSmoother.
 * This smoother is designed for continuous trajectories without stop points.
 *
 * @param input_trajectory The trajectory points to be filtered (modified in place)
 * @param initial_motion The initial speed and acceleration for motion
 * @param nearest_dist_threshold_m Distance threshold for trajectory matching
 * @param nearest_yaw_threshold_rad Yaw threshold for trajectory matching
 * @param smoother The continuous jerk smoother instance
 * @param current_odometry The current vehicle odometry data
 * @param max_velocity_per_point Per-point velocity upper bounds [m/s].
 *        Should be initialized with max_speed_mps and updated by lateral acceleration limits.
 *        This allows enforcing lateral acceleration limits as hard constraints in the QP.
 */
void filter_velocity(
  TrajectoryPoints & input_trajectory, const InitialMotion & initial_motion,
  const double nearest_dist_threshold_m, const double nearest_yaw_threshold_rad,
  const std::shared_ptr<ContinuousJerkSmoother> & smoother, const Odometry & current_odometry,
  const std::vector<double> & max_velocity_per_point = {});

}  // namespace autoware::trajectory_optimizer::plugin::trajectory_velocity_optimizer_utils

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_VELOCITY_OPTIMIZER_UTILS_HPP_
