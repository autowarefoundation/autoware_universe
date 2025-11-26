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
#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_MPT_OPTIMIZER_UTILS_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_MPT_OPTIMIZER_UTILS_HPP_

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>

namespace autoware::trajectory_optimizer::plugin::trajectory_mpt_optimizer_utils
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

struct BoundsPair
{
  std::vector<geometry_msgs::msg::Point> left_bound;
  std::vector<geometry_msgs::msg::Point> right_bound;
};

/**
 * @brief Calculates acceleration based on velocity change and distance using kinematic equation.
 *
 * Uses the kinematic relation: v² = v₀² + 2as
 * Solving for acceleration: a = (v² - v₀²) / (2s)
 *
 * @param p_curr Current trajectory point
 * @param p_next Next trajectory point
 * @return Acceleration in m/s²
 */
double calculate_acceleration_from_velocity_and_distance(
  const TrajectoryPoint & p_curr, const TrajectoryPoint & p_next);

/**
 * @brief Calculates time interval based on distance, velocity, and acceleration.
 *
 * Uses kinematic equations to compute time required to travel distance s
 * with initial velocity v and acceleration a:
 * - For constant velocity (a ≈ 0): t = s / v
 * - For accelerating motion: t = (v_next - v) / a, where v_next = √(v² + 2as)
 *
 * @param v Current velocity [m/s]
 * @param a Acceleration [m/s²]
 * @param p_curr Current trajectory point
 * @param p_next Next trajectory point
 * @return Time interval in seconds
 */
double calculate_time_interval(
  const double v, const double a, const TrajectoryPoint & p_curr, const TrajectoryPoint & p_next);

/**
 * @brief Recalculates acceleration and time stamps for trajectory based on positions and
 * velocities.
 *
 * Processes trajectory points to ensure kinematic consistency:
 * 1. Sets first point time_from_start to zero
 * 2. For each segment, calculates acceleration from velocity change and distance
 * 3. Calculates time interval using kinematic equations
 * 4. Propagates time_from_start forward through trajectory
 * 5. Applies moving average filter to smooth accelerations
 * 6. Ensures last point has zero acceleration
 *
 * @param traj_points Trajectory points to update (modified in-place)
 * @param smoothing_window Moving average window size (1 = no smoothing)
 */
void recalculate_trajectory_dynamics(TrajectoryPoints & traj_points, const int smoothing_window);

/**
 * @brief Calculates local path curvature at a given point.
 *
 * Uses three consecutive points to estimate curvature via finite differences:
 * κ = Δθ / arc_length
 *
 * @param traj_points Trajectory points
 * @param idx Index of point to calculate curvature at
 * @return Curvature in rad/m (0 if insufficient points)
 */
double calculate_curvature_at_point(const TrajectoryPoints & traj_points, const size_t idx);

/**
 * @brief Calculates corridor width with simple adjustments for curvature and velocity.
 *
 * @param curvature Local path curvature [rad/m]
 * @param velocity Current velocity [m/s]
 * @param base_width Base corridor width [m]
 * @param curvature_factor Additional width per unit curvature [m/rad]
 * @param velocity_factor Additional width scaling at low speeds [m]
 * @return Adjusted corridor width [m]
 */
double calculate_corridor_width(
  const double curvature, const double velocity, const double base_width,
  const double curvature_factor, const double velocity_factor);

/**
 * @brief Generates simple perpendicular offset bounds from trajectory.
 *
 * Creates left and right bounds by offsetting perpendicular to trajectory heading.
 * This is a workaround to avoid dependency on lanelet map bounds.
 *
 * @param traj_points Input trajectory points
 * @param corridor_width_m Base perpendicular offset distance [m]
 * @param enable_adaptive_width Enable width adjustments for curvature/velocity
 * @param curvature_width_factor Additional width per unit curvature [m/rad]
 * @param velocity_width_factor Additional width scaling at low speeds [m]
 * @param min_clearance_m Minimum clearance from vehicle edges [m]
 * @param vehicle_width_m Vehicle width [m]
 * @return BoundsPair containing left and right bound points
 */
BoundsPair generate_bounds(
  const TrajectoryPoints & traj_points, const double corridor_width_m,
  const bool enable_adaptive_width, const double curvature_width_factor,
  const double velocity_width_factor, const double min_clearance_m, const double vehicle_width_m);

}  // namespace autoware::trajectory_optimizer::plugin::trajectory_mpt_optimizer_utils

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__PLUGIN_UTILS__TRAJECTORY_MPT_OPTIMIZER_UTILS_HPP_
