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

#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__UTILS_HPP_
#define AUTOWARE__TRAJECTORY_OPTIMIZER__UTILS_HPP_

#include "autoware/path_smoother/elastic_band.hpp"
#include "autoware/path_smoother/replan_checker.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"
#include "autoware/velocity_smoother/smoother/jerk_filtered_smoother.hpp"

#include <rclcpp/logger.hpp>

#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_perception_msgs/msg/detail/predicted_objects__struct.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/detail/trajectory__struct.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>

#include <memory>
#include <vector>

namespace autoware::trajectory_optimizer::utils
{

using autoware::path_smoother::CommonParam;
using autoware::path_smoother::EBPathSmoother;
using autoware::path_smoother::EgoNearestParam;
using autoware::path_smoother::PlannerData;
using autoware::path_smoother::ReplanChecker;

using autoware::velocity_smoother::JerkFilteredSmoother;
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

void smooth_trajectory_with_elastic_band(
  TrajectoryPoints & traj_points, const Odometry & current_odometry,
  const std::shared_ptr<EBPathSmoother> & eb_path_smoother_ptr);

/**
 * @brief Checks if a trajectory point is valid.
 * @param point The point to be validated.
 * @return True if the trajectory point is valid, false otherwise.
 */
bool validate_point(const TrajectoryPoint & point);

/**
 * @brief Copies orientations from input trajectory to output trajectory points.
 * @param input_trajectory The reference input trajectory points.
 * @param output_trajectory The output trajectory points to be updated.
 * @param max_distance_m Maximum position deviation allowed for nearest neighbor matching.
 * @param max_yaw_rad Maximum yaw deviation allowed for nearest neighbor matching.
 *
 * For each output point, finds the nearest input point within the distance and yaw constraints
 * and copies its orientation. If no match is found, the output orientation is preserved.
 */
void copy_trajectory_orientation(
  const TrajectoryPoints & input_trajectory, TrajectoryPoints & output_trajectory,
  const double max_distance_m, const double max_yaw_rad);

/**
 * @brief Interpolates the given trajectory points based on trajectory length.
 *
 * @param traj_points The trajectory points to be interpolated.
 * @param interpolation_resolution_m Interpolation resolution for Akima spline.
 * @param max_distance_discrepancy_m Maximum position deviation allowed for orientation copying.
 * @param preserve_original_orientation Flag to indicate if orientation from original trajectory
 * should be copied.
 */
void apply_spline(
  TrajectoryPoints & traj_points, const double interpolation_resolution_m,
  const double max_distance_discrepancy_m, const bool preserve_original_orientation);

/**
 * @brief Gets the logger for the trajectory optimizer.
 *
 * @return The logger instance.
 */
rclcpp::Logger get_logger();

/**
 * @brief Removes invalid points from the input trajectory.
 *
 * @param input_trajectory The trajectory points to be cleaned.
 * @param min_dist_to_remove_m Minimum distance to remove close proximity points [m].
 */
void remove_invalid_points(
  std::vector<TrajectoryPoint> & input_trajectory, const double min_dist_to_remove_m = 1E-2);

/**
 * @brief Filters the velocity of the input trajectory based on the initial motion and parameters.
 *
 * @param input_trajectory The trajectory points to be filtered.
 * @param initial_motion The initial speed and acceleration for motion.
 * @param nearest_dist_threshold_m Distance threshold for trajectory matching.
 * @param nearest_yaw_threshold_rad Yaw threshold for trajectory matching.
 * @param smoother The smoother to be used for filtering the trajectory.
 * @param current_odometry The current odometry data.
 */
void filter_velocity(
  TrajectoryPoints & input_trajectory, const InitialMotion & initial_motion,
  double nearest_dist_threshold_m, double nearest_yaw_threshold_rad,
  const std::shared_ptr<JerkFilteredSmoother> & smoother, const Odometry & current_odometry);

/**
 * @brief Clamps the velocities of the input trajectory points to the specified minimum values.
 *
 * @param input_trajectory_array The trajectory points to be clamped.
 * @param min_velocity The minimum velocity to be clamped.
 * @param min_acceleration The minimum acceleration to be clamped.
 */
void clamp_velocities(
  std::vector<TrajectoryPoint> & input_trajectory_array, float min_velocity,
  float min_acceleration);

/**
 * @brief Sets the maximum velocity for the input trajectory points.
 *
 * @param input_trajectory_array The trajectory points to be updated.
 * @param max_velocity The maximum velocity to be set.
 */
void set_max_velocity(
  std::vector<TrajectoryPoint> & input_trajectory_array, const float max_velocity);

/**
 * @brief Compute time difference between consecutive trajectory points
 *
 * @param current Current trajectory point
 * @param next Next trajectory point
 * @return Time difference [s]
 */
double compute_dt(const TrajectoryPoint & current, const TrajectoryPoint & next);

/**
 * @brief Recalculates longitudinal acceleration from velocity differences.
 *
 * @param trajectory The trajectory points with velocities to recalculate accelerations from.
 * @param use_constant_dt If true, use constant_dt; if false, use time_from_start spacing.
 * @param constant_dt Constant time step in seconds (used only if use_constant_dt is true).
 */
void recalculate_longitudinal_acceleration(
  TrajectoryPoints & trajectory, const bool use_constant_dt = false,
  const double constant_dt = 0.1);

void limit_lateral_acceleration(
  TrajectoryPoints & input_trajectory_array, double max_lateral_accel_mps2,
  const Odometry & current_odometry);

/**
 * @brief Removes points from the input trajectory that are too close to each other.
 *
 * @param input_trajectory_array The trajectory points to be cleaned.
 * @param min_dist The minimum distance between points.
 */
void remove_close_proximity_points(
  std::vector<TrajectoryPoint> & input_trajectory_array, const double min_dist = 1E-2);

/**
 * @brief Adds the ego state to the trajectory points.
 *
 * @param traj_points The trajectory points to be updated.
 * @param current_odometry The current odometry data.
 * @param params The parameters for trajectory interpolation.
 */
void add_ego_state_to_trajectory(
  TrajectoryPoints & traj_points, const Odometry & current_odometry,
  double nearest_dist_threshold_m, double nearest_yaw_threshold_rad,
  double backward_trajectory_extension_m);

/**
 * @brief Expands the trajectory points with the ego history points.
 *
 * @param traj_points The trajectory points to be expanded.
 * @param ego_history_points The ego history points to be added.
 */
void expand_trajectory_with_ego_history(
  TrajectoryPoints & traj_points, const TrajectoryPoints & ego_history_points,
  const Odometry & current_odometry);

};  // namespace autoware::trajectory_optimizer::utils

#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__UTILS_HPP_
