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
 * @brief Copies the orientation of the input trajectory points to the output trajectory points.
 * @param input_trajectory The input trajectory points.
 * @param output_trajectory The output trajectory points.
 * @param params The parameters for trajectory optimization.
 */
void copy_trajectory_orientation(
  const TrajectoryPoints & input_trajectory, TrajectoryPoints & output_trajectory,
  const TrajectoryOptimizerParams & params);

/**
 * @brief Interpolates the given trajectory points based on trajectory length.
 *
 * @param traj_points The trajectory points to be interpolated.
 * @param params The parameters for trajectory interpolation.
 */
void apply_spline(TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params);

/**
 * @brief Interpolates the given trajectory points based on the current odometry and acceleration.
 *
 * @param traj_points The trajectory points to be interpolated.
 * @param current_odometry The current odometry data.
 * @param current_acceleration The current acceleration data.
 * @param params The parameters for trajectory interpolation.
 * @param smoother The smoother to be used for filtering the trajectory.
 */
void interpolate_trajectory(
  TrajectoryPoints & traj_points, const Odometry & current_odometry,
  const AccelWithCovarianceStamped & current_acceleration, const TrajectoryOptimizerParams & params,
  const std::shared_ptr<JerkFilteredSmoother> & jerk_filtered_smoother,
  const std::shared_ptr<EBPathSmoother> & eb_path_smoother_ptr);

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
 */
void remove_invalid_points(std::vector<TrajectoryPoint> & input_trajectory);

/**
 * @brief Filters the velocity of the input trajectory based on the initial motion and parameters.
 *
 * @param input_trajectory The trajectory points to be filtered.
 * @param initial_motion_speed The initial speed and acceleration for motion.
 * @param params The parameters for trajectory interpolation.
 * @param smoother The smoother to be used for filtering the trajectory.
 * @param current_odometry The current odometry data.
 */
void filter_velocity(
  TrajectoryPoints & input_trajectory, const InitialMotion & initial_motion,
  const TrajectoryOptimizerParams & params, const std::shared_ptr<JerkFilteredSmoother> & smoother,
  const Odometry & current_odometry);

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

void limit_lateral_acceleration(
  TrajectoryPoints & input_trajectory_array, const TrajectoryOptimizerParams & params);

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
  const TrajectoryOptimizerParams & params);

/**
 * @brief Expands the trajectory points with the ego history points.
 *
 * @param traj_points The trajectory points to be expanded.
 * @param ego_history_points The ego history points to be added.
 */
void expand_trajectory_with_ego_history(
  TrajectoryPoints & traj_points, const TrajectoryPoints & ego_history_points,
  const Odometry & current_odometry, const TrajectoryOptimizerParams & params);

};  // namespace autoware::trajectory_optimizer::utils

#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__UTILS_HPP_
