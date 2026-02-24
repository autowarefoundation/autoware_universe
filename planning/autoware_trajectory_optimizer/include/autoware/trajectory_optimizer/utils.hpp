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
 * @brief Gets the logger for the trajectory optimizer.
 *
 * @return The logger instance.
 */
rclcpp::Logger get_logger();

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

};  // namespace autoware::trajectory_optimizer::utils

#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__UTILS_HPP_
