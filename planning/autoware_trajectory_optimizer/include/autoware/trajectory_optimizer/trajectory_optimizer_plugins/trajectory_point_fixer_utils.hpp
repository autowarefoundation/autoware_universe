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

#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_POINT_FIXER_UTILS_HPP_  // NOLINT
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_POINT_FIXER_UTILS_HPP_  // NOLINT

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <vector>

namespace autoware::trajectory_optimizer::plugin::trajectory_point_fixer_utils
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using nav_msgs::msg::Odometry;

/**
 * @brief Identifies clusters of consecutive trajectory points closer than the minimum distance.
 *
 * @param traj_points The trajectory points to analyze
 * @param min_dist_m The minimum distance threshold for clustering
 * @return Vector of clusters, where each cluster is a vector of point indices
 */
std::vector<std::vector<size_t>> get_close_proximity_clusters(
  const TrajectoryPoints & traj_points, const double min_dist_m);

/**
 * @brief Converts odometry message to trajectory point format.
 *
 * @param current_odometry The current odometry data
 * @return TrajectoryPoint representing the ego vehicle state
 */
TrajectoryPoint create_ego_point_from_odometry(const Odometry & current_odometry);

/**
 * @brief Calculates reference yaw angle for a cluster based on previous trajectory direction.
 *
 * Falls back to ego orientation if insufficient previous points are available.
 *
 * @param cluster_of_indices Indices of points in the cluster
 * @param traj_points Full trajectory
 * @param ego_point Current ego vehicle state
 * @return Reference yaw angle in radians
 */
double calculate_cluster_reference_yaw(
  const std::vector<size_t> & cluster_of_indices, const TrajectoryPoints & traj_points,
  const TrajectoryPoint & ego_point);

/**
 * @brief Computes cumulative arc lengths along cluster points.
 *
 * Arc lengths are sorted to ensure monotonic increase.
 *
 * @param cluster_of_indices Indices of points in the cluster
 * @param traj_points Full trajectory
 * @return Vector of cumulative arc lengths
 */
std::vector<double> compute_cluster_arc_lengths(
  const std::vector<size_t> & cluster_of_indices, const TrajectoryPoints & traj_points);

/**
 * @brief Normalizes values to [0, 1] range based on maximum value.
 *
 * @param values Input values to normalize
 * @return Normalized values in [0, 1] range
 */
std::vector<double> normalize_values(const std::vector<double> & values);

/**
 * @brief Resamples a single cluster of close points by redistributing them along a reference line.
 *
 * The algorithm:
 * 1. Determines reference yaw from previous trajectory direction
 * 2. Projects cluster points onto a line with this orientation
 * 3. Redistributes points along the line using normalized arc length spacing
 *
 * @param cluster_of_indices Indices of points in the cluster
 * @param traj_points Full trajectory (modified in place)
 * @param ego_point Current ego vehicle state
 */
void resample_single_cluster(
  const std::vector<size_t> & cluster_of_indices, TrajectoryPoints & traj_points,
  const TrajectoryPoint & ego_point);

/**
 * @brief Resamples trajectory points that are too close together.
 *
 * Identifies clusters of points closer than the threshold distance and redistributes
 * them along a line direction while preserving arc length ratios.
 *
 * @param traj_points Trajectory to resample (modified in place)
 * @param current_odometry Current ego vehicle odometry
 * @param min_dist_m Minimum distance threshold for clustering
 */
void resample_close_proximity_points(
  TrajectoryPoints & traj_points, const Odometry & current_odometry, const double min_dist_m);

}  // namespace autoware::trajectory_optimizer::plugin::trajectory_point_fixer_utils

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_POINT_FIXER_UTILS_HPP_
