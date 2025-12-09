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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_point_fixer_utils.hpp"

#include <Eigen/Core>
#include <autoware_utils_geometry/geometry.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::trajectory_optimizer::plugin::trajectory_point_fixer_utils
{

std::vector<std::vector<size_t>> get_close_proximity_clusters(
  const TrajectoryPoints & traj_points, const double min_dist_m)
{
  std::vector<std::vector<size_t>> clusters_of_indices;
  std::vector<size_t> current_cluster_indices{0};

  for (size_t i = 1; i < traj_points.size(); ++i) {
    const double dist = autoware_utils_geometry::calc_distance2d(
      traj_points[i], traj_points[current_cluster_indices.back()]);
    if (dist < min_dist_m) {
      current_cluster_indices.push_back(i);
      continue;
    }
    if (current_cluster_indices.size() > 1) {
      clusters_of_indices.push_back(current_cluster_indices);
    }
    current_cluster_indices.clear();
    current_cluster_indices.push_back(i);
  }

  if (current_cluster_indices.size() > 1) {
    clusters_of_indices.push_back(current_cluster_indices);
  }
  return clusters_of_indices;
}

TrajectoryPoint create_ego_point_from_odometry(const Odometry & current_odometry)
{
  TrajectoryPoint ego_point;
  ego_point.time_from_start.sec = 0;
  ego_point.time_from_start.nanosec = 0;
  ego_point.pose = current_odometry.pose.pose;
  ego_point.longitudinal_velocity_mps = static_cast<float>(current_odometry.twist.twist.linear.x);
  return ego_point;
}

double calculate_cluster_reference_yaw(
  const std::vector<size_t> & cluster_of_indices, const TrajectoryPoints & traj_points,
  const TrajectoryPoint & ego_point)
{
  if (cluster_of_indices.front() > 1) {
    Eigen::Vector2d prev_point(
      traj_points[cluster_of_indices.front() - 1].pose.position.x,
      traj_points[cluster_of_indices.front() - 1].pose.position.y);
    Eigen::Vector2d prev_prev_point(
      traj_points[cluster_of_indices.front() - 2].pose.position.x,
      traj_points[cluster_of_indices.front() - 2].pose.position.y);
    Eigen::Vector2d dir_vector = prev_point - prev_prev_point;
    return std::atan2(dir_vector.y(), dir_vector.x());
  }

  tf2::Quaternion q;
  tf2::convert(ego_point.pose.orientation, q);
  return tf2::getYaw(q);
}

std::vector<double> compute_cluster_arc_lengths(
  const std::vector<size_t> & cluster_of_indices, const TrajectoryPoints & traj_points)
{
  std::vector<double> arc_lengths{};
  arc_lengths.reserve(cluster_of_indices.size());

  auto prev_point = traj_points[cluster_of_indices.front()];
  for (const auto idx : cluster_of_indices) {
    const auto distance = (arc_lengths.empty()) ? 0.0
                                                : autoware_utils_geometry::calc_distance2d(
                                                    prev_point, traj_points[idx]) +
                                                    arc_lengths.back();
    arc_lengths.push_back(distance);
    prev_point = traj_points[idx];
  }

  return arc_lengths;
}

std::vector<double> normalize_values(const std::vector<double> & values)
{
  std::vector<double> normalized{};
  normalized.reserve(values.size());

  if (values.empty()) {
    return normalized;
  }

  const double total_length = values.back();
  for (const auto value : values) {
    normalized.push_back(total_length > 1e-6 ? value / total_length : 0.0);
  }

  return normalized;
}

void resample_single_cluster(
  const std::vector<size_t> & cluster_of_indices, TrajectoryPoints & traj_points,
  const TrajectoryPoint & ego_point)
{
  if (cluster_of_indices.empty()) {
    return;
  }

  const double yaw = calculate_cluster_reference_yaw(cluster_of_indices, traj_points, ego_point);

  const auto arc_lengths = compute_cluster_arc_lengths(cluster_of_indices, traj_points);
  if (arc_lengths.empty()) {
    return;
  }
  const auto normalized_arc_lengths = normalize_values(arc_lengths);

  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const auto idx : cluster_of_indices) {
    const auto & point = traj_points[idx];
    sum_x += point.pose.position.x;
    sum_y += point.pose.position.y;
  }
  const auto cluster_size = static_cast<double>(cluster_of_indices.size());
  const auto avg_position_x = sum_x / cluster_size;
  const auto avg_position_y = sum_y / cluster_size;

  tf2::Quaternion orientation_q;
  orientation_q.setRPY(0.0, 0.0, yaw);

  std::vector<double> projection_lengths;
  projection_lengths.reserve(cluster_of_indices.size());
  const Eigen::Vector2d line_dir(std::cos(yaw), std::sin(yaw));

  for (const auto idx : cluster_of_indices) {
    const auto & point = traj_points[idx];
    const Eigen::Vector2d vec_point(
      point.pose.position.x - avg_position_x, point.pose.position.y - avg_position_y);
    const double projection_length = vec_point.dot(line_dir);
    projection_lengths.push_back(projection_length);
  }

  std::sort(projection_lengths.begin(), projection_lengths.end());
  const auto & largest_projection_length = projection_lengths.back();
  const auto & smallest_projection_length = projection_lengths.front();
  const auto projection_length_range = largest_projection_length - smallest_projection_length;

  TrajectoryPoint initial_vector_point;
  initial_vector_point.pose.position.x = avg_position_x + smallest_projection_length * line_dir.x();
  initial_vector_point.pose.position.y = avg_position_y + smallest_projection_length * line_dir.y();

  for (size_t i = 0; i < cluster_of_indices.size(); ++i) {
    const auto idx = cluster_of_indices[i];
    const auto normalized_projected_length = normalized_arc_lengths[i] * projection_length_range;

    auto & point = traj_points[idx];
    point.pose.position.x =
      initial_vector_point.pose.position.x + normalized_projected_length * line_dir.x();
    point.pose.position.y =
      initial_vector_point.pose.position.y + normalized_projected_length * line_dir.y();
    point.pose.orientation = tf2::toMsg(orientation_q);
  }
}

void resample_close_proximity_points(
  TrajectoryPoints & traj_points, const Odometry & current_odometry, const double min_dist_m)
{
  if (traj_points.size() < 2) {
    return;
  }

  const auto clusters_of_indices = get_close_proximity_clusters(traj_points, min_dist_m);
  if (clusters_of_indices.empty()) {
    return;
  }

  const auto ego_point = create_ego_point_from_odometry(current_odometry);

  for (const auto & cluster_of_indices : clusters_of_indices) {
    resample_single_cluster(cluster_of_indices, traj_points, ego_point);
  }
}

}  // namespace autoware::trajectory_optimizer::plugin::trajectory_point_fixer_utils
