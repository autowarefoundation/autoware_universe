// Copyright 2021 Tier IV, Inc.
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

#include "autoware/planning_evaluator/metrics/metrics_utils.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <boost/geometry.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cmath>

namespace planning_diagnostics
{
namespace metrics
{
namespace utils
{
namespace bg = boost::geometry;

// Debug helper function to visualize polygon as ASCII art
void debug_draw_polygon(
  const autoware_utils::Polygon2d & polygon,
  const std::string & label = "Polygon",
  int width = 80,
  int height = 40)
{
  if (polygon.outer().empty()) {
    std::cout << "[DEBUG] " << label << " is empty\n";
    return;
  }

  // Find bounding box
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto & point : polygon.outer()) {
    min_x = std::min(min_x, bg::get<0>(point));
    max_x = std::max(max_x, bg::get<0>(point));
    min_y = std::min(min_y, bg::get<1>(point));
    max_y = std::max(max_y, bg::get<1>(point));
  }

  double range_x = max_x - min_x;
  double range_y = max_y - min_y;

  if (range_x < 1e-6 || range_y < 1e-6) {
    std::cout << "[DEBUG] " << label << " is too small to visualize\n";
    return;
  }

  // Create canvas
  std::vector<std::vector<char>> canvas(height, std::vector<char>(width, ' '));

  // Draw polygon edges
  const auto & outer = polygon.outer();
  for (size_t i = 0; i < outer.size(); ++i) {
    const auto & p1 = outer[i];
    const auto & p2 = outer[(i + 1) % outer.size()];

    double x1 = (bg::get<0>(p1) - min_x) / range_x * (width - 1);
    double y1 = (bg::get<1>(p1) - min_y) / range_y * (height - 1);
    double x2 = (bg::get<0>(p2) - min_x) / range_x * (width - 1);
    double y2 = (bg::get<1>(p2) - min_y) / range_y * (height - 1);

    // Simple line drawing (Bresenham-like)
    int ix1 = static_cast<int>(x1);
    int iy1 = height - 1 - static_cast<int>(y1);
    int ix2 = static_cast<int>(x2);
    int iy2 = height - 1 - static_cast<int>(y2);

    int dx = std::abs(ix2 - ix1);
    int dy = std::abs(iy2 - iy1);
    int sx = (ix1 < ix2) ? 1 : -1;
    int sy = (iy1 < iy2) ? 1 : -1;
    int err = dx - dy;

    int x = ix1, y = iy1;
    for (int step = 0; step < std::max(dx, dy) + 1; ++step) {
      if (x >= 0 && x < width && y >= 0 && y < height) {
        canvas[y][x] = '*';
      }
      if (x == ix2 && y == iy2) break;

      int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x += sx; }
      if (e2 < dx) { err += dx; y += sy; }
    }
  }

  // Print canvas
  std::cout << "\n[DEBUG] " << label << " Visualization:\n";
  std::cout << "Bounds: X[" << std::fixed << std::setprecision(2)
            << min_x << ", " << max_x << "] Y[" << min_y << ", " << max_y << "]\n";
  std::cout << "+" << std::string(width, '-') << "+\n";
  for (const auto & row : canvas) {
    std::cout << "|";
    for (char c : row) {
      std::cout << c;
    }
    std::cout << "|\n";
  }
  std::cout << "+" << std::string(width, '-') << "+\n\n";
}

size_t getIndexAfterDistance(const Trajectory & traj, const size_t curr_id, const double distance)
{
  // Get Current Trajectory Point
  const TrajectoryPoint & curr_p = traj.points.at(curr_id);

  size_t target_id = curr_id;
  for (size_t traj_id = curr_id + 1; traj_id < traj.points.size(); ++traj_id) {
    double current_distance = autoware_utils::calc_distance2d(traj.points.at(traj_id), curr_p);
    if (current_distance >= distance) {
      target_id = traj_id;
      break;
    }
  }

  return target_id;
}

Trajectory get_lookahead_trajectory(
  const Trajectory & traj, const Pose & ego_pose, const double max_dist_m, const double max_time_s)
{
  if (traj.points.empty()) {
    return traj;
  }

  const auto ego_index =
    autoware::motion_utils::findNearestSegmentIndex(traj.points, ego_pose.position);
  Trajectory lookahead_traj;
  lookahead_traj.header = traj.header;
  double dist = 0.0;
  double time = 0.0;
  auto curr_point_it = std::next(traj.points.begin(), ego_index);
  auto prev_point_it = curr_point_it;
  while (curr_point_it != traj.points.end() && dist <= max_dist_m && time <= max_time_s) {
    lookahead_traj.points.push_back(*curr_point_it);
    const auto d =
      autoware_utils::calc_distance2d(prev_point_it->pose.position, curr_point_it->pose.position);
    dist += d;
    if (prev_point_it->longitudinal_velocity_mps != 0.0) {
      time += d / std::abs(prev_point_it->longitudinal_velocity_mps);
    }
    prev_point_it = curr_point_it;
    ++curr_point_it;
  }
  return lookahead_traj;
}

double calc_lookahead_trajectory_distance(const Trajectory & traj, const Pose & ego_pose)
{
  const auto ego_index =
    autoware::motion_utils::findNearestSegmentIndex(traj.points, ego_pose.position);
  double dist = 0.0;
  auto curr_point_it = std::next(traj.points.begin(), ego_index);
  auto prev_point_it = curr_point_it;
  for (size_t i = 0; i < traj.points.size(); ++i) {
    const auto d =
      autoware_utils::calc_distance2d(prev_point_it->pose.position, curr_point_it->pose.position);
    dist += d;
    prev_point_it = curr_point_it;
    ++curr_point_it;
  }

  return dist;
}

autoware_utils::Polygon2d create_pose_footprint(
  const autoware_utils::LinearRing2d & local_ego_footprint, const Pose & ego_pose)
{
  // create ego polygon
  const autoware_utils::LinearRing2d ego_footprint =
    autoware_utils::transform_vector(local_ego_footprint, autoware_utils::pose2transform(ego_pose));
  autoware_utils::Polygon2d ego_polygon;
  ego_polygon.outer() = ego_footprint;
  bg::correct(ego_polygon);

  return ego_polygon;
}



autoware_utils::Polygon2d create_trajectory_footprint(
  const VehicleInfo & vehicle_info, const Trajectory & traj, const Pose & ego_pose)
{
  autoware_utils::LinearRing2d & local_ego_footprint = vehicle_info.createFootprint();

  if (traj.points.empty()) {
    return create_pose_footprint(local_ego_footprint, ego_pose);
  }

  // Step0. Find closest point to ego pose to cut past trajectory
  size_t p0_index = 0;
  double last_dist_to_ego = calc_distance2d(ego_pose, traj.points.front().pose);
  for (size_t i = 1; i < traj.points.size(); ++i) {
    const double dist_to_ego = calc_distance2d(ego_pose, traj.points.at(i).pose);
    if (dist_to_ego > last_dist_to_ego) {
      break;
    }
    last_dist_to_ego = dist_to_ego;
    p0_index = i;
  }

  if (p0_index >= traj.points.size() - 1) {
    return create_pose_footprint(local_ego_footprint, ego_pose);
  }

  // Step1. Sample and interpolate trajectory poses
  // Adaptive resampling / interpolation thresholds based on vehicle size
  const double min_resampling_distance = vehicle_info.vehicle_length_m * 0.2;
  const double max_gap_distance = vehicle_info.vehicle_length_m * 0.5;

  // Helper function to interpolate pose between two poses
  auto interpolate_pose = [](const Pose & p1, const Pose & p2, double ratio) -> Pose {
    Pose interpolated;

    // Linear interpolation for position
    interpolated.position.x = p1.position.x + (p2.position.x - p1.position.x) * ratio;
    interpolated.position.y = p1.position.y + (p2.position.y - p1.position.y) * ratio;
    interpolated.position.z = p1.position.z + (p2.position.z - p1.position.z) * ratio;

    // orientation from direction vector (p1 -> p2)
    const double dx = p2.position.x - p1.position.x;
    const double dy = p2.position.y - p1.position.y;
    const double half_yaw = std::atan2(dy, dx) * 0.5;
    interpolated.orientation.x = 0.0;
    interpolated.orientation.y = 0.0;
    interpolated.orientation.z = std::sin(half_yaw);
    interpolated.orientation.w = std::cos(half_yaw);

    return interpolated;
  };
  if
  std::vector<Pose> sampled_poses;
  sampled_poses.push_back(traj.points[p0_index].pose);

  size_t last_sampled_idx = p0_index;
  double accumulated_dist = 0.0;

  for (size_t i = p0_index + 1; i < traj.points.size(); ++i) {
    const auto & prev = traj.points[i - 1];
    const auto & current = traj.points[i];
    accumulated_dist += autoware_utils::calc_distance2d(prev.pose, current.pose);

    if (accumulated_dist >= min_resampling_distance) {
      // Check gap distance from last sampled point
      const double gap_dist = autoware_utils::calc_distance2d(
        traj.points[last_sampled_idx].pose, current.pose);

      // If gap is too large, create interpolated poses
      if (gap_dist > max_gap_distance) {
        const size_t num_intermediate = static_cast<size_t>(std::ceil(gap_dist / max_gap_distance));
        for (size_t j = 1; j < num_intermediate; ++j) {
          const double ratio = static_cast<double>(j) / static_cast<double>(num_intermediate);
          sampled_poses.push_back(interpolate_pose(
            traj.points[last_sampled_idx].pose, current.pose, ratio));
        }
      }
      sampled_poses.push_back(current.pose);
      last_sampled_idx = i;
      accumulated_dist = 0.0;
    }
  }

  // Step2. Create swept area polygon for footprint front edge and rear edge

  // a array/list to hold swept_o
  for (const auto & pose : sampled_poses) {
    const auto transform = autoware_utils::pose2transform(pose);
    footprints.push_back(autoware_utils::transform_vector(local_ego_footprint, transform));
  }

  // // Create footprints at sampled poses
  // const auto local_footprint = vehicle_info.createFootprint();
  // std::vector<autoware_utils::Polygon2d> footprint_polygons;
  // footprint_polygons.reserve(sampled_poses.size());

  // for (const auto & pose : sampled_poses) {
  //   const auto transform = autoware_utils::pose2transform(pose);
  //   const auto transformed_footprint = autoware_utils::transform_vector(local_footprint, transform);
  //   autoware_utils::Polygon2d footprint_polygon;
  //   footprint_polygon.outer() = transformed_footprint;
  //   bg::correct(footprint_polygon);
  //   footprint_polygons.push_back(footprint_polygon);
  // }

  // // Union all footprint polygons progressively
  // // Use batch union for better performance: union pairs, then union results
  // std::vector<autoware_utils::Polygon2d> current_level = footprint_polygons;

  // while (current_level.size() > 1) {
  //   std::vector<autoware_utils::Polygon2d> next_level;

  //   for (size_t i = 0; i < current_level.size(); i += 2) {
  //     if (i + 1 < current_level.size()) {
  //       // Union pair of polygons
  //       std::vector<autoware_utils::Polygon2d> union_result;
  //       bg::union_(current_level[i], current_level[i + 1], union_result);

  //       if (!union_result.empty()) {
  //         // Take the largest polygon if multiple results
  //         auto largest = std::max_element(
  //           union_result.begin(), union_result.end(),
  //           [](const auto & a, const auto & b) { return bg::area(a) < bg::area(b); });
  //         next_level.push_back(*largest);
  //       } else {
  //         // Fallback: keep first polygon if union fails
  //         next_level.push_back(current_level[i]);
  //       }
  //     } else {
  //       // Odd one out, carry forward
  //       next_level.push_back(current_level[i]);
  //     }
  //   }

  //   current_level = std::move(next_level);
  // }

  // autoware_utils::Polygon2d trajectory_footprint = current_level[0];

  // DEBUG: Visualize the trajectory footprint
  std::cout << "[DEBUG] Vehicle length: " << std::fixed << std::setprecision(2) << vehicle_length << "m\n";
  std::cout << "[DEBUG] Vehicle width: " << vehicle_width << "m\n";
  std::cout << "[DEBUG] Resampling distance: " << min_resampling_distance << "m (0.2x vehicle length)\n";
  std::cout << "[DEBUG] Max gap distance: " << max_gap_distance << "m (0.5x vehicle length)\n";
  std::cout << "[DEBUG] Sampled " << sampled_poses.size() << " poses from "
            << traj.points.size() << " trajectory points\n";
  debug_draw_polygon(trajectory_footprint, "Trajectory Footprint (Adaptive Resampling)", 100, 30);

  return trajectory_footprint;
}
}  // namespace utils
}  // namespace metrics
}  // namespace planning_diagnostics
