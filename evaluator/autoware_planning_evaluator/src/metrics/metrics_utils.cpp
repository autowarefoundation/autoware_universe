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

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

namespace planning_diagnostics
{
namespace metrics
{
namespace utils
{
using autoware_utils::Point2d;
using autoware_utils::Polygon2d;
using autoware_utils::Segment2d;
namespace bg = boost::geometry;

size_t get_index_after_distance(
  const Trajectory & traj, const size_t curr_id, const double distance)
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

bool polygon_intersects(const Polygon2d & poly1, const Polygon2d & poly2)
{
  const auto & outer1 = poly1.outer();
  const auto & outer2 = poly2.outer();

  // Separating Axis Theorem: test perpendicular axes of both polygon edges
  const auto test_axes = [&](const auto & poly_outer, const auto & other_outer) {
    const size_t n = poly_outer.size();
    for (size_t i = 0; i < n; ++i) {
      const auto & p1 = poly_outer[i];
      const auto & p2 = poly_outer[(i + 1) % n];

      const double edge_x = p2.x() - p1.x();
      const double edge_y = p2.y() - p1.y();
      const double edge_len_sq = edge_x * edge_x + edge_y * edge_y;
      if (edge_len_sq < 1e-2) continue;

      const double axis_x = -edge_y;
      const double axis_y = edge_x;

      double min1 = poly_outer[0].x() * axis_x + poly_outer[0].y() * axis_y;
      double max1 = min1;
      for (size_t j = 1; j < n; ++j) {
        const double proj = poly_outer[j].x() * axis_x + poly_outer[j].y() * axis_y;
        min1 = std::min(min1, proj);
        max1 = std::max(max1, proj);
      }

      const size_t m = other_outer.size();
      double min2 = other_outer[0].x() * axis_x + other_outer[0].y() * axis_y;
      double max2 = min2;
      for (size_t j = 1; j < m; ++j) {
        const double proj = other_outer[j].x() * axis_x + other_outer[j].y() * axis_y;
        min2 = std::min(min2, proj);
        max2 = std::max(max2, proj);
      }

      if (max1 < min2 || max2 < min1) {
        return false;  // Found separating axis
      }
    }
    return true;
  };

  return test_axes(outer1, outer2) && test_axes(outer2, outer1);
}

std::pair<double, double> calculate_point_to_polygon_boundary_distances(
  const Pose & pose, const Polygon2d & polygon)
{
  const Point2d point(pose.position.x, pose.position.y);
  const auto & outer = polygon.outer();

  // Calculate max distance: distance to farthest vertex
  double max_dist = 0.0;
  for (const auto & vertex : outer) {
    const double dist = bg::distance(point, vertex);
    max_dist = std::max(max_dist, dist);
  }

  // Calculate min distance: distance to nearest edge segment
  double min_dist = std::numeric_limits<double>::max();
  const size_t num_vertices = outer.size();
  for (size_t i = 0; i < num_vertices; ++i) {
    const auto & p1 = outer[i];
    const auto & p2 = outer[(i + 1) % num_vertices];

    // Use boost::geometry segment to calculate distance from point to line segment
    Segment2d segment(p1, p2);
    const double dist = bg::distance(point, segment);
    min_dist = std::min(min_dist, dist);
  }

  return {min_dist, max_dist};
}

}  // namespace utils
}  // namespace metrics
}  // namespace planning_diagnostics
