// Copyright 2024 TIER IV, Inc.
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

#include "footprint.hpp"

#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <tf2/utils.hpp>

#include <autoware_planning_msgs/msg/detail/trajectory_point__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/detail/intersects/interface.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <vector>

namespace autoware::motion_velocity_planner::out_of_lane
{
autoware_utils::Polygon2d make_base_footprint(const PlannerParam & p, const bool ignore_offset)
{
  autoware_utils::Polygon2d base_footprint;
  const auto front_offset = ignore_offset ? 0.0 : p.extra_front_offset;
  const auto rear_offset = ignore_offset ? 0.0 : p.extra_rear_offset;
  const auto right_offset = ignore_offset ? 0.0 : p.extra_right_offset;
  const auto left_offset = ignore_offset ? 0.0 : p.extra_left_offset;
  base_footprint.outer() = {
    {p.front_offset + front_offset, p.left_offset + left_offset},
    {p.front_offset + front_offset, p.right_offset - right_offset},
    {p.rear_offset - rear_offset, p.right_offset - right_offset},
    {p.rear_offset - rear_offset, p.left_offset + left_offset}};
  boost::geometry::correct(base_footprint);
  return base_footprint;
}

lanelet::BasicPolygon2d project_to_trajectory_point(
  const autoware_utils::Polygon2d & footprint,
  const autoware_planning_msgs::msg::TrajectoryPoint & trajectory_point)
{
  lanelet::BasicPolygon2d projected_footprint;
  const auto angle = tf2::getYaw(trajectory_point.pose.orientation);
  const auto rotated_footprint = autoware_utils::rotate_polygon(footprint, angle);
  for (const auto & p : rotated_footprint.outer()) {
    projected_footprint.emplace_back(
      p.x() + trajectory_point.pose.position.x, p.y() + trajectory_point.pose.position.y);
  }
  return projected_footprint;
}

std::vector<lanelet::BasicPolygon2d> calculate_trajectory_footprints(
  const EgoData & ego_data, const PlannerParam & params)
{
  const auto base_footprint = make_base_footprint(params);
  const auto current_ego_position =
    autoware_utils::Point2d(ego_data.pose.position.x, ego_data.pose.position.y);
  const auto current_ego_yaw = tf2::getYaw(ego_data.pose.orientation);
  std::vector<lanelet::BasicPolygon2d> trajectory_footprints;
  trajectory_footprints.reserve(ego_data.trajectory_points.size());
  // cut the first footprints to (roughly) start after the current ego front
  const auto last_cut_index = [&]() {
    auto s = 0.0;
    for (auto i = 0UL; i + 1 < ego_data.trajectory_points.size(); ++i) {
      s += autoware_utils::calc_distance2d(
        ego_data.trajectory_points[i], ego_data.trajectory_points[i + 1]);
      if (s >= params.front_offset) {
        return i + 1;
      }
    }
    return ego_data.trajectory_points.size() - 1UL;
  }();
  for (auto i = 0UL; i <= last_cut_index; ++i) {
    const auto & trajectory_point = ego_data.trajectory_points[i];
    // cut the footprint beyond the current ego front
    const autoware_utils::Point2d ego_pos_in_fp_frame(
      trajectory_point.pose.position.x - current_ego_position.x(),
      trajectory_point.pose.position.y - current_ego_position.y());
    const auto angle_diff = autoware_utils_math::normalize_radian(
      tf2::getYaw(trajectory_point.pose.orientation) - current_ego_yaw);
    const auto ego_front_x_in_trajectory_point_frame =
      ego_pos_in_fp_frame.x() + std::cos(angle_diff) * params.front_offset;
    // empty footprint if the current ego front is ahead
    if (ego_front_x_in_trajectory_point_frame >= base_footprint.outer()[0].x()) {
      trajectory_footprints.emplace_back();
    } else {
      auto base_footprint_beyond_current_ego_front = base_footprint;
      for (auto & p : base_footprint_beyond_current_ego_front.outer()) {
        p.x() = std::max(ego_front_x_in_trajectory_point_frame, p.x());
      }
      trajectory_footprints.push_back(
        project_to_trajectory_point(base_footprint_beyond_current_ego_front, trajectory_point));
    }
  }
  for (auto i = last_cut_index + 1; i < ego_data.trajectory_points.size(); ++i) {
    const auto & trajectory_point = ego_data.trajectory_points[i];
    trajectory_footprints.push_back(project_to_trajectory_point(base_footprint, trajectory_point));
  }
  return trajectory_footprints;
}

lanelet::BasicPolygon2d calculate_current_ego_footprint(
  const EgoData & ego_data, const PlannerParam & params, const bool ignore_offset)
{
  const auto base_footprint = make_base_footprint(params, ignore_offset);
  const auto angle = tf2::getYaw(ego_data.pose.orientation);
  const auto rotated_footprint = autoware_utils::rotate_polygon(base_footprint, angle);
  lanelet::BasicPolygon2d footprint;
  for (const auto & p : rotated_footprint.outer())
    footprint.emplace_back(p.x() + ego_data.pose.position.x, p.y() + ego_data.pose.position.y);
  return footprint;
}
}  // namespace autoware::motion_velocity_planner::out_of_lane
