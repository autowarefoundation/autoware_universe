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
  std::vector<lanelet::BasicPolygon2d> trajectory_footprints;
  trajectory_footprints.reserve(ego_data.trajectory_points.size());
  const auto base_footprint = make_base_footprint(params);
  // create a polygon to cut the trajectory footprints behind the current front of the ego vehicle
  constexpr auto cut_multiplier = 10.0;
  const auto cut_polygon = autoware_utils_geometry::to_footprint(
    ego_data.pose, params.front_offset, -params.rear_offset * cut_multiplier,
    params.left_offset * cut_multiplier);
  auto arc_length = 0.0;
  for (auto i = 0UL; i < ego_data.trajectory_points.size(); ++i) {
    const auto & trajectory_point = ego_data.trajectory_points[i];
    const auto trajectory_footprint = project_to_trajectory_point(base_footprint, trajectory_point);
    // we only apply the cut to the beginning of the trajectory
    if (arc_length > params.front_offset) {
      trajectory_footprints.push_back(trajectory_footprint);
      continue;
    }
    if (i + 1 < ego_data.trajectory_points.size()) {
      arc_length +=
        autoware_utils::calc_distance2d(trajectory_point, ego_data.trajectory_points[i + 1]);
    }
    lanelet::BasicPolygons2d cut_result;
    boost::geometry::difference(trajectory_footprint, cut_polygon, cut_result);
    trajectory_footprints.push_back(
      cut_result.empty() ? lanelet::BasicPolygon2d() : cut_result.front());
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
