// Copyright 2025 Tier IV, Inc.
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

#ifndef PARKED_VEHICLES_STOP_HPP_
#define PARKED_VEHICLES_STOP_HPP_

#include "scene_crosswalk.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <vector>

namespace autoware::behavior_velocity_planner
{

inline lanelet::BasicPolygon2d create_search_area(
  const lanelet::ConstLanelet & crosswalk_lanelet,
  const lanelet::BasicPoint2d & first_path_point_on_crosswalk, const double search_distance)
{
  const auto is_ego_coming_from_left_side_of_crosswalk =
    lanelet::geometry::distance2d(
      crosswalk_lanelet.leftBound2d().basicLineString(), first_path_point_on_crosswalk) <
    lanelet::geometry::distance2d(
      crosswalk_lanelet.rightBound2d().basicLineString(), first_path_point_on_crosswalk);
  const auto left_dist = is_ego_coming_from_left_side_of_crosswalk ? search_distance : 0.0;
  const auto right_dist = is_ego_coming_from_left_side_of_crosswalk ? 0.0 : search_distance;
  const auto nearest_crosswalk_bound = is_ego_coming_from_left_side_of_crosswalk
                                         ? crosswalk_lanelet.leftBound2d().basicLineString()
                                         : crosswalk_lanelet.rightBound2d().basicLineString();
  boost::geometry::strategy::buffer::distance_asymmetric<double> distance_strategy{
    left_dist, right_dist};
  boost::geometry::strategy::buffer::side_straight side_strategy;
  boost::geometry::strategy::buffer::join_round join_strategy;
  boost::geometry::strategy::buffer::end_flat end_strategy;
  boost::geometry::strategy::buffer::point_square point_strategy;
  lanelet::BasicPolygons2d search_area;
  boost::geometry::buffer(
    nearest_crosswalk_bound, search_area, distance_strategy, side_strategy, join_strategy,
    end_strategy, point_strategy);
  if (!search_area.empty()) {
    return search_area.front();
  }
  return {};
}

std::optional<geometry_msgs::msg::Point> calculate_furthest_parked_object_point(
  const std::vector<PathPointWithLaneId> & ego_path,
  const std::vector<autoware_utils_geometry::Polygon2d> & object_polygons,
  const geometry_msgs::msg::Point & first_path_point_on_crosswalk)
{
  double furthest_parked_object_arc_length = 0.0;
  const auto max_stop_arc_length =
    motion_utils::calcSignedArcLength(ego_path, 0UL, first_path_point_on_crosswalk);
  geometry_msgs::msg::Point furthest_parked_object_point;
  for (const auto & object_polygon : object_polygons) {
    for (const auto & p : object_polygon.outer()) {
      const auto pt = geometry_msgs::msg::Point().set__x(p.x()).set__y(p.y());
      const auto arc_length = motion_utils::calcSignedArcLength(ego_path, 0UL, pt);
      if (arc_length > furthest_parked_object_arc_length) {
        furthest_parked_object_arc_length = arc_length;
        furthest_parked_object_point = pt;
      }
    }
  }
  if (furthest_parked_object_arc_length == 0.0) {
    return std::nullopt;
  }
  if (furthest_parked_object_arc_length > max_stop_arc_length) {
    furthest_parked_object_arc_length = max_stop_arc_length;
    furthest_parked_object_point = first_path_point_on_crosswalk;
  }
  return furthest_parked_object_point;
}

}  // namespace autoware::behavior_velocity_planner

#endif  // PARKED_VEHICLES_STOP_HPP_
