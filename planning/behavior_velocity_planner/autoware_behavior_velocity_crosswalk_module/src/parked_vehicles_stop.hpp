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

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/strategies/cartesian/buffer_side_straight.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/utility/Utilities.h>
#include <lanelet2_routing/Forward.h>

#include <algorithm>
#include <vector>

namespace autoware::behavior_velocity_planner
{

/// @brief
inline lanelet::BasicPolygon2d create_search_area(
  const lanelet::ConstLanelet & crosswalk_lanelet, const lanelet::ConstLanelets & path_lanelets,
  const geometry_msgs::msg::Point & first_path_point_on_crosswalk, const double search_distance)
{
  lanelet::BasicLineString2d path_left_bound;
  lanelet::BasicLineString2d path_right_bound;
  // create left and right bounds without duplicated points
  for (auto & ll : path_lanelets) {
    auto left_bound = ll.leftBound2d().basicLineString();
    auto left_bound_begin =
      path_left_bound.empty() ? left_bound.begin() : std::next(left_bound.begin());
    path_left_bound.insert(path_left_bound.end(), left_bound_begin, left_bound.end());
    auto right_bound = ll.rightBound2d().basicLineString();
    auto right_bound_begin =
      path_right_bound.empty() ? right_bound.begin() : std::next(right_bound.begin());
    path_right_bound.insert(path_right_bound.end(), right_bound_begin, right_bound.end());
  }
  const auto end_search_point =
    lanelet::BasicPoint2d(first_path_point_on_crosswalk.x, first_path_point_on_crosswalk.y);
  const auto is_ego_coming_from_left_side_of_crosswalk =
    lanelet::geometry::distance2d(
      crosswalk_lanelet.leftBound2d().basicLineString(), end_search_point) <
    lanelet::geometry::distance2d(
      crosswalk_lanelet.rightBound2d().basicLineString(), end_search_point);
  const auto nearest_crosswalk_bound = is_ego_coming_from_left_side_of_crosswalk
                                         ? crosswalk_lanelet.leftBound2d().basicLineString()
                                         : crosswalk_lanelet.rightBound2d().basicLineString();
  const auto dist_front =
    lanelet::geometry::toArcCoordinates(path_left_bound, nearest_crosswalk_bound.front()).distance;
  const auto dist_back =
    lanelet::geometry::toArcCoordinates(path_left_bound, nearest_crosswalk_bound.back()).distance;
  const auto left_end_point =
    dist_front > dist_back ? nearest_crosswalk_bound.front() : nearest_crosswalk_bound.back();
  const auto right_end_point =
    dist_front <= dist_back ? nearest_crosswalk_bound.front() : nearest_crosswalk_bound.back();
  const auto left_dist =
    lanelet::geometry::toArcCoordinates(path_left_bound, left_end_point).distance;
  const auto right_dist =
    lanelet::geometry::toArcCoordinates(path_right_bound, right_end_point).distance;

  const auto expanded_left_bound = lanelet::geometry::offsetNoThrow(path_left_bound, left_dist);
  const auto expanded_right_bound = lanelet::geometry::offsetNoThrow(path_right_bound, right_dist);

  const auto end_search_left_arc_coordinates =
    lanelet::geometry::toArcCoordinates(expanded_left_bound, left_end_point);
  const auto start_search_left_arc_length =
    std::max(end_search_left_arc_coordinates.length - search_distance, 0.0);
  const auto end_search_right_arc_coordinates =
    lanelet::geometry::toArcCoordinates(expanded_right_bound, right_end_point);
  const auto start_search_right_arc_length =
    std::max(end_search_right_arc_coordinates.length - search_distance, 0.0);

  lanelet::BasicLineString2d search_left_bound;
  lanelet::BasicLineString2d search_right_bound;
  auto arc_length = 0.0;
  for (auto i = 0UL; i + 1 < expanded_left_bound.size(); ++i) {
    const auto & p1 = expanded_left_bound[i];
    const auto & p2 = expanded_left_bound[i + 1];
    arc_length += lanelet::geometry::distance2d(p1, p2);
    if (arc_length > end_search_left_arc_coordinates.length) {
      break;
    }
    if (arc_length >= start_search_left_arc_length) {
      search_left_bound.emplace_back(p2);
    }
  }
  const auto first_left_point =
    lanelet::geometry::fromArcCoordinates(expanded_left_bound, {start_search_left_arc_length, 0.0});
  if (
    !search_left_bound.empty() &&
    lanelet::geometry::distance2d(search_left_bound.front(), first_left_point) > 1e-3) {
    search_left_bound.insert(search_left_bound.begin(), first_left_point);
  }
  if (
    !search_left_bound.empty() &&
    lanelet::geometry::distance2d(search_left_bound.back(), left_end_point) > 1e-3) {
    search_left_bound.push_back(left_end_point);
  }
  // search_left_bound.emplace_back(left_end_point);
  arc_length = 0.0;
  for (auto i = 0UL; i + 1 < expanded_right_bound.size(); ++i) {
    const auto & p1 = expanded_right_bound[i];
    const auto & p2 = expanded_right_bound[i + 1];
    arc_length += lanelet::geometry::distance2d(p1, p2);
    if (arc_length > end_search_right_arc_coordinates.length) {
      break;
    }
    if (arc_length >= start_search_right_arc_length) {
      search_right_bound.emplace_back(p2);
    }
  }
  const auto first_right_point =
    lanelet::geometry::fromArcCoordinates(expanded_left_bound, {start_search_left_arc_length, 0.0});
  if (
    !search_right_bound.empty() &&
    lanelet::geometry::distance2d(search_right_bound.front(), first_right_point) > 1e-3) {
    search_right_bound.insert(search_right_bound.begin(), first_right_point);
  }
  if (
    !search_right_bound.empty() &&
    lanelet::geometry::distance2d(search_right_bound.back(), right_end_point) > 1e-3) {
    search_right_bound.push_back(right_end_point);
  }
  // search_right_bound.emplace_back(right_end_point);

  lanelet::BasicPolygon2d search_area(search_left_bound);
  for (auto it = search_right_bound.rbegin(); it != search_right_bound.rend(); ++it) {
    search_area.emplace_back(*it);
  }
  // if(!search_area.empty()) {
  //   search_area.emplace_back(search_area.front());
  // }
  boost::geometry::correct(search_area);
  // search_area.clear();
  // search_area.insert(search_area.end(), expanded_left_bound.begin(), expanded_left_bound.end());
  // search_area.insert(search_area.end(), expanded_right_bound.rbegin(),
  // expanded_right_bound.rend());
  return search_area;
}

inline std::optional<geometry_msgs::msg::Point> calculate_furthest_parked_object_point(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & ego_path,
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
