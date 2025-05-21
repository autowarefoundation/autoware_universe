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

#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <optional>
#include <vector>

namespace autoware::behavior_velocity_planner
{
/// @brief create a search area ahead of a crosswalk by the given distance
/// @param [in] crosswalk_lanelet lanelet of the target crosswalk
/// @param [in] path_lanelets sequence of path lanelets
/// @param [in] first_path_point_on_crosswalk first path point on the crosswalk
/// @param [in] search_distance [m] distance ahead of the crosswalk to create the search area
/// @return the search area ahead of the crosswalk
lanelet::BasicPolygon2d create_search_area(
  const lanelet::ConstLanelet & crosswalk_lanelet, const lanelet::ConstLanelets & path_lanelets,
  const geometry_msgs::msg::Point & first_path_point_on_crosswalk, const double search_distance);
/// @brief calculate the furthest footprint point belonging to a parked object footprint before the
/// crosswalk
/// @param [in] ego_path ego path
/// @param [in] object_polygons polygons of the object footprints
/// @param [in] first_path_point_on_crosswalk first path point on the crosswalk
/// @return optional furthest parked object point along the path before the crosswalk
std::optional<geometry_msgs::msg::Point> calculate_furthest_parked_object_point(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & ego_path,
  const std::vector<autoware_utils_geometry::Polygon2d> & object_polygons,
  const geometry_msgs::msg::Point & first_path_point_on_crosswalk);
/// @brief determine if the path already plans to stop inside the search area
/// @param [in] path ego path
/// @param [in] ego_idx ego index in the path
/// @param [in] search_area search area
/// @return true if a stopped point after the ego index was found inside the search area
bool is_planning_to_stop_in_search_area(
  const std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> & path,
  const size_t ego_idx, const lanelet::BasicPolygon2d & search_area);

}  // namespace autoware::behavior_velocity_planner

#endif  // PARKED_VEHICLES_STOP_HPP_
