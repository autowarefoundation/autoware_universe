// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__MAP_UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__MAP_UTILS_HPP_

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/types.hpp"

#include <lanelet2_core/LaneletMap.h>

#include <vector>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
/// @brief Extract uncrossable segments from the lanelet map that are in range of ego
/// @param[in] lanelet_map lanelet map
/// @param[in] ego_point point of the current ego position
/// @param[in] params parameters with linestring types that cannot be crossed and maximum range
/// @return the uncrossable segments stored in a rtree
SegmentRtree extract_uncrossable_segments(
  const lanelet::LaneletMap & lanelet_map, const Point & ego_point,
  const DrivableAreaExpansionParameters & params);

/// @brief Determine if the given linestring has one of the given types
/// @param[in] ls linestring to check
/// @param[in] types type strings to check
/// @return true if the linestring has one of the given types
bool has_types(
  const lanelet::ConstLineString3d & ls,
  const std::vector<DrivableAreaExpansionParameters::LinestringType> & types);

/// @brief Determine if a polygon is separated from a point by an uncrossable linestring
/// @details Guard rails are used as the uncrossable linestrings.
/// The polygon is separated only if all of its points are separated, so that an object is not
/// ignored while a part of it can still reach the given point.
/// @param[in] lanelet_map lanelet map
/// @param[in] from point to check against (e.g. a point on the ego path)
/// @param[in] polygon polygon to check (e.g. the footprint of an object)
/// @return true if every point of the polygon is separated from the given point
bool is_separated_by_uncrossable_linestring(
  const lanelet::LaneletMap & lanelet_map, const Point & from, const Polygon2d & polygon);
}  // namespace autoware::behavior_path_planner::drivable_area_expansion

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__MAP_UTILS_HPP_
