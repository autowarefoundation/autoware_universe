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
#include <lanelet2_core/primitives/BoundingBox.h>

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

/// @brief Extract the segments of the uncrossable linestrings overlapping the search box
/// @details Only the segments overlapping the box are kept, so that a linestring much longer than
/// the box does not inflate the resulting rtree. The segments are meant to be extracted once per
/// planning cycle and then queried for each object.
/// If @p types is empty, no segment is extracted and the returned rtree is empty.
/// @param[in] lanelet_map lanelet map
/// @param[in] search_box box covering the points to check (e.g. the ego path and the objects)
/// @param[in] types linestring types representing a physical boundary that objects cannot cross
/// @return the uncrossable segments stored in a rtree
SegmentRtree extract_uncrossable_segments(
  const lanelet::LaneletMap & lanelet_map, const lanelet::BoundingBox2d & search_box,
  const std::vector<DrivableAreaExpansionParameters::LinestringType> & types);

/// @brief Determine if an object cannot reach a point without crossing an uncrossable segment
/// @details The object is separated only if all of the following hold.
/// - Every point of its footprint is separated from @p from, so that an object is not ignored
///   while a part of it can still reach the given point.
/// - Its footprint does not overlap the boundary, so that an object standing on the boundary, and
///   thus possibly already on the other side of it, is not ignored.
/// - Its most likely predicted path never reaches a pose with a clear line of sight to @p from.
///   The path is cut where it crosses the boundary, assuming the object stops there. A path going
///   around the boundary or through one of its gaps is never cut and thus keeps the object.
/// Note that the check is done against a single point, so an object that can only reach the given
/// point through a far away gap of the boundary may still be reported as separated.
/// If @p uncrossable_segments is empty, the check is disabled and false is always returned.
/// @param[in] uncrossable_segments uncrossable segments prepared once per planning cycle
/// @param[in] from point to check against (e.g. a point on the ego path)
/// @param[in] object object to check
/// @return true if the object cannot reach the given point without crossing the boundary
bool is_separated_by_uncrossable_segments(
  const SegmentRtree & uncrossable_segments, const Point & from, const PredictedObject & object);
}  // namespace autoware::behavior_path_planner::drivable_area_expansion

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__MAP_UTILS_HPP_
