// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__GEOMETRY_UTILS_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__GEOMETRY_UTILS_HPP_

#include <autoware_lane_event_classifier/types.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <iterator>
#include <string>
#include <vector>

namespace autoware::lane_event_classifier
{

/**
 * @brief Returns the vehicle footprint polygon in the map frame, or a single centre point when
 * vehicle info is unavailable.
 * @param input Per-cycle input providing the ego pose and optional vehicle info.
 */
inline std::vector<lanelet::BasicPoint2d> compute_footprint(const LaneEventInput & input)
{
  const auto & pose = input.odometry_ptr->pose.pose;

  if (!input.vehicle_info_ptr) {
    return {{pose.position.x, pose.position.y}};
  }

  const auto map_footprint = autoware_utils_geometry::transform_vector(
    input.vehicle_info_ptr->createFootprint(), autoware_utils_geometry::pose2transform(pose));

  std::vector<lanelet::BasicPoint2d> map_footprint_2d;
  map_footprint_2d.reserve(map_footprint.size());
  std::transform(
    map_footprint.cbegin(), map_footprint.cend(), std::back_inserter(map_footprint_2d),
    [](const auto & footprint_point) {
      return lanelet::BasicPoint2d{footprint_point.x(), footprint_point.y()};
    });
  return map_footprint_2d;
}

/**
 * @brief Returns true only if every footprint corner lies inside the lanelet.
 * @param lane Lanelet to test against.
 * @param footprint Footprint corners in the map frame.
 */
inline bool is_footprint_fully_inside_lane(
  const lanelet::ConstLanelet & lane, const std::vector<lanelet::BasicPoint2d> & footprint)
{
  return std::all_of(
    footprint.cbegin(), footprint.cend(), [&lane](const lanelet::BasicPoint2d & footprint_corner) {
      return lanelet::geometry::inside(lane, footprint_corner);
    });
}

/**
 * @brief Returns true if the boundary linestring is virtual (attribute type == "virtual").
 * @param bound Boundary linestring to test.
 */
inline bool is_virtual_linestring(const lanelet::ConstLineString3d & bound)
{
  // std::string default: a const char* default would make Attribute::as<> compare by pointer.
  return bound.attributeOr(lanelet::AttributeName::Type, std::string{}) ==
         lanelet::AttributeValueString::Virtual;
}

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__GEOMETRY_UTILS_HPP_
