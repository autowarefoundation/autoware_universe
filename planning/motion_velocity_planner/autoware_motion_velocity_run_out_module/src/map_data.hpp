// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef MAP_DATA_HPP_
#define MAP_DATA_HPP_

#include "parameters.hpp"
#include "types.hpp"

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <boost/geometry/algorithms/envelope.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
template <class T>
universe_utils::Segment2d convert(const lanelet::Segment<T> & segment)
{
  return {{segment.first.x(), segment.first.y()}, {segment.second.x(), segment.second.y()}};
}

/// @brief prepare the bounding box where map data will be extracted
lanelet::BoundingBox2d prepare_relevent_bounding_box(
  const TrajectoryCornerFootprint & ego_footprint,
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects);

/// @brief add polygons used to ignore collisions and objects or cut predicted paths
void add_ignore_and_cut_polygons(
  FilteringDataPerLabel & data_per_label, const std::vector<lanelet::Lanelet> & lanelets,
  const std::vector<uint8_t> & labels, const std::vector<ObjectParameters> & params_per_label);

/// @brief add segments used to cut the predicted paths in the given polygons
void add_cut_segments(
  FilteringDataPerLabel & data_per_label, const std::vector<lanelet::Polygon3d> & polygons,
  const std::vector<uint8_t> & labels, const std::vector<ObjectParameters> & params_per_label);

/// @brief add segments used to cut the predicted paths in the given linestrings
void add_cut_segments(
  FilteringDataPerLabel & data_per_label, const std::vector<lanelet::LineString3d> & linestrings,
  const std::vector<uint8_t> & labels, const std::vector<ObjectParameters> & params_per_label);

/// @brief calculate map filtering data for each object classification label
FilteringDataPerLabel calculate_filtering_data(
  const lanelet::LaneletMapPtr & map_ptr, const TrajectoryCornerFootprint & ego_footprint,
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects, const Parameters & parameters);
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // MAP_DATA_HPP_
