// Copyright 2024 Tier IV, Inc.
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

#include "autoware/behavior_velocity_intersection_module/intersection_lanelets.hpp"

#include "autoware/behavior_velocity_intersection_module/util.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>

#include <string>

namespace autoware::behavior_velocity_planner
{

void IntersectionLanelets::update(
  const bool is_prioritized, const InterpolatedPathInfo & interpolated_path_info,
  const autoware_utils::LinearRing2d & footprint, const double vehicle_length)
{
  is_prioritized_ = is_prioritized;
  // find the first conflicting/detection area polygon intersecting the path
  if (!first_conflicting_area_) {
    auto first = util::getFirstPointInsidePolygonsByFootprint(
      conflicting_area_, interpolated_path_info, footprint, vehicle_length);
    if (first) {
      first_conflicting_lane_ = conflicting_.at(first.value().second);
      first_conflicting_area_ = conflicting_area_.at(first.value().second);
    }
  }
  if (!first_attention_area_) {
    const auto first = util::getFirstPointInsidePolygonsByFootprint(
      attention_non_preceding_area_, interpolated_path_info, footprint, vehicle_length);
    if (first) {
      first_attention_lane_ = attention_non_preceding_.at(first.value().second);
      first_attention_area_ = attention_non_preceding_area_.at(first.value().second);
    }
  }
}

void IntersectionLanelets::calcBoundingBoxes()
{
  const auto bounding_boxes = [](const lanelet::ConstLanelets & lanelets) {
    std::vector<lanelet::BoundingBox2d> boxes;
    boxes.reserve(lanelets.size());
    for (const auto & lanelet : lanelets) {
      boxes.push_back(lanelet::geometry::boundingBox2d(lanelet));
    }
    return boxes;
  };
  attention_bounding_boxes_ = bounding_boxes(attention_);
  attention_non_preceding_bounding_boxes_ = bounding_boxes(attention_non_preceding_);
  adjacent_bounding_boxes_ = bounding_boxes(adjacent_);
}
}  // namespace autoware::behavior_velocity_planner
