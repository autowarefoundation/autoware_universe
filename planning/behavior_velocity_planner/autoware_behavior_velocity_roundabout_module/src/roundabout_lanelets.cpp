// Copyright 2025 TIER IV, Inc.
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

#include "roundabout_lanelets.hpp"

#include <autoware/behavior_velocity_intersection_module/util.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <string>

namespace autoware::behavior_velocity_planner
{

void RoundaboutLanelets::update(
  const InterpolatedPathInfo & interpolated_path_info,
  const autoware_utils::LinearRing2d & footprint, const double vehicle_length,
  [[maybe_unused]] lanelet::routing::RoutingGraphPtr routing_graph_ptr)
{
  if (!first_attention_area_) {
    const auto first = util::getFirstPointInsidePolygonsByFootprint(
      attention_non_preceding_area_, interpolated_path_info, footprint, vehicle_length);
    if (first) {
      first_attention_lane_ = attention_non_preceding_.at(first.value().second);
      first_attention_area_ = attention_non_preceding_area_.at(first.value().second);
    }
  }
}
}  // namespace autoware::behavior_velocity_planner
