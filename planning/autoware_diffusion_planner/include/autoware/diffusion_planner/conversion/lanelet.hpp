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

#ifndef AUTOWARE__DIFFUSION_PLANNER__CONVERSION__LANELET_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__CONVERSION__LANELET_HPP_

#include "autoware/diffusion_planner/polyline.hpp"

#include <lanelet2_core/LaneletMap.h>

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{

enum LineType {
  LINE_TYPE_CROSSWALK = 0,
  LINE_TYPE_CURBSTONE = 1,
  LINE_TYPE_GUARD_RAIL = 2,
  LINE_TYPE_LINE_THICK = 3,
  LINE_TYPE_LINE_THIN = 4,
  LINE_TYPE_PEDESTRIAN_MARKING = 5,
  LINE_TYPE_ROAD_BORDER = 6,
  LINE_TYPE_ROAD_SHOULDER = 7,
  LINE_TYPE_VIRTUAL = 8,
  LINE_TYPE_ZEBRA_MARKING = 9,
  LINE_TYPE_NUM = 10
};

const std::map<std::string, LineType> LINE_TYPE_MAP = {
  {"crosswalk", LINE_TYPE_CROSSWALK},     {"curbstone", LINE_TYPE_CURBSTONE},
  {"guard_rail", LINE_TYPE_GUARD_RAIL},   {"line_thick", LINE_TYPE_LINE_THICK},
  {"line_thin", LINE_TYPE_LINE_THIN},     {"pedestrian_marking", LINE_TYPE_PEDESTRIAN_MARKING},
  {"road_border", LINE_TYPE_ROAD_BORDER}, {"road_shoulder", LINE_TYPE_ROAD_SHOULDER},
  {"virtual", LINE_TYPE_VIRTUAL},         {"zebra_marking", LINE_TYPE_ZEBRA_MARKING}};

struct LaneSegment
{
  int64_t id;
  Polyline polyline;
  bool is_intersection{false};
  std::vector<BoundarySegment> left_boundaries;
  std::vector<BoundarySegment> right_boundaries;
  LineType left_line_type;
  LineType right_line_type;
  std::optional<float> speed_limit_mps{std::nullopt};
  int64_t turn_direction;
  int64_t traffic_light_id;

  static const int64_t TURN_DIRECTION_NONE = -1;
  static const int64_t TURN_DIRECTION_STRAIGHT = 0;
  static const int64_t TURN_DIRECTION_LEFT = 1;
  static const int64_t TURN_DIRECTION_RIGHT = 2;

  static const int64_t TRAFFIC_LIGHT_ID_NONE = -1;

  LaneSegment(
    int64_t id, Polyline polyline, bool is_intersection,
    const std::vector<BoundarySegment> & left_boundaries,
    const std::vector<BoundarySegment> & right_boundaries, LineType left_line_type,
    LineType right_line_type, std::optional<float> speed_limit_mps, int64_t turn_direction,
    int64_t traffic_light_id)
  : id(id),
    polyline(std::move(polyline)),
    is_intersection(is_intersection),
    left_boundaries(left_boundaries),
    right_boundaries(right_boundaries),
    left_line_type(left_line_type),
    right_line_type(right_line_type),
    speed_limit_mps(speed_limit_mps),
    turn_direction(turn_direction),
    traffic_light_id(traffic_light_id)
  {
  }
};

/**
 * @brief Convert a lanelet map to line segment data
 * @param lanelet_map_ptr Pointer of loaded lanelet map.
 * @param num_lane_points Number of points per lane segment.
 * @return std::vector<LaneSegment>
 */
[[nodiscard]] std::vector<LaneSegment> convert_to_lane_segments(
  const lanelet::LaneletMapConstPtr lanelet_map_ptr, const int64_t num_lane_points);

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__CONVERSION__LANELET_HPP_
