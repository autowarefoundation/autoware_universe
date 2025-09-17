// Copyright 2024 TIER IV, Inc.
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

#include "autoware/diffusion_planner/conversion/lanelet.hpp"

#include "autoware_utils_math/unit_conversion.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>

#include <geometry_msgs/msg/detail/point__struct.hpp>

#include <lanelet2_core/Forward.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>
namespace autoware::diffusion_planner
{

namespace
{
std::vector<LanePoint> interpolate_points(const std::vector<LanePoint> & input, size_t num_points)
{
  if (input.size() < 2 || num_points < 2) {
    std::cerr << "Need at least 2 input points\n";
    return input;
  }
  // Step 1: Compute cumulative distances
  std::vector<double> arc_lengths(input.size(), 0.0);
  for (size_t i = 1; i < input.size(); ++i) {
    arc_lengths[i] = arc_lengths[i - 1] + (input[i] - input[i - 1]).norm();
  }
  const double total_length = arc_lengths.back();

  // Step 2: Generate target arc lengths
  std::vector<LanePoint> result;
  result.reserve(num_points);

  // Always include the first point
  result.push_back(input.front());

  // Generate interior points
  if (num_points == 2) {
    // Always include the last point
    result.push_back(input.back());
    return result;
  }

  const double step = total_length / static_cast<double>(num_points - 1);
  size_t seg_idx = 0;

  for (size_t i = 1; i < num_points - 1; ++i) {
    const double target = static_cast<double>(i) * step;

    // Find the correct segment containing the target arc length
    while (seg_idx + 1 < arc_lengths.size() && arc_lengths[seg_idx + 1] < target) {
      ++seg_idx;
    }

    // Ensure we don't go past the last segment
    if (seg_idx >= arc_lengths.size() - 1) {
      seg_idx = arc_lengths.size() - 2;
    }

    // Interpolate between input[seg_idx] and input[seg_idx + 1]
    const double seg_start = arc_lengths[seg_idx];
    const double seg_end = arc_lengths[seg_idx + 1];
    const double seg_length = seg_end - seg_start;

    // Calculate interpolation parameter, handling zero-length segments
    constexpr double epsilon = 1e-6;
    const double safe_seg_length = std::max(seg_length, epsilon);
    const double t = std::clamp((target - seg_start) / safe_seg_length, 0.0, 1.0);
    const LanePoint new_point = input[seg_idx] + t * (input[seg_idx + 1] - input[seg_idx]);
    result.push_back(new_point);
  }
  // Always include the last point
  result.push_back(input.back());

  return result;
}

std::vector<LanePoint> convert_to_polyline(const lanelet::ConstLineString3d & line_string) noexcept
{
  std::vector<LanePoint> output;
  output.reserve(line_string.size());
  for (const lanelet::Point3d::ConstType & point : line_string) {
    output.emplace_back(point.x(), point.y(), point.z());
  }
  return output;
}
}  // namespace

std::vector<LaneSegment> convert_to_lane_segments(
  const lanelet::LaneletMapConstPtr lanelet_map_ptr, const int64_t num_lane_points)
{
  std::vector<LaneSegment> lane_segments;
  lane_segments.reserve(lanelet_map_ptr->laneletLayer.size());
  // parse lanelet layers
  for (const auto & lanelet : lanelet_map_ptr->laneletLayer) {
    if (!lanelet.hasAttribute("subtype")) {
      continue;
    }
    const auto lanelet_subtype = lanelet.attribute("subtype").as<std::string>();
    if (!lanelet_subtype || ACCEPTABLE_LANE_SUBTYPES.count(lanelet_subtype.value()) == 0) {
      continue;
    }
    const Polyline centerline(
      interpolate_points(convert_to_polyline(lanelet.centerline3d()), num_lane_points));
    const Polyline left_boundary(
      interpolate_points(convert_to_polyline(lanelet.leftBound3d()), num_lane_points));
    const Polyline right_boundary(
      interpolate_points(convert_to_polyline(lanelet.rightBound3d()), num_lane_points));

    LanePoint mean_point(0.0, 0.0, 0.0);
    for (const LanePoint & p : centerline) {
      mean_point += p;
    }
    mean_point /= static_cast<double>(centerline.size());

    const std::string left_line_type_str = lanelet.leftBound().attributeOr("type", "");
    const std::string right_line_type_str = lanelet.rightBound().attributeOr("type", "");
    const LineType left_line_type =
      (LINE_TYPE_MAP.count(left_line_type_str) ? LINE_TYPE_MAP.at(left_line_type_str)
                                               : LINE_TYPE_VIRTUAL);
    const LineType right_line_type =
      (LINE_TYPE_MAP.count(right_line_type_str) ? LINE_TYPE_MAP.at(right_line_type_str)
                                                : LINE_TYPE_VIRTUAL);

    const lanelet::AttributeMap & attrs = lanelet.attributes();
    const std::optional<float> speed_limit_mps =
      attrs.find("speed_limit") != attrs.end()
        ? std::make_optional(
            autoware_utils_math::kmph2mps(std::stof(attrs.at("speed_limit").value())))
        : std::nullopt;

    int64_t turn_direction = LaneSegment::TURN_DIRECTION_NONE;
    const std::map<std::string, int64_t> turn_direction_map = {
      {"straight", LaneSegment::TURN_DIRECTION_STRAIGHT},
      {"left", LaneSegment::TURN_DIRECTION_LEFT},
      {"right", LaneSegment::TURN_DIRECTION_RIGHT}};
    if (attrs.find("turn_direction") != attrs.end()) {
      const std::string turn_direction_str = attrs.at("turn_direction").value();
      const auto itr = turn_direction_map.find(turn_direction_str);
      if (itr != turn_direction_map.end()) {
        turn_direction = itr->second;
      }
    }

    const std::vector<lanelet::format_v2::TrafficLightConstPtr> traffic_light_list =
      lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();

    // According to the definition, the number of elements in the traffic_light_list should be
    // either 0 or 1; however, this is not always the case with older map data. Therefore, if there
    // are multiple elements, we only use the first element.
    const int64_t traffic_light_id =
      (traffic_light_list.empty() ? LaneSegment::TRAFFIC_LIGHT_ID_NONE
                                  : traffic_light_list.front()->id());

    lane_segments.emplace_back(
      lanelet.id(), centerline, left_boundary, right_boundary, mean_point, left_line_type,
      right_line_type, speed_limit_mps, turn_direction, traffic_light_id);
  }
  return lane_segments;
}
}  // namespace autoware::diffusion_planner
