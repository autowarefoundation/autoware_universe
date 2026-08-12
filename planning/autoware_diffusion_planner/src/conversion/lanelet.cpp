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

#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/trajectory/interpolator/akima_spline.hpp"
#include "autoware/trajectory/interpolator/linear.hpp"
#include "autoware_utils_math/unit_conversion.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>

#include <geometry_msgs/msg/detail/point__struct.hpp>

#include <lanelet2_core/Forward.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>
namespace autoware::diffusion_planner
{

namespace
{
using autoware::experimental::trajectory::interpolator::AkimaSpline;
using autoware::experimental::trajectory::interpolator::Linear;

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

// Subdivides into multiple segments when step_m exceeds max_step_m so each segment stays within
// the resolution bound. First/last points are exact (not spline-evaluated).
std::vector<std::vector<LanePoint>> resample_line_string(
  const std::vector<LanePoint> & input, const size_t num_points, const double max_step_m)
{
  if (input.size() < 2 || num_points < 2) {
    return {input};
  }

  // Compute cumulative arc lengths along the input polyline
  std::vector<double> arc_lengths(input.size(), 0.0);
  for (size_t i = 1; i < input.size(); ++i) {
    arc_lengths[i] = arc_lengths[i - 1] + (input[i] - input[i - 1]).norm();
  }
  const double total_length = arc_lengths.back();

  constexpr double k_epsilon = 1e-6;
  if (total_length < k_epsilon) {
    return {std::vector<LanePoint>(num_points, input.front())};
  }

  // Determine the number of output segments needed to satisfy the resolution bound
  const double step_m = total_length / static_cast<double>(num_points - 1);
  const double safe_max_step_m = std::max(max_step_m, k_epsilon);
  const auto n_segments = static_cast<size_t>(std::max(1.0, std::ceil(step_m / safe_max_step_m)));

  // Extract per-axis value arrays for interpolator construction
  std::vector<double> x_vals(input.size());
  std::vector<double> y_vals(input.size());
  std::vector<double> z_vals(input.size());
  for (size_t i = 0; i < input.size(); ++i) {
    x_vals[i] = input[i].x();
    y_vals[i] = input[i].y();
    z_vals[i] = input[i].z();
  }

  // Build interpolators. AkimaSpline requires >= 5 input points; use Linear otherwise.
  std::function<LanePoint(double)> compute_point;

  if (input.size() >= 5) {
    AkimaSpline x_spline;
    AkimaSpline y_spline;
    AkimaSpline z_spline;
    const auto rx = x_spline.build(arc_lengths, x_vals);
    const auto ry = y_spline.build(arc_lengths, y_vals);
    const auto rz = z_spline.build(arc_lengths, z_vals);
    if (!rx || !ry || !rz) {
      std::cerr << "resample_line_string: failed to build AkimaSpline, returning single segment\n";
      return {interpolate_points(input, num_points)};
    }
    compute_point = [x_spline, y_spline, z_spline](const double s) {
      return LanePoint{x_spline.compute(s), y_spline.compute(s), z_spline.compute(s)};
    };
  } else {
    Linear x_spline;
    Linear y_spline;
    Linear z_spline;
    const auto rx = x_spline.build(arc_lengths, x_vals);
    const auto ry = y_spline.build(arc_lengths, y_vals);
    const auto rz = z_spline.build(arc_lengths, z_vals);
    if (!rx || !ry || !rz) {
      std::cerr
        << "resample_line_string: failed to build Linear interpolator, returning single segment\n";
      return {interpolate_points(input, num_points)};
    }
    compute_point = [x_spline, y_spline, z_spline](const double s) {
      return LanePoint{x_spline.compute(s), y_spline.compute(s), z_spline.compute(s)};
    };
  }

  // Sample each segment with exactly num_points points
  const double segment_length = total_length / static_cast<double>(n_segments);
  std::vector<std::vector<LanePoint>> result;
  result.reserve(n_segments);

  for (size_t i = 0; i < n_segments; ++i) {
    const double s_start = static_cast<double>(i) * segment_length;
    const double inner_step = segment_length / static_cast<double>(num_points - 1);

    std::vector<LanePoint> lane_points;
    lane_points.reserve(num_points);

    for (size_t j = 0; j < num_points; ++j) {
      if (i == 0 && j == 0) {
        lane_points.push_back(input.front());
        continue;
      }
      if (i == n_segments - 1 && j == num_points - 1) {
        lane_points.push_back(input.back());
        continue;
      }
      const double s = std::clamp(s_start + static_cast<double>(j) * inner_step, 0.0, total_length);
      lane_points.push_back(compute_point(s));
    }
    result.push_back(std::move(lane_points));
  }

  return result;
}

template <typename T>
std::vector<LanePoint> convert_to_polyline(const T & line_string) noexcept
{
  std::vector<LanePoint> output;
  output.reserve(line_string.size());
  for (const auto & point : line_string) {
    output.emplace_back(point.x(), point.y(), point.z());
  }
  return output;
}
}  // namespace

LaneletMap convert_to_internal_lanelet_map(
  const lanelet::LaneletMapConstPtr lanelet_map_ptr, const double line_string_max_step_m)
{
  LaneletMap lanelet_map;
  lanelet_map.lane_segments.reserve(lanelet_map_ptr->laneletLayer.size());
  lanelet_map.polygons.reserve(lanelet_map_ptr->polygonLayer.size());
  lanelet_map.line_strings.reserve(lanelet_map_ptr->lineStringLayer.size());

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
      interpolate_points(convert_to_polyline(lanelet.centerline3d()), POINTS_PER_SEGMENT));
    const Polyline left_boundary(
      interpolate_points(convert_to_polyline(lanelet.leftBound3d()), POINTS_PER_SEGMENT));
    const Polyline right_boundary(
      interpolate_points(convert_to_polyline(lanelet.rightBound3d()), POINTS_PER_SEGMENT));

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

    lanelet_map.lane_segments.emplace_back(
      lanelet.id(), centerline, left_boundary, right_boundary, mean_point, left_line_type,
      right_line_type, speed_limit_mps, turn_direction, traffic_light_id);
  }

  // parse polygon layers
  for (const auto & polygon : lanelet_map_ptr->polygonLayer) {
    const std::string polygon_type = polygon.attributeOr("type", "");
    const auto it = POLYGON_TYPE_MAP.find(polygon_type);
    if (it == POLYGON_TYPE_MAP.end()) {
      continue;
    }
    const std::vector<LanePoint> points(
      interpolate_points(convert_to_polyline(polygon.basicLineString()), POINTS_PER_POLYGON));
    lanelet_map.polygons.push_back(Polygon{points, it->second});
  }

  // parse line string layers
  for (const auto & line_string : lanelet_map_ptr->lineStringLayer) {
    const std::string line_string_type = line_string.attributeOr("type", "");
    const auto it = LINE_STRING_TYPE_MAP.find(line_string_type);
    if (it == LINE_STRING_TYPE_MAP.end()) {
      continue;
    }
    const auto segments = resample_line_string(
      convert_to_polyline(line_string), POINTS_PER_LINE_STRING, line_string_max_step_m);
    for (const auto & points : segments) {
      lanelet_map.line_strings.push_back(LineString{points, it->second});
    }
  }

  return lanelet_map;
}
}  // namespace autoware::diffusion_planner
