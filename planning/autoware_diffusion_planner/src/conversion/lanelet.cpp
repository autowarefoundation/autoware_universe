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

#include "autoware/diffusion_planner/polyline.hpp"
#include "autoware_utils_math/unit_conversion.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/Forward.hpp>

#include <geometry_msgs/msg/detail/point__struct.hpp>

#include <lanelet2_core/Forward.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <optional>
#include <vector>
namespace autoware::diffusion_planner
{
// Compute Euclidean distance between two LanePoints
inline float euclidean_distance(const LanePoint & p1, const LanePoint & p2)
{
  float dx = p2.x() - p1.x();
  float dy = p2.y() - p1.y();
  float dz = p2.z() - p1.z();
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<LanePoint> interpolate_points(const std::vector<LanePoint> & input, size_t num_points)
{
  if (input.size() < 2 || num_points < 2) {
    std::cerr << "Need at least 2 input points\n";
    return input;
  }
  // Step 1: Compute cumulative distances
  std::vector<float> arc_lengths(input.size(), 0.0f);
  for (size_t i = 1; i < input.size(); ++i) {
    arc_lengths[i] = arc_lengths[i - 1] + input[i].distance(input[i - 1]);
  }
  float total_length = arc_lengths.back();

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

  float step = total_length / static_cast<float>(num_points - 1);
  size_t seg_idx = 0;

  for (size_t i = 1; i < num_points - 1; ++i) {
    float target = static_cast<float>(i) * step;

    // Find the correct segment containing the target arc length
    while (seg_idx + 1 < arc_lengths.size() && arc_lengths[seg_idx + 1] < target) {
      ++seg_idx;
    }

    // Ensure we don't go past the last segment
    if (seg_idx >= arc_lengths.size() - 1) {
      seg_idx = arc_lengths.size() - 2;
    }

    // Interpolate between input[seg_idx] and input[seg_idx + 1]
    float seg_start = arc_lengths[seg_idx];
    float seg_end = arc_lengths[seg_idx + 1];
    float seg_length = seg_end - seg_start;

    // Calculate interpolation parameter, handling zero-length segments
    float safe_seg_length = std::max(seg_length, 1e-6f);
    float t = (target - seg_start) / safe_seg_length;
    // Clamp t to [0, 1] to ensure we don't extrapolate
    t = std::max(0.0f, std::min(1.0f, t));
    result.push_back(input[seg_idx].lerp(input[seg_idx + 1], t));
  }
  // Always include the last point
  result.push_back(input.back());

  // Recalculate direction vectors based on actual interpolated positions
  // Helper lambda to update a point's direction vector
  auto update_point_direction =
    [](std::vector<LanePoint> & points, size_t point_idx, size_t from_idx, size_t to_idx) {
      float dx = points[to_idx].x() - points[from_idx].x();
      float dy = points[to_idx].y() - points[from_idx].y();
      float dz = points[to_idx].z() - points[from_idx].z();

      normalize_direction(dx, dy, dz);

      points[point_idx] = LanePoint(
        points[point_idx].x(), points[point_idx].y(), points[point_idx].z(), dx, dy, dz,
        points[point_idx].label());
    };

  if (result.size() > 1) {
    // Handle first point direction (points to next)
    update_point_direction(result, 0, 0, 1);

    // Handle middle points (point to next)
    for (size_t i = 1; i < result.size() - 1; ++i) {
      update_point_direction(result, i, i, i + 1);
    }

    // Handle last point direction (from previous)
    size_t last_idx = result.size() - 1;
    update_point_direction(result, last_idx, last_idx - 1, last_idx);
  }

  return result;
}

std::vector<LaneSegment> LaneletConverter::convert_to_lane_segments(
  const int64_t num_lane_points) const
{
  std::vector<LaneSegment> lane_segments;
  lane_segments.reserve(lanelet_map_ptr_->laneletLayer.size());
  // parse lanelet layers
  for (const auto & lanelet : lanelet_map_ptr_->laneletLayer) {
    const auto lanelet_subtype = to_subtype_name(lanelet);
    if (!is_lane_like(lanelet_subtype)) {
      std::cerr << "Skipping lanelet ID, since it is not LaneLike: " << lanelet.id() << std::endl;
      continue;
    }
    Polyline lane_polyline(MapType::Unused);
    std::vector<BoundarySegment> left_boundary_segments;
    std::vector<BoundarySegment> right_boundary_segments;
    // TODO(Daniel): avoid unnecessary copy and creation
    auto points = from_linestring(lanelet.centerline3d());
    lane_polyline.assign_waypoints(interpolate_points(points, num_lane_points));
    const auto left_bound = lanelet.leftBound3d();
    auto left_points = from_linestring(left_bound);
    left_boundary_segments.emplace_back(
      MapType::Unused, interpolate_points(left_points, num_lane_points));
    const auto right_bound = lanelet.rightBound3d();
    auto right_points = from_linestring(right_bound);
    right_boundary_segments.emplace_back(
      MapType::Unused, interpolate_points(right_points, num_lane_points));

    const auto & attrs = lanelet.attributes();
    bool is_intersection = attrs.find("turn_direction") != attrs.end();
    std::optional<float> speed_limit_mps =
      attrs.find("speed_limit") != attrs.end()
        ? std::make_optional(
            autoware_utils_math::kmph2mps(std::stof(attrs.at("speed_limit").value())))
        : std::nullopt;

    lane_segments.emplace_back(
      lanelet.id(), lane_polyline, is_intersection, left_boundary_segments, right_boundary_segments,
      speed_limit_mps);
  }
  return lane_segments;
}

// Template function for converting any geometry type to lane points
template <typename GeometryType>
std::vector<LanePoint> LaneletConverter::from_geometry(
  const GeometryType & geometry, const geometry_msgs::msg::Point & position,
  double distance_threshold) noexcept
{
  if (geometry.size() == 0) {
    return {};
  }

  std::vector<LanePoint> output;
  for (auto itr = geometry.begin(); itr != geometry.end(); ++itr) {
    if (auto distance =
          std::hypot(itr->x() - position.x, itr->y() - position.y, itr->z() - position.z);
        distance > distance_threshold) {
      continue;
    }
    float dx{0.0f};
    float dy{0.0f};
    float dz{0.0f};
    if (itr == geometry.begin()) {
      dx = 0.0f;
      dy = 0.0f;
      dz = 0.0f;
    } else {
      dx = static_cast<float>(itr->x() - (itr - 1)->x());
      dy = static_cast<float>(itr->y() - (itr - 1)->y());
      dz = static_cast<float>(itr->z() - (itr - 1)->z());
      normalize_direction(dx, dy, dz);
    }
    output.emplace_back(
      itr->x(), itr->y(), itr->z(), dx, dy, dz, 0.0);  // TODO(danielsanchezaran): Label ID
  }
  return output;
}

template <typename GeometryType>
std::vector<LanePoint> LaneletConverter::from_geometry(const GeometryType & geometry) noexcept
{
  geometry_msgs::msg::Point position;
  position.x = 0.0;
  position.y = 0.0;
  position.z = 0.0;
  return from_geometry(geometry, position, std::numeric_limits<double>::max());
}

std::vector<LanePoint> LaneletConverter::from_linestring(
  const lanelet::ConstLineString3d & linestring) noexcept
{
  return from_geometry(linestring);
}

std::vector<LanePoint> LaneletConverter::from_linestring(
  const lanelet::ConstLineString3d & linestring, const geometry_msgs::msg::Point & position,
  double distance_threshold) noexcept
{
  return from_geometry(linestring, position, distance_threshold);
}

std::vector<LanePoint> LaneletConverter::from_polygon(
  const lanelet::CompoundPolygon3d & polygon) noexcept
{
  return from_geometry(polygon);
}

std::vector<LanePoint> LaneletConverter::from_polygon(
  const lanelet::CompoundPolygon3d & polygon, const geometry_msgs::msg::Point & position,
  double distance_threshold) noexcept
{
  return from_geometry(polygon, position, distance_threshold);
}
}  // namespace autoware::diffusion_planner
