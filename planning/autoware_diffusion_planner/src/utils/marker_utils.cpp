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

#include "autoware/diffusion_planner/utils/marker_utils.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"

#include <builtin_interfaces/msg/duration.hpp>
#include <rclcpp/duration.hpp>

#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/point.hpp>  // Include the header for Point

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::utils
{
using geometry_msgs::msg::Point;

namespace
{
// Helper function to create a basic marker with common properties
Marker create_base_marker(
  const Time & stamp, const std::string & frame_id, const std::string & ns, int id, int type,
  const ColorRGBA & color, const rclcpp::Duration & lifetime, double scale_x)
{
  Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale_x;
  marker.color = color;
  marker.lifetime = lifetime;
  return marker;
}

// Helper to extract point data from lane vector
struct LanePointData
{
  double x, y;
  double lb_x, lb_y;
  double rb_x, rb_y;
  double norm;
};

LanePointData extract_lane_point(
  const std::vector<float> & lane_vector, int64_t l, int64_t p, int64_t P, int64_t D)
{
  LanePointData data;
  data.x = lane_vector[P * D * l + p * D + X];
  data.y = lane_vector[P * D * l + p * D + Y];
  data.lb_x = lane_vector[P * D * l + p * D + LB_X] + data.x;
  data.lb_y = lane_vector[P * D * l + p * D + LB_Y] + data.y;
  data.rb_x = lane_vector[P * D * l + p * D + RB_X] + data.x;
  data.rb_y = lane_vector[P * D * l + p * D + RB_Y] + data.y;
  data.norm = std::sqrt(data.x * data.x + data.y * data.y);
  return data;
}

// Helper to transform points
struct TransformedPoints
{
  double x, y, z;
  double lb_x, lb_y;
  double rb_x, rb_y;
};

TransformedPoints transform_lane_points(
  const LanePointData & data, const Eigen::Matrix4d & transform)
{
  Eigen::Matrix<double, 4, 3> points;
  points << data.x, data.lb_x, data.rb_x, data.y, data.lb_y, data.rb_y, 0.0, 0.0, 0.0, 1.0, 1.0,
    1.0;

  Eigen::Matrix<double, 4, 3> transformed = transform * points;

  TransformedPoints result;
  result.x = transformed(0, 0);
  result.y = transformed(1, 0);
  result.lb_x = transformed(0, 1);
  result.lb_y = transformed(1, 1);
  result.rb_x = transformed(0, 2);
  result.rb_y = transformed(1, 2);
  result.z = transformed(2, 0) + 0.1f;
  return result;
}

// Helper to add point to marker
void add_point_to_marker(Marker & marker, double x, double y, double z)
{
  Point pt;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  marker.points.push_back(pt);
}

}  // anonymous namespace

ColorRGBA get_traffic_light_color(float g, float y, float r, const ColorRGBA & original_color)
{
  ColorRGBA color;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 0.0;
  color.a = 0.8;
  if (static_cast<bool>(g)) {
    color.g = 1.0;
    return color;
  }
  if (static_cast<bool>(y)) {
    color.g = 1.0;
    color.r = 1.0;
    return color;
  }

  if (static_cast<bool>(r)) {
    color.r = 1.0;
    return color;
  }
  return original_color;
};

MarkerArray create_linestring_marker(
  const Eigen::Matrix4d & transform_ego_to_map, const std::vector<float> & linestring_vector,
  const std::vector<int64_t> & shape, const Time & stamp, const rclcpp::Duration & lifetime,
  const std::string & frame_id)
{
  MarkerArray marker_array;
  if (shape.size() < 4) {
    return marker_array;
  }
  const int64_t P = shape[2];
  const int64_t D = shape[3];
  if (P <= 0 || D <= 0 || linestring_vector.size() % static_cast<size_t>(P * D) != 0) {
    return marker_array;
  }
  const size_t num_line_strings = linestring_vector.size() / static_cast<size_t>(P * D);
  constexpr float near_zero_threshold = 1e-2f;
  // The tensor layout is [x, y, one_hot_types...], so the type indices come from 2 + LineStringType
  constexpr int64_t stop_line_type_idx = 2 + LINE_STRING_TYPE_STOP_LINE;
  constexpr int64_t road_border_type_idx = 2 + LINE_STRING_TYPE_ROAD_BORDER;

  // Dark red
  ColorRGBA road_border_color;
  road_border_color.r = 0.8f;
  road_border_color.g = 0.0f;
  road_border_color.b = 0.2f;
  road_border_color.a = 0.8f;

  // Orange
  ColorRGBA stop_line_color;
  stop_line_color.r = 1.0f;
  stop_line_color.g = 0.65f;
  stop_line_color.b = 0.0f;
  stop_line_color.a = 0.8f;

  for (size_t l = 0; l < num_line_strings; ++l) {
    // All points in a line string share the same type; check the first point's type flag
    const bool is_stop_line =
      linestring_vector[P * D * static_cast<int64_t>(l) + stop_line_type_idx] > 0.5f;
    const bool is_road_border =
      linestring_vector[P * D * static_cast<int64_t>(l) + road_border_type_idx] > 0.5f;

    if (!is_road_border && !is_stop_line) {
      continue;
    }

    std::string ns = is_stop_line ? "stop_line" : "road_border";
    ColorRGBA color = is_stop_line ? stop_line_color : road_border_color;
    Marker marker = create_base_marker(
      stamp, frame_id, ns, static_cast<int>(l), Marker::LINE_STRIP, color, lifetime, 0.2);

    float total_norm = 0.0f;
    for (int64_t p = 0; p < P; ++p) {
      const float x = linestring_vector[P * D * static_cast<int64_t>(l) + p * D + X];
      const float y = linestring_vector[P * D * static_cast<int64_t>(l) + p * D + Y];
      const float norm = std::sqrt(x * x + y * y);
      total_norm += norm;

      if (norm < near_zero_threshold) {
        continue;
      }

      const Eigen::Vector4d pt_ego(x, y, 0.0, 1.0);
      const Eigen::Vector4d pt_map = transform_ego_to_map * pt_ego;
      add_point_to_marker(marker, pt_map.x(), pt_map.y(), pt_map.z() + 0.1);
    }

    if (total_norm < near_zero_threshold || marker.points.empty()) {
      continue;
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

MarkerArray create_lane_marker(
  const Eigen::Matrix4d & transform_ego_to_map, const std::vector<float> & lane_vector,
  const std::vector<int64_t> & shape, const Time & stamp, const rclcpp::Duration & lifetime,
  const std::array<float, 4> colors, const std::string & frame_id,
  const bool set_traffic_light_color)
{
  MarkerArray marker_array;
  const int64_t P = shape[2];
  const int64_t D = shape[3];
  const size_t num_segments = lane_vector.size() / (P * D);
  int64_t segment_count = 0;
  constexpr double near_zero_threshold = 1e-2;
  // Setup colors
  ColorRGBA lane_color;
  lane_color.r = colors[0];
  lane_color.g = colors[1];
  lane_color.b = colors[2];
  lane_color.a = colors[3];

  ColorRGBA bounds_color;
  bounds_color.r = 0.9f;
  bounds_color.g = 0.65f;
  bounds_color.b = 0.0f;
  bounds_color.a = 0.8f;

  for (size_t l = 0; l < num_segments; ++l) {
    // Create markers for this segment
    Marker marker_centerline = create_base_marker(
      stamp, frame_id, "lane", static_cast<int>(l), Marker::LINE_STRIP, lane_color, lifetime, 0.3);

    Marker marker_left_bound = create_base_marker(
      stamp, frame_id, "lane_lb", static_cast<int>(l), Marker::LINE_STRIP, bounds_color, lifetime,
      0.3);

    Marker marker_right_bound = create_base_marker(
      stamp, frame_id, "lane_rb", static_cast<int>(l), Marker::LINE_STRIP, bounds_color, lifetime,
      0.3);

    // Sphere marker with alternating colors
    ColorRGBA sphere_color;
    sphere_color.r = segment_count % 2 == 0 ? 0.1f : 0.9f;
    sphere_color.g = segment_count % 2 == 0 ? 0.9f : 0.1f;
    sphere_color.b = 0.9f;
    sphere_color.a = 0.8f;

    Marker marker_sphere = create_base_marker(
      stamp, frame_id, "sphere", static_cast<int>(l), Marker::SPHERE_LIST, sphere_color, lifetime,
      0.5);
    marker_sphere.scale.y = 0.5;
    marker_sphere.scale.z = 0.5;

    // Apply traffic light color if requested
    if (set_traffic_light_color) {
      const auto g = lane_vector[P * D * l + 0 * D + TRAFFIC_LIGHT_GREEN];
      const auto y = lane_vector[P * D * l + 0 * D + TRAFFIC_LIGHT_YELLOW];
      const auto r = lane_vector[P * D * l + 0 * D + TRAFFIC_LIGHT_RED];
      marker_centerline.color = get_traffic_light_color(g, y, r, lane_color);
    }

    // Process points for this segment
    float total_norm = 0.0f;
    for (int64_t p = 0; p < P; ++p) {
      // Extract point data
      LanePointData point_data = extract_lane_point(lane_vector, l, p, P, D);
      total_norm += point_data.norm;

      // Skip near-zero points (likely padding)
      if (point_data.norm < near_zero_threshold) {
        continue;
      }

      // Transform points from ego to map frame
      TransformedPoints transformed = transform_lane_points(point_data, transform_ego_to_map);

      // Add points to respective markers
      add_point_to_marker(marker_centerline, transformed.x, transformed.y, transformed.z);
      add_point_to_marker(marker_left_bound, transformed.lb_x, transformed.lb_y, transformed.z);
      add_point_to_marker(marker_right_bound, transformed.rb_x, transformed.rb_y, transformed.z);
      add_point_to_marker(marker_sphere, transformed.x, transformed.y, transformed.z + 0.1f);
    }

    // Skip empty segments
    if (total_norm < near_zero_threshold) {
      continue;
    }
    ++segment_count;

    // Add non-empty markers to array
    if (!marker_sphere.points.empty()) {
      marker_array.markers.push_back(marker_sphere);
    }
    if (!marker_centerline.points.empty()) {
      marker_array.markers.push_back(marker_centerline);
    }
    if (!marker_left_bound.points.empty()) {
      marker_array.markers.push_back(marker_left_bound);
    }
    if (!marker_right_bound.points.empty()) {
      marker_array.markers.push_back(marker_right_bound);
    }
  }

  return marker_array;
}

}  // namespace autoware::diffusion_planner::utils
