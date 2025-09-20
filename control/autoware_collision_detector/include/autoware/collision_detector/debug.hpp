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

#ifndef AUTOWARE__COLLISION_DETECTOR__DEBUG_HPP_
#define AUTOWARE__COLLISION_DETECTOR__DEBUG_HPP_

#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <optional>

namespace autoware::collision_detector
{
inline visualization_msgs::msg::MarkerArray generate_debug_markers(
  const autoware_utils_geometry::Polygon2d & ego_polygon,
  const std::optional<std::pair<double, geometry_msgs::msg::Point>> & nearest_obstacle_data,
  const bool is_error)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.ns = "ego_footprint";
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale = autoware_utils::create_marker_scale(0.1, 0.0, 0.0);
  if (is_error) {
    marker.color = autoware_utils::create_marker_color(1.0, 0.0, 0.0, 0.8);
  } else {
    marker.color = autoware_utils::create_marker_color(0.0, 1.0, 0.0, 0.8);
  }

  for (const auto & p : ego_polygon.outer()) {
    marker.points.push_back(autoware_utils::create_marker_position(p.x(), p.y(), 0.0));
  }
  if (!ego_polygon.outer().empty()) {
    marker.points.push_back(marker.points.front());
  }
  marker_array.markers.push_back(marker);

  marker.header.frame_id = "map";
  marker.ns = "nearest_obstacle";
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.scale = autoware_utils::create_marker_scale(0.5, 0.5, 0.5);       // Sphere diameter
  marker.color = autoware_utils::create_marker_color(1.0, 0.0, 0.0, 0.8);  // Red
  if (nearest_obstacle_data) {
    const auto & [_, nearest_point_msg] = *nearest_obstacle_data;
    marker.pose.position = nearest_point_msg;
    marker.action = visualization_msgs::msg::Marker::ADD;
  } else {
    marker.action = visualization_msgs::msg::Marker::DELETE;
  }
  marker_array.markers.push_back(marker);
  return marker_array;
}
}  // namespace autoware::collision_detector

#endif  // AUTOWARE__COLLISION_DETECTOR__DEBUG_HPP_
