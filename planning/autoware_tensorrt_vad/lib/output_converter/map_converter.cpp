// Copyright 2025 TIER IV.
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

#include "autoware/tensorrt_vad/output_converter/map_converter.hpp"

namespace autoware::tensorrt_vad::vad_interface
{

OutputMapConverter::OutputMapConverter(
  const CoordinateTransformer & coordinate_transformer, const VadInterfaceConfig & config)
: Converter(coordinate_transformer, config)
{
}

std::array<float, 3> OutputMapConverter::get_color_for_type(const std::string & type) const
{
  auto color_it = config_.map_colors.find(type);
  if (color_it != config_.map_colors.end()) {
    return color_it->second;
  } else {
    // Use white color as default
    return {1.0f, 1.0f, 1.0f};
  }
}

visualization_msgs::msg::Marker OutputMapConverter::create_polyline_marker(
  const MapPolyline & map_polyline, const int32_t marker_id, const rclcpp::Time & stamp,
  const Eigen::Matrix4d & base2map_transform) const
{
  visualization_msgs::msg::Marker marker;

  const std::string & type = map_polyline.type;
  const auto & polyline = map_polyline.points;

  marker.ns = type;       // namespace shows the type of the polyline
  marker.id = marker_id;  // set unique ID for each marker
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // orientation is fixed.
  marker.pose.orientation.w = 1.0;

  // Set color based on type
  const auto color = get_color_for_type(type);
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 0.8;

  // Transform each point in the polyline and add to the marker
  for (const auto & point : polyline) {
    if (point.size() >= 2) {
      Eigen::Vector4d base_point(
        static_cast<double>(point[0]), static_cast<double>(point[1]), 0.0, 1.0);
      Eigen::Vector4d map_point = base2map_transform * base_point;

      geometry_msgs::msg::Point geometry_point;
      geometry_point.x = map_point[0];
      geometry_point.y = map_point[1];
      geometry_point.z = map_point[2];

      marker.points.push_back(geometry_point);
    }
  }

  // Decide how to display the marker
  if (marker.points.size() >= 2) {
    // If there are 2 or more points, display as a line
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale.x = 0.1;  // Line thickness
  } else {
    // If polyline does not have 2 or more points, set invalid type
    marker.type = visualization_msgs::msg::Marker::ARROW;  // This will be filtered out
  }

  return marker;
}

visualization_msgs::msg::MarkerArray OutputMapConverter::process_map_points(
  const std::vector<MapPolyline> & vad_map_polylines, const rclcpp::Time & stamp,
  const Eigen::Matrix4d & base2map_transform) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  int32_t marker_id = 0;
  for (const auto & map_polyline : vad_map_polylines) {
    const auto & polyline = map_polyline.points;

    if (polyline.empty()) {
      ++marker_id;
      continue;  // if polyline is empty, skip
    }

    auto marker = create_polyline_marker(map_polyline, marker_id++, stamp, base2map_transform);

    // Only add markers with valid line strips (at least 2 points)
    if (marker.type == visualization_msgs::msg::Marker::LINE_STRIP && marker.points.size() >= 2) {
      marker_array.markers.push_back(marker);
    }
  }

  return marker_array;
}

}  // namespace autoware::tensorrt_vad::vad_interface
