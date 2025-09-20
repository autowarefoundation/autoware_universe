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

#include "lanelet_elevation_filter.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

namespace autoware::compare_map_segmentation
{

LaneletElevationFilter::LaneletElevationFilter(const LaneletElevationFilterParams & params)
: params_(params), map_initialized_(false)
{
  grid_processor_ = std::make_shared<GridProcessor>(params_.grid_resolution);
}

void LaneletElevationFilter::setLaneletMap(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & map_msg)
{
  try {
    lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(*map_msg, lanelet_map_);

    initializeGridFromMap();
    map_initialized_ = true;
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to convert lanelet map: " + std::string(e.what()));
  }
}

void LaneletElevationFilter::initializeGridFromMap()
{
  if (!lanelet_map_) {
    return;
  }

  grid_processor_->processLanelets(
    lanelet_map_, params_.sampling_distance, params_.extension_count, params_.cache_directory);
}

visualization_msgs::msg::MarkerArray LaneletElevationFilter::createDebugMarkers(
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  if (!map_initialized_ || !grid_processor_) {
    return marker_array;
  }

  auto grid_cells = grid_processor_->getGridCells();
  auto [min_elevation, max_elevation] = grid_processor_->getGridBounds();

  if (grid_cells.empty()) {
    return marker_array;
  }

  // If elevation range is very small, use a default color scheme
  bool use_default_color = std::abs(max_elevation - min_elevation) < 1e-3;

  // Create grid visualization markers
  for (size_t i = 0; i < grid_cells.size(); ++i) {
    const auto & [index, cell] = grid_cells[i];

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = params_.target_frame;
    marker.header.stamp = stamp;
    marker.ns = "elevation_grid";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set position (center of grid cell)
    marker.pose.position.x = index.x * params_.grid_resolution + params_.grid_resolution * 0.5;
    marker.pose.position.y = index.y * params_.grid_resolution + params_.grid_resolution * 0.5;
    marker.pose.position.z = cell.average_height;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set scale
    marker.scale.x = params_.grid_resolution * 0.9;  // Slightly smaller for visibility
    marker.scale.y = params_.grid_resolution * 0.9;
    marker.scale.z = 0.1;  // Thin cube to represent surface

    // Color based on elevation (blue = low, red = high)
    if (use_default_color) {
      // Use a default green color when elevation range is very small
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    } else {
      double elevation_ratio =
        (cell.average_height - min_elevation) / (max_elevation - min_elevation);
      marker.color.r = static_cast<float>(elevation_ratio);
      marker.color.g = 0.0f;
      marker.color.b = static_cast<float>(1.0 - elevation_ratio);
    }
    marker.color.a = 0.7f;  // Semi-transparent

    marker.lifetime = rclcpp::Duration::from_seconds(1.0);

    marker_array.markers.push_back(marker);
  }

  // Add elevation scale marker
  visualization_msgs::msg::Marker scale_marker;
  scale_marker.header.frame_id = params_.target_frame;
  scale_marker.header.stamp = stamp;
  scale_marker.ns = "elevation_scale";
  scale_marker.id = 0;
  scale_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  scale_marker.action = visualization_msgs::msg::Marker::ADD;

  // Position scale text
  scale_marker.pose.position.x = 0.0;
  scale_marker.pose.position.y = 0.0;
  scale_marker.pose.position.z = max_elevation + 2.0;

  scale_marker.pose.orientation.x = 0.0;
  scale_marker.pose.orientation.y = 0.0;
  scale_marker.pose.orientation.z = 0.0;
  scale_marker.pose.orientation.w = 1.0;

  scale_marker.scale.z = 1.0;  // Text size

  scale_marker.color.r = 1.0f;
  scale_marker.color.g = 1.0f;
  scale_marker.color.b = 1.0f;
  scale_marker.color.a = 1.0f;

  scale_marker.text = "Elevation: " + std::to_string(min_elevation) + "m (blue) - " +
                      std::to_string(max_elevation) + "m (red)";

  scale_marker.lifetime = rclcpp::Duration::from_seconds(1.0);

  marker_array.markers.push_back(scale_marker);

  return marker_array;
}

}  // namespace autoware::compare_map_segmentation
