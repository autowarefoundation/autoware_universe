// Copyright 2025 Tier IV, Inc.
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

#include "node.hpp"

#include <autoware_utils/ros/parameter.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <pcl/common/io.h>
#include <pcl/point_types.h>

#include <chrono>
#include <memory>

namespace autoware::compare_map_segmentation
{

LaneletElevationFilterComponent::LaneletElevationFilterComponent(
  const rclcpp::NodeOptions & node_options)
: Filter("LaneletElevationFilter", node_options)
{
  loadParameters();

  // Initialize the filter
  filter_ = std::make_unique<LaneletElevationFilter>(params_);

  // Initialize timing utilities following fusion node pattern
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();

  // Initialize debug publisher if enabled
  if (params_.enable_debug) {
    try {
      // Initialize DebugPublisher following fusion node pattern
      debug_publisher_ = std::make_unique<autoware_utils::DebugPublisher>(this, get_name());

      // Initialize debug markers publisher
      debug_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/debug/elevation_grid_markers", rclcpp::QoS(1).transient_local());

      RCLCPP_INFO(this->get_logger(), "Debug publishers initialized");
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Failed to initialize debug publishers: %s", e.what());
      params_.enable_debug = false;  // Disable debug mode if initialization fails
    }
  }

  // Subscribe to lanelet map
  map_sub_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/lanelet_map", rclcpp::QoS(1).transient_local(),
    std::bind(&LaneletElevationFilterComponent::onMap, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "LaneletElevationFilter has been initialized.");

  printParameters();
}

void LaneletElevationFilterComponent::printParameters()
{
  RCLCPP_INFO(this->get_logger(), "=== Lanelet Elevation Filter Configuration ===");
  RCLCPP_INFO(this->get_logger(), "Grid resolution: %.2f m", params_.grid_resolution);
  RCLCPP_INFO(this->get_logger(), "Height threshold: %.2f m", params_.height_threshold);
  RCLCPP_INFO(this->get_logger(), "Sampling distance: %.2f m", params_.sampling_distance);
  RCLCPP_INFO(this->get_logger(), "Target frame: %s", params_.target_frame.c_str());
  RCLCPP_INFO(this->get_logger(), "Debug mode: %s", params_.enable_debug ? "enabled" : "disabled");
}

void LaneletElevationFilterComponent::loadParameters()
{
  // Load parameters with default values
  params_.grid_resolution = this->declare_parameter<double>("grid_resolution", 1.0);
  params_.height_threshold = this->declare_parameter<double>("height_threshold", 2.0);
  params_.sampling_distance = this->declare_parameter<double>("sampling_distance", 0.5);
  params_.target_frame = this->declare_parameter<std::string>("target_frame", "map");
  params_.enable_debug = this->declare_parameter<bool>("enable_debug", false);

  // Validate parameters
  if (params_.grid_resolution <= 0.0) {
    throw std::invalid_argument("grid_resolution must be positive");
  }
  if (params_.height_threshold <= 0.0) {
    throw std::invalid_argument("height_threshold must be positive");
  }
  if (params_.sampling_distance <= 0.0) {
    throw std::invalid_argument("sampling_distance must be positive");
  }
}

void LaneletElevationFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  // Start timing for debug if enabled - do this FIRST
  if (params_.enable_debug && stop_watch_ptr_) {
    stop_watch_ptr_->tic("processing_time");
  }

  if (!input) {
    RCLCPP_ERROR(this->get_logger(), "Input point cloud is null");
    return;
  }

  if (!filter_) {
    // If filter is not initialized, pass through the input
    output = *input;
    return;
  }

  if (!filter_->getGridProcessor()) {
    RCLCPP_ERROR(this->get_logger(), "Grid processor is not initialized");
    output = *input;
    return;
  }

  // Use similar direct processing as other filters
  int point_step = input->point_step;

  // Check if the input has valid fields
  if (input->fields.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Input point cloud has no fields");
    output = *input;
    return;
  }

  int x_idx = pcl::getFieldIndex(*input, "x");
  int y_idx = pcl::getFieldIndex(*input, "y");
  int z_idx = pcl::getFieldIndex(*input, "z");

  if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
    RCLCPP_ERROR(this->get_logger(), "Input point cloud missing x, y, or z fields");
    output = *input;
    return;
  }

  int offset_x = input->fields[x_idx].offset;
  int offset_y = input->fields[y_idx].offset;
  int offset_z = input->fields[z_idx].offset;

  output.data.resize(input->data.size());
  output.point_step = point_step;
  size_t output_size = 0;

  const double height_threshold = params_.height_threshold;

  for (size_t global_offset = 0; global_offset < input->data.size(); global_offset += point_step) {
    pcl::PointXYZ point{};
    std::memcpy(&point.x, &input->data[global_offset + offset_x], sizeof(float));
    std::memcpy(&point.y, &input->data[global_offset + offset_y], sizeof(float));
    std::memcpy(&point.z, &input->data[global_offset + offset_z], sizeof(float));

    // Check if point is finite and within elevation threshold
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    try {
      if (filter_->getGridProcessor()->isPointValid(point.x, point.y, point.z, height_threshold)) {
        std::memcpy(&output.data[output_size], &input->data[global_offset], point_step);
        output_size += point_step;
      }
    } catch (const std::exception & e) {
      // If there's an error with grid processing, include the point to be safe
      std::memcpy(&output.data[output_size], &input->data[global_offset], point_step);
      output_size += point_step;
    }
  }

  // Set output cloud properties
  output.header = input->header;
  output.fields = input->fields;
  output.data.resize(output_size);
  output.height = input->height;
  output.width = output_size / point_step / output.height;
  output.row_step = output_size / output.height;
  output.is_bigendian = input->is_bigendian;
  output.is_dense = input->is_dense;

  // Publish processing time for debug if available
  if (params_.enable_debug && debug_publisher_ && stop_watch_ptr_) {
    try {
      const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);

      // Create Float64Stamped message following fusion node pattern
      autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
      processing_time_msg.stamp = input->header.stamp;
      processing_time_msg.data = processing_time_ms;

      debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
        "debug/processing_time_ms", processing_time_msg);

      RCLCPP_DEBUG(this->get_logger(), "Published processing time: %.2f ms", processing_time_ms);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Failed to publish debug processing time: %s",
        e.what());
    }
  }
}

void LaneletElevationFilterComponent::onMap(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg)
{
  try {
    RCLCPP_INFO(this->get_logger(), "Received lanelet map, processing...");

    const auto start_time = this->now();

    // Set the lanelet map in the filter
    filter_->setLaneletMap(msg);
    const auto end_time = this->now();

    const auto processing_time = (end_time - start_time).seconds() * 1000.0;

    RCLCPP_INFO(
      this->get_logger(), "Lanelet map processed successfully in %.2f ms", processing_time);

    // Publish debug markers created by the lanelet elevation filter
    if (params_.enable_debug && debug_markers_pub_) {
      auto marker_array = filter_->createDebugMarkers(this->now());
      if (!marker_array.markers.empty()) {
        debug_markers_pub_->publish(marker_array);
        RCLCPP_INFO(
          this->get_logger(), "Published %zu debug elevation markers", marker_array.markers.size());
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to process lanelet map: %s", e.what());
  }
}

}  // namespace autoware::compare_map_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::compare_map_segmentation::LaneletElevationFilterComponent)
