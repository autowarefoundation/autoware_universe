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

#include "node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/pointcloud_preprocessor/transform_info.hpp>
#include <autoware_utils/ros/parameter.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <pcl/common/io.h>
#include <pcl/point_types.h>

#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

namespace autoware::compare_map_segmentation
{

LaneletElevationFilterComponent::LaneletElevationFilterComponent(
  const rclcpp::NodeOptions & node_options)
: Filter("LaneletElevationFilter", node_options)
{
  loadParameters();

  filter_ = std::make_unique<LaneletElevationFilter>(params_);

  if (params_.enable_debug) {
    stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<autoware_utils::DebugPublisher>(this, get_name());

    debug_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/debug/elevation_grid_markers", rclcpp::QoS(1).transient_local());
    RCLCPP_INFO(this->get_logger(), "Debug publishers initialized");
  }

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
  RCLCPP_INFO(this->get_logger(), "Extension count: %d cells", params_.extension_count);
  RCLCPP_INFO(this->get_logger(), "Target frame: %s", params_.target_frame.c_str());
  RCLCPP_INFO(
    this->get_logger(), "Cache directory: %s",
    params_.cache_directory.empty() ? "/tmp/autoware_lanelet_cache/"
                                    : params_.cache_directory.c_str());
  RCLCPP_INFO(
    this->get_logger(), "Require map coverage: %s",
    params_.require_map_coverage ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), "Debug mode: %s", params_.enable_debug ? "enabled" : "disabled");
}

void LaneletElevationFilterComponent::loadParameters()
{
  // Load parameters with default values
  params_.grid_resolution = this->declare_parameter<double>("grid_resolution", 1.0);
  params_.height_threshold = this->declare_parameter<double>("height_threshold", 2.0);
  params_.sampling_distance = this->declare_parameter<double>("sampling_distance", 0.5);
  params_.target_frame = this->declare_parameter<std::string>("target_frame", "map");
  params_.cache_directory = this->declare_parameter<std::string>(
    "cache_directory",
    "$(find-pkg-share autoware_compare_map_segmentation)/data/lanelet_grid_cache");
  params_.enable_debug = this->declare_parameter<bool>("enable_debug", false);
  params_.extension_count = this->declare_parameter<int>("extension_count", 20);
  params_.require_map_coverage = this->declare_parameter<bool>("require_map_coverage", false);

  // Validate parameters
  if (params_.grid_resolution <= 0.1) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid grid_resolution: %f. Must be greater than 0.1 meters.",
      params_.grid_resolution);
    throw std::invalid_argument("grid_resolution must be positive");
  }

  if (params_.height_threshold < 0.0) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid height_threshold: %f. Must be non-negative.",
      params_.height_threshold);
    throw std::invalid_argument("height_threshold must be non-negative");
  }

  if (params_.extension_count < 0) {
    RCLCPP_WARN(
      this->get_logger(), "Extension count is negative (%d), setting to 0",
      params_.extension_count);
    params_.extension_count = 0;
  }

  // Resolve ROS package path
  if (params_.cache_directory.find("$(find-pkg-share") != std::string::npos) {
    std::string resolved_cache_dir = resolvePackageSharePath(params_.cache_directory);
    if (!resolved_cache_dir.empty()) {
      params_.cache_directory = resolved_cache_dir;
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Failed to resolve cache directory: %s, using fallback",
        params_.cache_directory.c_str());
      params_.cache_directory = "/tmp/autoware_lanelet_cache";
    }
  }

  // Ensure cache directory is set to a reliable path
  if (params_.cache_directory.empty()) {
    params_.cache_directory = "/tmp/autoware_lanelet_cache";
  }

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
  if (params_.extension_count < 0) {
    throw std::invalid_argument("extension_count must be non-negative");
  }
}

void LaneletElevationFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  if (params_.enable_debug && stop_watch_ptr_) {
    stop_watch_ptr_->tic("processing_time");
  }

  if (!input) {
    RCLCPP_ERROR(this->get_logger(), "Input point cloud is null");
    return;
  }

  if (!filter_) {
    output = *input;
    return;
  }

  if (!filter_->getGridProcessor()) {
    RCLCPP_ERROR(this->get_logger(), "Grid processor is not initialized");
    output = *input;
    return;
  }

  tf_input_orig_frame_ = input->header.frame_id;

  TransformInfo transform_info;
  transform_info.need_transform = false;

  if (!params_.target_frame.empty() && input->header.frame_id != params_.target_frame) {
    auto eigen_transform_opt = managed_tf_buffer_->getTransform<Eigen::Matrix4f>(
      params_.target_frame, input->header.frame_id, input->header.stamp,
      rclcpp::Duration::from_seconds(1.0), this->get_logger());
    if (!eigen_transform_opt) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get transformation matrix");
      output = *input;
      return;
    }
    transform_info.eigen_transform = *eigen_transform_opt;
    transform_info.need_transform = true;
  }

  int point_step = input->point_step;
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

    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    float transformed_x = point.x;
    float transformed_y = point.y;
    float transformed_z = point.z;

    if (transform_info.need_transform) {
      Eigen::Vector4f point_eigen(point.x, point.y, point.z, 1.0f);
      Eigen::Vector4f transformed_point = transform_info.eigen_transform * point_eigen;

      transformed_x = transformed_point(0);
      transformed_y = transformed_point(1);
      transformed_z = transformed_point(2);
    }

    try {
      bool should_keep_point = filter_->getGridProcessor()->isPointValid(
        transformed_x, transformed_y, transformed_z, height_threshold,
        params_.require_map_coverage);

      if (should_keep_point) {
        std::memcpy(&output.data[output_size], &input->data[global_offset], point_step);
        output_size += point_step;
      }
    } catch (const std::exception & e) {
      std::memcpy(&output.data[output_size], &input->data[global_offset], point_step);
      output_size += point_step;
    }
  }

  output.header = input->header;
  output.fields = input->fields;
  output.data.resize(output_size);
  output.height = input->height;
  output.width = output_size / point_step / output.height;
  output.row_step = output_size / output.height;
  output.is_bigendian = input->is_bigendian;
  output.is_dense = input->is_dense;

  if (params_.enable_debug && debug_publisher_ && stop_watch_ptr_) {
    try {
      const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);

      autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
      processing_time_msg.stamp = input->header.stamp;
      processing_time_msg.data = processing_time_ms;

      debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
        "debug/processing_time_ms", processing_time_msg);
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
    const auto start_time = std::chrono::high_resolution_clock::now();

    filter_->setLaneletMap(msg);

    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto processing_time =
      std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    const double processing_time_ms = processing_time.count() / 1000.0;

    RCLCPP_INFO(
      this->get_logger(), "Lanelet map processed successfully in %.2f ms", processing_time_ms);

    // Publish debug markers created by the lanelet elevation filter
    if (params_.enable_debug && debug_markers_pub_) {
      auto marker_array = filter_->createDebugMarkers(this->now());
      if (!marker_array.markers.empty()) {
        debug_markers_pub_->publish(marker_array);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to process lanelet map: %s", e.what());
  }
}

std::string LaneletElevationFilterComponent::resolvePackageSharePath(const std::string & path)
{
  size_t start = path.find("$(find-pkg-share ");
  if (start == std::string::npos) {
    return path;
  }

  size_t package_start = start + strlen("$(find-pkg-share ");
  size_t package_end = path.find(")", package_start);
  if (package_end == std::string::npos) {
    RCLCPP_ERROR(this->get_logger(), "Invalid $(find-pkg-share) syntax in path: %s", path.c_str());
    return "";
  }

  std::string package_name = path.substr(package_start, package_end - package_start);

  try {
    // Get the package share directory
    std::string package_share_dir = ament_index_cpp::get_package_share_directory(package_name);

    // Replace the substitution with the actual path
    std::string resolved_path = path;
    resolved_path.replace(start, package_end - start + 1, package_share_dir);

    // Create the directory if it doesn't exist
    std::filesystem::create_directories(resolved_path);

    return resolved_path;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to resolve package '%s': %s", package_name.c_str(), e.what());
    return "";
  }
}

}  // namespace autoware::compare_map_segmentation

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::compare_map_segmentation::LaneletElevationFilterComponent)
