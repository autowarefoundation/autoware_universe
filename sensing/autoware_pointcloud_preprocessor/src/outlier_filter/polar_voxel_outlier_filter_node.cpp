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

#include "autoware/pointcloud_preprocessor/outlier_filter/polar_voxel_outlier_filter_node.hpp"

#include <autoware/pointcloud_preprocessor/utility/memory.hpp>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <vector>
#include <map>
#include <cstdint>
#include <cstring>
#include <algorithm>

namespace autoware::pointcloud_preprocessor
{

PolarVoxelOutlierFilterComponent::PolarVoxelOutlierFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("PolarVoxelOutlierFilter", options)
{
  // set initial parameters
  {
    radius_resolution_ = declare_parameter<double>("radius_resolution");
    azimuth_resolution_ = declare_parameter<double>("azimuth_resolution");
    elevation_resolution_ = declare_parameter<double>("elevation_resolution");
    voxel_points_threshold_ = declare_parameter<int>("voxel_points_threshold");
    min_radius_ = declare_parameter<double>("min_radius");
    max_radius_ = declare_parameter<double>("max_radius");
    use_return_type_classification_ = declare_parameter<bool>("use_return_type_classification");
    primary_return_types_ = declare_parameter<std::vector<int>>("primary_return_types");
    secondary_return_types_ = declare_parameter<std::vector<int>>("secondary_return_types");
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PolarVoxelOutlierFilterComponent::param_callback, this, _1));
}

void PolarVoxelOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  if (indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }

  // Add input validation
  if (!input) {
    RCLCPP_ERROR(get_logger(), "Input point cloud is null");
    return;
  }

  // Check if the input point cloud has PointXYZIRCAEDT layout (with pre-computed polar coordinates)
  if (autoware::pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzircaedt(*input)) {
    RCLCPP_DEBUG(get_logger(), "Using PointXYZIRCAEDT format with pre-computed polar coordinates");
    filter_point_xyzircaedt(input, indices, output);
  } else {
    RCLCPP_DEBUG(get_logger(), "Using PointXYZ format, computing polar coordinates");
    filter_point_xyz(input, indices, output);
  }
}

void PolarVoxelOutlierFilterComponent::filter_point_xyz(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  (void)indices;  // Suppress unused parameter warning
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);

  // Map to store point indices per polar voxel
  std::map<PolarVoxelIndex, std::vector<size_t>> voxel_point_map;

  // First pass: count points in each polar voxel
  for (size_t i = 0; i < pcl_input->points.size(); ++i) {
    const auto & point = pcl_input->points[i];
    
    // Skip invalid points
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {  // NOLINT
      continue;
    }

    double radius = 0.0;
    double azimuth = 0.0;
    double elevation = 0.0;
    cartesian_to_polar(static_cast<double>(point.x), static_cast<double>(point.y), static_cast<double>(point.z), radius, azimuth, elevation);  // NOLINT

    // Skip points outside radius range
    if (radius < min_radius_ || radius > max_radius_) {
      continue;
    }

    // Skip points with insufficient radius (degenerate points at origin)
    if (std::abs(radius) < std::numeric_limits<double>::epsilon()) {
      continue; // Skip degenerate points
    }

    PolarVoxelIndex voxel_idx = cartesian_to_polar_voxel(static_cast<double>(point.x), static_cast<double>(point.y), static_cast<double>(point.z));  // NOLINT
    voxel_point_map[voxel_idx].push_back(i);
  }

  // Second pass: filter points based on voxel occupancy
  pcl_output->points.reserve(pcl_input->points.size());
  for (const auto & voxel_entry : voxel_point_map) {
    if (static_cast<int>(voxel_entry.second.size()) >= voxel_points_threshold_) {
      // Include all points from voxels that meet the threshold
      for (size_t point_idx : voxel_entry.second) {
        pcl_output->points.push_back(pcl_input->points[point_idx]);
      }
    }
  }

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

void PolarVoxelOutlierFilterComponent::filter_point_xyzircaedt(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  (void)indices;  // Suppress unused parameter warning
  
  // Copy input to output initially, then we'll filter it
  output = *input;
  
  // Map to store point indices per polar voxel with separate vectors for primary and secondary returns
  std::map<PolarVoxelIndex, VoxelPoints> voxel_point_map;

  // Create iterators for the required fields
  sensor_msgs::PointCloud2ConstIterator<float> iter_distance(*input, "distance");
  sensor_msgs::PointCloud2ConstIterator<float> iter_azimuth(*input, "azimuth");
  sensor_msgs::PointCloud2ConstIterator<float> iter_elevation(*input, "elevation");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_return_type(*input, "return_type");

  size_t point_idx = 0;
  // First pass: count points in each polar voxel using pre-computed coordinates
  for (; iter_distance != iter_distance.end(); ++iter_distance, ++iter_azimuth, ++iter_elevation, ++iter_return_type, ++point_idx) {

    // Use pre-computed polar coordinates from the point
    auto radius = static_cast<double>(*iter_distance);
    auto azimuth = static_cast<double>(*iter_azimuth);
    auto elevation = static_cast<double>(*iter_elevation);
    uint8_t return_type = *iter_return_type;

    // Skip invalid points
    if (!std::isfinite(radius) || !std::isfinite(azimuth) || !std::isfinite(elevation)) {
      continue;
    }

    // Skip points outside radius range
    if (radius < min_radius_ || radius > max_radius_) {
      continue;
    }

    PolarVoxelIndex voxel_idx = polar_to_polar_voxel(radius, azimuth, elevation);
    
    // Add point to appropriate vector based on return type
    classify_point_by_return_type(return_type, point_idx, voxel_idx, voxel_point_map);
  }

  // Collect valid point indices
  std::vector<bool> valid_points(input->width * input->height, false);
  valid_points.reserve(input->width * input->height);

  for (const auto & voxel_entry : voxel_point_map) {
    const auto & primary_returns = voxel_entry.second.primary_returns;
    const auto & secondary_returns = voxel_entry.second.secondary_returns;
    
    // Check if either primary or secondary returns meet the threshold
    if (static_cast<int>(primary_returns.size()) >= voxel_points_threshold_ ||
        static_cast<int>(secondary_returns.size()) >= voxel_points_threshold_) {
      // Add all points from both vectors
      for (size_t idx : primary_returns) {
        valid_points[idx] = true;
      }
      for (size_t idx : secondary_returns) {
        valid_points[idx] = true;
      }
    }
  }

  // Create filtered output
  sensor_msgs::msg::PointCloud2 filtered_cloud;
  filtered_cloud.header = input->header;
  filtered_cloud.fields = input->fields;
  filtered_cloud.is_bigendian = input->is_bigendian;
  filtered_cloud.point_step = input->point_step;
  filtered_cloud.is_dense = input->is_dense;

  // Count valid points
  size_t valid_count = 0;
  for (bool valid : valid_points) {
    if (valid) valid_count++;
  }

  filtered_cloud.height = 1;
  filtered_cloud.width = valid_count;
  filtered_cloud.row_step = filtered_cloud.width * filtered_cloud.point_step;
  filtered_cloud.data.resize(filtered_cloud.row_step);

  // Copy valid points
  size_t output_idx = 0;
  for (size_t i = 0; i < valid_points.size(); ++i) {
    if (valid_points[i]) {
      std::memcpy(
        &filtered_cloud.data[output_idx * filtered_cloud.point_step],
        &input->data[i * input->point_step],
        input->point_step);
      output_idx++;
    }
  }

  output = filtered_cloud;

  // Debug output for voxel statistics
  size_t total_voxels = voxel_point_map.size();
  size_t populated_voxels = 0;
  for (const auto& voxel : voxel_point_map) {
    if (static_cast<int>(voxel.second.primary_returns.size() + voxel.second.secondary_returns.size()) >= voxel_points_threshold_) {
      populated_voxels++;
    }
  }
  RCLCPP_DEBUG(get_logger(), "Voxel stats: %zu total, %zu populated", total_voxels, populated_voxels);
}

void PolarVoxelOutlierFilterComponent::cartesian_to_polar(
  double x, double y, double z, double & radius, double & azimuth, double & elevation)
{
  radius = std::sqrt(x * x + y * y + z * z);
  azimuth = std::atan2(y, x);
  elevation = std::atan2(z, std::sqrt(x * x + y * y));
}

PolarVoxelOutlierFilterComponent::PolarVoxelIndex 
PolarVoxelOutlierFilterComponent::cartesian_to_polar_voxel(double x, double y, double z) const
{
  double radius = 0.0;
  double azimuth = 0.0;
  double elevation = 0.0;
  cartesian_to_polar(x, y, z, radius, azimuth, elevation);
  return polar_to_polar_voxel(radius, azimuth, elevation);
}

PolarVoxelOutlierFilterComponent::PolarVoxelIndex 
PolarVoxelOutlierFilterComponent::polar_to_polar_voxel(double radius, double azimuth, double elevation) const
{
  PolarVoxelIndex voxel_idx{};
  voxel_idx.radius_idx = static_cast<int>(std::floor(radius / radius_resolution_));
  voxel_idx.azimuth_idx = static_cast<int>(std::floor(azimuth / azimuth_resolution_));
  voxel_idx.elevation_idx = static_cast<int>(std::floor(elevation / elevation_resolution_));
  return voxel_idx;
}

// Extract validation logic
bool PolarVoxelOutlierFilterComponent::validate_polar_coordinates(double radius, double azimuth, double elevation) const
{
  // Skip points with NaN or Inf values
  if (!std::isfinite(radius) || !std::isfinite(azimuth) || !std::isfinite(elevation)) {
    return false;
  }

  // Skip points outside the configured radius range
  if (radius < min_radius_ || radius > max_radius_) {
    return false;
  }

  return true;
}

// Helper function to check if return type is in primary returns list
bool PolarVoxelOutlierFilterComponent::is_primary_return_type(uint8_t return_type) const
{
  auto it = std::find(primary_return_types_.begin(), primary_return_types_.end(), static_cast<int64_t>(return_type));
  return it != primary_return_types_.end();
}

// Helper function to check if return type is in secondary returns list
// If not in primary or secondary, treat as secondary
bool PolarVoxelOutlierFilterComponent::is_secondary_return_type(uint8_t return_type) const
{
  auto it = std::find(secondary_return_types_.begin(), secondary_return_types_.end(), static_cast<int64_t>(return_type));
  return it != secondary_return_types_.end();
}

// Extract voxel classification logic  
void PolarVoxelOutlierFilterComponent::classify_point_by_return_type(uint8_t return_type, size_t point_idx, 
                                                               PolarVoxelIndex voxel_idx, 
                                                               std::map<PolarVoxelIndex, VoxelPoints>& voxel_point_map) const
{
  if (!use_return_type_classification_) {
    // When return type classification is disabled, treat all points as primary returns
    voxel_point_map[voxel_idx].primary_returns.push_back(point_idx);
  } else {
    // Use configured return type classification
    if (is_primary_return_type(return_type)) {
      voxel_point_map[voxel_idx].primary_returns.push_back(point_idx);
    } else if (is_secondary_return_type(return_type)) {
      voxel_point_map[voxel_idx].secondary_returns.push_back(point_idx);
    }
    // Points with return types not in either list are ignored when classification is enabled
  }
}

rcl_interfaces::msg::SetParametersResult PolarVoxelOutlierFilterComponent::param_callback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "radius_resolution", radius_resolution_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new radius resolution to: %f.", radius_resolution_);
  }
  if (get_param(p, "azimuth_resolution", azimuth_resolution_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new azimuth resolution to: %f.", azimuth_resolution_);
  }
  if (get_param(p, "elevation_resolution", elevation_resolution_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new elevation resolution to: %f.", elevation_resolution_);
  }
  if (get_param(p, "voxel_points_threshold", voxel_points_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new voxel points threshold to: %d.", voxel_points_threshold_);
  }
  if (get_param(p, "min_radius", min_radius_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new min radius to: %f.", min_radius_);
  }
  if (get_param(p, "max_radius", max_radius_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new max radius to: %f.", max_radius_);
  }
  if (get_param(p, "use_return_type_classification", use_return_type_classification_)) {
    RCLCPP_DEBUG(get_logger(), "Setting use return type classification to: %s.", use_return_type_classification_ ? "true" : "false");
  }
  if (get_param(p, "primary_return_types", primary_return_types_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new primary return types");
  }
  if (get_param(p, "secondary_return_types", secondary_return_types_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new secondary return types");
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent)