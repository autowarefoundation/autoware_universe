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
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PolarVoxelOutlierFilterComponent::paramCallback, this, _1));
}

void PolarVoxelOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  if (indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }

  // Check if the input point cloud has PointXYZIRCAEDT layout (with pre-computed polar coordinates)
  if (autoware::pointcloud_preprocessor::utils::is_data_layout_compatible_with_point_xyzircaedt(*input)) {
    RCLCPP_DEBUG(get_logger(), "Using PointXYZIRCAEDT format with pre-computed polar coordinates");
    filterPointXYZIRCAEDT(input, indices, output);
  } else {
    RCLCPP_DEBUG(get_logger(), "Using PointXYZ format, computing polar coordinates");
    filterPointXYZ(input, indices, output);
  }
}

void PolarVoxelOutlierFilterComponent::filterPointXYZ(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  (void)indices;  // Suppress unused parameter warning
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);

  // Map to store point counts per polar voxel
  std::map<PolarVoxelIndex, std::vector<size_t>> voxel_point_map;

  // First pass: count points in each polar voxel
  for (size_t i = 0; i < pcl_input->points.size(); ++i) {
    const auto & point = pcl_input->points[i];
    
    // Skip invalid points
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    double radius, azimuth, elevation;
    cartesianToPolar(point.x, point.y, point.z, radius, azimuth, elevation);

    // Skip points outside radius range
    if (radius < min_radius_ || radius > max_radius_) {
      continue;
    }

    PolarVoxelIndex voxel_idx = cartesianToPolarVoxel(point.x, point.y, point.z);
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

void PolarVoxelOutlierFilterComponent::filterPointXYZIRCAEDT(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  (void)indices;  // Suppress unused parameter warning
  
  // Copy input to output initially, then we'll filter it
  output = *input;
  
  // Structure to hold both first and second returns for each voxel
  struct VoxelPoints {
    std::vector<size_t> first_returns;
    std::vector<size_t> second_returns;
  };
  
  // Map to store point counts per polar voxel with separate vectors for first and second returns
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
    double radius = static_cast<double>(*iter_distance);
    double azimuth = static_cast<double>(*iter_azimuth);
    double elevation = static_cast<double>(*iter_elevation);
    uint8_t return_type = *iter_return_type;

    // Skip invalid points
    if (!std::isfinite(radius) || !std::isfinite(azimuth) || !std::isfinite(elevation)) {
      continue;
    }

    // Skip points outside radius range
    if (radius < min_radius_ || radius > max_radius_) {
      continue;
    }

    PolarVoxelIndex voxel_idx = polarToPolarVoxel(radius, azimuth, elevation);
    
    // Add point to appropriate vector based on return type
    if (return_type == 1) {  // First return
      voxel_point_map[voxel_idx].first_returns.push_back(point_idx);
    } else {  // Second or subsequent returns
      voxel_point_map[voxel_idx].second_returns.push_back(point_idx);
    }
  }

  // Collect valid point indices
  std::vector<bool> valid_points(input->width * input->height, false);
  
  for (const auto & voxel_entry : voxel_point_map) {
    const auto & first_returns = voxel_entry.second.first_returns;
    const auto & second_returns = voxel_entry.second.second_returns;
    
    // Check if either first or second returns meet the threshold
    if (static_cast<int>(first_returns.size()) >= voxel_points_threshold_ ||
        static_cast<int>(second_returns.size()) >= voxel_points_threshold_) {
      // Add all points from both vectors
      for (size_t idx : first_returns) {
        valid_points[idx] = true;
      }
      for (size_t idx : second_returns) {
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
}

void PolarVoxelOutlierFilterComponent::cartesianToPolar(
  double x, double y, double z, double & radius, double & azimuth, double & elevation) const
{
  radius = std::sqrt(x * x + y * y + z * z);
  azimuth = std::atan2(y, x);
  elevation = std::atan2(z, std::sqrt(x * x + y * y));
}

PolarVoxelOutlierFilterComponent::PolarVoxelIndex 
PolarVoxelOutlierFilterComponent::cartesianToPolarVoxel(double x, double y, double z) const
{
  double radius, azimuth, elevation;
  cartesianToPolar(x, y, z, radius, azimuth, elevation);
  return polarToPolarVoxel(radius, azimuth, elevation);
}

PolarVoxelOutlierFilterComponent::PolarVoxelIndex 
PolarVoxelOutlierFilterComponent::polarToPolarVoxel(double radius, double azimuth, double elevation) const
{
  PolarVoxelIndex voxel_idx;
  voxel_idx.radius_idx = static_cast<int>(std::floor(radius / radius_resolution_));
  voxel_idx.azimuth_idx = static_cast<int>(std::floor(azimuth / azimuth_resolution_));
  voxel_idx.elevation_idx = static_cast<int>(std::floor(elevation / elevation_resolution_));
  return voxel_idx;
}

rcl_interfaces::msg::SetParametersResult PolarVoxelOutlierFilterComponent::paramCallback(
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

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent) 