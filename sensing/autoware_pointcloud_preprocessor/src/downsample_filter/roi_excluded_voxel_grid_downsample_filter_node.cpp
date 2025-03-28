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

#include "autoware/pointcloud_preprocessor/downsample_filter/roi_excluded_voxel_grid_downsample_filter_node.hpp"

#include "autoware/pointcloud_preprocessor/downsample_filter/roi_excluded_faster_voxel_grid_downsample_filter.hpp"

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

RoiExcludedVoxelGridDownsampleFilterComponent::RoiExcludedVoxelGridDownsampleFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("RoiExcludedVoxelGridDownsampleFilter", options)
{
  // Declare voxel size parameters
  voxel_size_x_ = static_cast<float>(declare_parameter("voxel_size_x", 0.3));
  voxel_size_y_ = static_cast<float>(declare_parameter("voxel_size_y", 0.3));
  voxel_size_z_ = static_cast<float>(declare_parameter("voxel_size_z", 0.1));
  // Declare ROI parameters
  roi_x_min_ = static_cast<float>(declare_parameter("roi_x_min", -10.0));
  roi_x_max_ = static_cast<float>(declare_parameter("roi_x_max", 10.0));
  roi_y_min_ = static_cast<float>(declare_parameter("roi_y_min", -10.0));
  roi_y_max_ = static_cast<float>(declare_parameter("roi_y_max", 10.0));

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RoiExcludedVoxelGridDownsampleFilterComponent::paramCallback, this, _1));
}

void RoiExcludedVoxelGridDownsampleFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  (void)input;
  (void)indices;
  (void)output;
}

void RoiExcludedVoxelGridDownsampleFilterComponent::faster_filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
  const TransformInfo & transform_info)
{
  std::scoped_lock lock(mutex_);
  if (indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }
  RoiExcludedFasterVoxelGridDownsampleFilter roi_excluded_filter;
  roi_excluded_filter.set_voxel_size(voxel_size_x_, voxel_size_y_, voxel_size_z_);
  roi_excluded_filter.set_roi(roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_);
  roi_excluded_filter.set_field_offsets(input, this->get_logger());
  roi_excluded_filter.filter(input, output, transform_info, this->get_logger());
}

rcl_interfaces::msg::SetParametersResult
RoiExcludedVoxelGridDownsampleFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::scoped_lock lock(mutex_);
  if (get_param(parameters, "voxel_size_x", voxel_size_x_)) {
    RCLCPP_DEBUG(get_logger(), "New voxel_size_x: %f", voxel_size_x_);
  }
  if (get_param(parameters, "voxel_size_y", voxel_size_y_)) {
    RCLCPP_DEBUG(get_logger(), "New voxel_size_y: %f", voxel_size_y_);
  }
  if (get_param(parameters, "voxel_size_z", voxel_size_z_)) {
    RCLCPP_DEBUG(get_logger(), "New voxel_size_z: %f", voxel_size_z_);
  }
  if (get_param(parameters, "roi_x_min", roi_x_min_)) {
    RCLCPP_DEBUG(get_logger(), "New roi_x_min: %f", roi_x_min_);
  }
  if (get_param(parameters, "roi_x_max", roi_x_max_)) {
    RCLCPP_DEBUG(get_logger(), "New roi_x_max: %f", roi_x_max_);
  }
  if (get_param(parameters, "roi_y_min", roi_y_min_)) {
    RCLCPP_DEBUG(get_logger(), "New roi_y_min: %f", roi_y_min_);
  }
  if (get_param(parameters, "roi_y_max", roi_y_max_)) {
    RCLCPP_DEBUG(get_logger(), "New roi_y_max: %f", roi_y_max_);
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::pointcloud_preprocessor::RoiExcludedVoxelGridDownsampleFilterComponent)
