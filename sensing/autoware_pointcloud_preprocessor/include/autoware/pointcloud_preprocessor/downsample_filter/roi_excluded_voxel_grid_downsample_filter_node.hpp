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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__DOWNSAMPLE_FILTER__ROI_EXCLUDED_VOXEL_GRID_DOWNSAMPLE_FILTER_NODE_HPP_  // NOLINT
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__DOWNSAMPLE_FILTER__ROI_EXCLUDED_VOXEL_GRID_DOWNSAMPLE_FILTER_NODE_HPP_  // NOLINT

#include "autoware/pointcloud_preprocessor/filter.hpp"
#include "autoware/pointcloud_preprocessor/transform_info.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
class RoiExcludedVoxelGridDownsampleFilterComponent : public Filter
{
public:
  explicit RoiExcludedVoxelGridDownsampleFilterComponent(const rclcpp::NodeOptions & options);

protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;
  void faster_filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
    const TransformInfo & transform_info) override;

private:
  float voxel_size_x_;
  float voxel_size_y_;
  float voxel_size_z_;
  float roi_x_min_;
  float roi_x_max_;
  float roi_y_min_;
  float roi_y_max_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);
};
}  // namespace autoware::pointcloud_preprocessor

// clang-format off
#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__DOWNSAMPLE_FILTER__ROI_EXCLUDED_VOXEL_GRID_DOWNSAMPLE_FILTER_NODE_HPP_  // NOLINT
// clang-format on
