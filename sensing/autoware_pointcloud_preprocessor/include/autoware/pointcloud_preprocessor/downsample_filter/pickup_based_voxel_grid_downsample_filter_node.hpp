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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__DOWNSAMPLE_FILTER__PICKUP_BASED_VOXEL_GRID_DOWNSAMPLE_FILTER_NODE_HPP_  // NOLINT
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__DOWNSAMPLE_FILTER__PICKUP_BASED_VOXEL_GRID_DOWNSAMPLE_FILTER_NODE_HPP_  // NOLINT

#include "autoware/pointcloud_preprocessor/filter.hpp"

#include <sensor_msgs/msg/point_cloud2.h>

#include <vector>

namespace autoware::pointcloud_preprocessor
{

struct VoxelSize
{
  float x;
  float y;
  float z;
};

void downsample_with_voxel_grid(
  const sensor_msgs::msg::PointCloud2 & input, const VoxelSize & voxel_size,
  sensor_msgs::msg::PointCloud2 & output);

/**
 * @class PickupBasedVoxelGridDownsampleFilterComponent
 * @brief A filter component for downsampling point clouds using a voxel grid approach.
 *
 * This component reduces the number of points in a point cloud by grouping them into voxels
 * and picking a representative point for each voxel. It's useful for reducing computational
 * load when processing large point clouds.
 */
class PickupBasedVoxelGridDownsampleFilterComponent
: public autoware::pointcloud_preprocessor::Filter
{
protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

private:
  VoxelSize voxel_size_;  ///< The size of the voxel grid.

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

public:
  explicit PickupBasedVoxelGridDownsampleFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::pointcloud_preprocessor

// clang-format off
#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__DOWNSAMPLE_FILTER__PICKUP_BASED_VOXEL_GRID_DOWNSAMPLE_FILTER_NODE_HPP_  // NOLINT
// clang-format on
