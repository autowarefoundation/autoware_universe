// Copyright 2020 Tier IV, Inc.
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

#ifndef VOXEL_BASED_COMPARE_MAP_FILTER__NODE_HPP_
#define VOXEL_BASED_COMPARE_MAP_FILTER__NODE_HPP_

#include "autoware/compare_map_segmentation/voxel_grid_map_loader.hpp"
#include "autoware/pointcloud_preprocessor/filter.hpp"

#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

namespace autoware::compare_map_segmentation
{
class VoxelBasedCompareMapFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

  using PointIndices = pcl_msgs::msg::PointIndices;
  using PointIndicesPtr = PointIndices::SharedPtr;
  using PointIndicesConstPtr = PointIndices::ConstSharedPtr;

protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;
  void input_indices_callback(
    const PointCloud2ConstPtr cloud, const PointIndicesConstPtr indices) override;

  bool convert_output_costly(std::unique_ptr<PointCloud2> & output) override;

private:
  // pcl::SegmentDifferences<pcl::PointXYZ> impl_;

  // interfaces
  std::unique_ptr<VoxelGridMapLoader> voxel_grid_map_loader_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // parameters
  double distance_threshold_;
  bool set_map_in_voxel_grid_;

  // diagnostics
  diagnostic_updater::Updater diagnostic_updater_;
  void checkStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit VoxelBasedCompareMapFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::compare_map_segmentation

#endif  // VOXEL_BASED_COMPARE_MAP_FILTER__NODE_HPP_
