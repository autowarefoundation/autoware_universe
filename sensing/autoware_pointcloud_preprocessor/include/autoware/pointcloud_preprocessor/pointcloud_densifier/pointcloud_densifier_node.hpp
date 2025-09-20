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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__POINTCLOUD_DENSIFIER__POINTCLOUD_DENSIFIER_NODE_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__POINTCLOUD_DENSIFIER__POINTCLOUD_DENSIFIER_NODE_HPP_

#include "autoware/pointcloud_preprocessor/filter.hpp"
#include "autoware/pointcloud_preprocessor/pointcloud_densifier/occupancy_grid.hpp"
#include "autoware/pointcloud_preprocessor/transform_info.hpp"

#include <managed_transform_buffer/managed_transform_buffer.hpp>

#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

class PointCloudDensifierNode : public Filter
{
protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  void faster_filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output,
    const TransformInfo & transform_info) override;

private:
  sensor_msgs::msg::PointCloud2::SharedPtr filterPointCloudByROI(
    const PointCloud2ConstPtr & input_cloud, const IndicesPtr & indices = nullptr);

  void transformAndMergePreviousClouds(
    const PointCloud2ConstPtr & current_msg, const OccupancyGrid & occupancy_grid,
    PointCloud2 & combined_cloud);

  void storeCurrentCloud(const sensor_msgs::msg::PointCloud2::SharedPtr & filtered_cloud);

  bool isValidTransform(const Eigen::Matrix4d & transform) const;

  struct DensifierParam
  {
    int num_previous_frames{1};
    double x_min{80.0};
    double x_max{200.0};
    double y_min{-20.0};
    double y_max{20.0};
    double grid_resolution{0.3};
  } param_;

  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;

  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> previous_pointclouds_;

  std::shared_ptr<managed_transform_buffer::ManagedTransformBuffer> managed_tf_buffer_;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit PointCloudDensifierNode(const rclcpp::NodeOptions & options);
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__POINTCLOUD_DENSIFIER__POINTCLOUD_DENSIFIER_NODE_HPP_
