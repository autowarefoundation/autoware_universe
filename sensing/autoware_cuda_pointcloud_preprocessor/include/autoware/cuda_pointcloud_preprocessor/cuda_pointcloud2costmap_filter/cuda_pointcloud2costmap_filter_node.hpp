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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD2COSTMAP_FILTER__CUDA_POINTCLOUD2COSTMAP_FILTER_NODE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD2COSTMAP_FILTER__CUDA_POINTCLOUD2COSTMAP_FILTER_NODE_HPP_

#include "cuda_pointcloud2costmap_filter.hpp"

#include <cuda_blackboard/cuda_adaptation.hpp>
#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>

#include <memory>
#include <string>

namespace autoware::cuda_pointcloud_preprocessor
{
class CudaPointcloud2CostmapFilterNode : public rclcpp::Node
{
public:
  explicit CudaPointcloud2CostmapFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void cudaPointcloudCallback(const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg);

  // CUDA sub
  std::shared_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    sub_{};

  // ROS pub for grid_map
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_costmap_{};

  std::unique_ptr<CudaPointcloud2CostmapFilter> cuda_pointcloud2costmap_filter_{};

  // Costmap parameters
  CostmapParameters costmap_params_;
  std::string costmap_frame_;
};
}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD2COSTMAP_FILTER__CUDA_POINTCLOUD2COSTMAP_FILTER_NODE_HPP_
