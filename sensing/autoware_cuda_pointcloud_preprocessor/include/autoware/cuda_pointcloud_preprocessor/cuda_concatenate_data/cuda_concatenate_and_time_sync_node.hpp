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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_CONCATENATE_AND_TIME_SYNC_NODE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_CONCATENATE_AND_TIME_SYNC_NODE_HPP_

#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_traits.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/concatenate_and_time_sync_node.hpp"

#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autoware::cuda_pointcloud_preprocessor
{

class CudaPointCloudConcatenateDataSynchronizerComponent
: public autoware::pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponentTemplated<
    autoware::pointcloud_preprocessor::CudaPointCloud2Traits>
{
public:
  explicit CudaPointCloudConcatenateDataSynchronizerComponent(
    const rclcpp::NodeOptions & node_options)
  : autoware::pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponentTemplated<
      autoware::pointcloud_preprocessor::CudaPointCloud2Traits>(node_options)
  {
  }
  ~CudaPointCloudConcatenateDataSynchronizerComponent() override = default;
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_CONCATENATE_AND_TIME_SYNC_NODE_HPP_
