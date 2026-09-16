// Copyright 2026 NEWSLab, National Taiwan University
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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_RANDOM_DOWNSAMPLE_FILTER_NODE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_RANDOM_DOWNSAMPLE_FILTER_NODE_HPP_

#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_random_downsample_filter.hpp"

#include <cuda_blackboard/cuda_adaptation.hpp>
#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::cuda_pointcloud_preprocessor
{

/// GPU random downsample, shaped to drop into the localization preprocessing
/// chain in place of `autoware::pointcloud_preprocessor::RandomDownsampleFilterComponent`.
///
/// Reads and writes `cuda_blackboard::CudaPointCloud2`, so it chains
/// GPU-resident behind `CudaCropBoxFilterNode` without a round trip through
/// host memory. The blackboard publisher also carries a plain `PointCloud2` for
/// anything that has not been converted, which is how the chain still feeds a
/// subscriber outside this process.
class CudaRandomDownsampleFilterNode : public rclcpp::Node
{
public:
  explicit CudaRandomDownsampleFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void pointcloudCallback(const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg);

  std::unique_ptr<CudaRandomDownsampleFilter> filter_;
  std::shared_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>> sub_;
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>> pub_;
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_RANDOM_DOWNSAMPLE_FILTER_NODE_HPP_
