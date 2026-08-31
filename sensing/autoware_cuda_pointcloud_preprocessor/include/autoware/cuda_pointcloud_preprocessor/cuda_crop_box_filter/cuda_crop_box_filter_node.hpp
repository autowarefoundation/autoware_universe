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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CROP_BOX_FILTER__CUDA_CROP_BOX_FILTER_NODE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CROP_BOX_FILTER__CUDA_CROP_BOX_FILTER_NODE_HPP_

#include "autoware/cuda_pointcloud_preprocessor/cuda_crop_box_filter/cuda_crop_box_filter.hpp"

#include <cuda_blackboard/cuda_adaptation.hpp>
#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autoware::cuda_pointcloud_preprocessor
{

/// GPU crop box, shaped to drop into the localization preprocessing chain.
///
/// Reads and writes `cuda_blackboard::CudaPointCloud2`, so it chains
/// GPU-resident with `CudaVoxelGridDownsampleFilterNode` downstream. The
/// blackboard publisher also carries a plain `PointCloud2` for anything that has
/// not been converted, which is how the chain still feeds a subscriber outside
/// this process.
class CudaCropBoxFilterNode : public rclcpp::Node
{
public:
  explicit CudaCropBoxFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void pointcloudCallback(const cuda_blackboard::CudaPointCloud2::ConstSharedPtr msg);

  std::string expected_frame_;
  bool warned_about_frame_{false};
  bool warned_about_layout_{false};

  std::unique_ptr<CudaCropBoxFilter> filter_;
  std::shared_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>> sub_;
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>> pub_;
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CROP_BOX_FILTER__CUDA_CROP_BOX_FILTER_NODE_HPP_
