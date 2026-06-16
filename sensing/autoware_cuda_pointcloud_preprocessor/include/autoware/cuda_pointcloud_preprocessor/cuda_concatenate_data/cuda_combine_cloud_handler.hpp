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

#pragma once

#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_traits.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/combine_cloud_handler.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

template <>
class CombineCloudHandler<CudaPointCloud2Traits> : public CombineCloudHandlerBase
{
protected:
  struct CudaConcatStruct
  {
    cudaStream_t stream;
    std::unique_ptr<CudaPointCloud2Traits::PointCloudMessage> cloud_ptr;
    std::size_t max_pointcloud_size{0};
  };

  std::unordered_map<std::string, CudaConcatStruct> cuda_concat_struct_map_;
  std::unique_ptr<CudaPointCloud2Traits::PointCloudMessage> concatenated_cloud_ptr_;
  std::size_t max_concat_pointcloud_size_{0};
  std::mutex mutex_;

public:
  CombineCloudHandler(
    rclcpp::Node & node, const std::vector<std::string> & input_topics, std::string output_frame,
    bool is_motion_compensated, bool publish_synchronized_pointcloud,
    bool keep_input_frame_in_synchronized_pointcloud);

  ConcatenatedCloudResult<CudaPointCloud2Traits> combine_pointclouds(
    std::unordered_map<
      std::string, typename CudaPointCloud2Traits::PointCloudMessage::ConstSharedPtr> &
      topic_to_cloud_map,
    const std::shared_ptr<CollectorInfoBase> & collector_info);

  void allocate_pointclouds() override;

<<<<<<< HEAD
  // Per-topic CUDA stream on which that topic's point cloud is consumed during concatenation. It is
  // handed to the topic's CudaBlackboardSubscriber so the blackboard can order the
  // producer/consumer and the buffer free at the stream level instead of synchronizing the whole
  // process.
=======
  /// Returns the per-topic CUDA stream used while consuming that topic's cloud.
  /// Use this stream when constructing the topic's CudaBlackboardSubscriber.
>>>>>>> 9030ff412f (sensing/cuda_pp: shorten stream docstrings and drop constructor comment)
  cudaStream_t get_stream(const std::string & topic) const
  {
    return cuda_concat_struct_map_.at(topic).stream;
  }
};

}  // namespace autoware::pointcloud_preprocessor
