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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_TRAITS_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_TRAITS_HPP_

#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

namespace autoware::pointcloud_preprocessor
{

struct CudaPointCloud2Traits
{
  using PointCloudMessage = cuda_blackboard::CudaPointCloud2;
  using PublisherType = cuda_blackboard::CudaBlackboardPublisher<PointCloudMessage>;
  using SubscriberType = cuda_blackboard::CudaBlackboardSubscriber<PointCloudMessage>;
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_TRAITS_HPP_
