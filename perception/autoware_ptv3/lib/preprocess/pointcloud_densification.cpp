// Copyright 2026 TIER IV, Inc.
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

#include "autoware/ptv3/preprocess/pointcloud_densification.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>

namespace autoware::ptv3
{

PointCloudDensification::PointCloudDensification(const DensificationParam & param) : param_(param)
{
}

void PointCloudDensification::enqueuePointCloud(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
  const Eigen::Affine3f & affine_world2current)
{
  enqueue(msg_ptr, affine_world2current);
  dequeue();
}

void PointCloudDensification::enqueue(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
  const Eigen::Affine3f & affine_world2current)
{
  affine_world2current_ = affine_world2current;
  current_timestamp_ = rclcpp::Time(msg_ptr->header.stamp).seconds();

  PointCloudWithTransform pointcloud = {msg_ptr, affine_world2current.inverse()};

  pointcloud_cache_.push_front(std::move(pointcloud));
}

void PointCloudDensification::dequeue()
{
  if (pointcloud_cache_.size() > param_.getPointcloudCacheSize()) {
    pointcloud_cache_.pop_back();
  }
}

}  // namespace autoware::ptv3
