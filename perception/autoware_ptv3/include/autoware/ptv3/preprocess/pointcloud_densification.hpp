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

#ifndef AUTOWARE__PTV3__PREPROCESS__POINTCLOUD_DENSIFICATION_HPP_
#define AUTOWARE__PTV3__PREPROCESS__POINTCLOUD_DENSIFICATION_HPP_

#include <Eigen/Geometry>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <cstddef>
#include <list>
#include <memory>
#include <string>

namespace autoware::ptv3
{

class DensificationParam
{
public:
  DensificationParam(const std::string & world_frame_id, const unsigned int num_past_frames)
  : world_frame_id_(world_frame_id), pointcloud_cache_size_(num_past_frames + /*current frame*/ 1)
  {
  }

  std::string getWorldFrameId() const { return world_frame_id_; }
  unsigned int getPointcloudCacheSize() const { return pointcloud_cache_size_; }

private:
  std::string world_frame_id_;
  unsigned int pointcloud_cache_size_{1};
};

struct PointCloudWithTransform
{
  std::shared_ptr<const cuda_blackboard::CudaPointCloud2> input_pointcloud_msg_ptr;
  Eigen::Affine3f affine_past2world;
};

/// Cache of the current lidar frame and its ego-motion history.
///
/// The cache front is always the current frame; older sweeps follow in
/// reverse-chronological order together with the rigid transform from their
/// lidar frame into the world frame.
class PointCloudDensification
{
public:
  explicit PointCloudDensification(const DensificationParam & param);

  /// Cache the incoming frame together with the world-to-lidar transform at its stamp.
  void enqueuePointCloud(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
    const Eigen::Affine3f & affine_world2current);

  double getCurrentTimestamp() const { return current_timestamp_; }
  Eigen::Affine3f getAffineWorldToCurrent() const { return affine_world2current_; }
  std::list<PointCloudWithTransform>::iterator getPointCloudCacheIter()
  {
    return pointcloud_cache_.begin();
  }
  bool isCacheEnd(std::list<PointCloudWithTransform>::iterator iter)
  {
    return iter == pointcloud_cache_.end();
  }
  std::size_t getIdx(std::list<PointCloudWithTransform>::iterator iter)
  {
    return std::distance(pointcloud_cache_.begin(), iter);
  }
  std::size_t getCacheSize()
  {
    return std::distance(pointcloud_cache_.begin(), pointcloud_cache_.end());
  }
  unsigned int getPointcloudCacheSize() const { return param_.getPointcloudCacheSize(); }

private:
  void enqueue(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
    const Eigen::Affine3f & affine);
  void dequeue();

  DensificationParam param_;
  double current_timestamp_{0.0};
  Eigen::Affine3f affine_world2current_;
  std::list<PointCloudWithTransform> pointcloud_cache_;
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PREPROCESS__POINTCLOUD_DENSIFICATION_HPP_
