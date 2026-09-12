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

#ifndef AUTOWARE__PTV3__PREPROCESS__SWEEP_AGGREGATOR_HPP_
#define AUTOWARE__PTV3__PREPROCESS__SWEEP_AGGREGATOR_HPP_

#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/preprocess/pointcloud_densification.hpp"
#include "autoware/ptv3/ptv3_config.hpp"

#include <Eigen/Geometry>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <cstddef>
#include <memory>

namespace autoware::ptv3
{

/// Densified network input cloud.
///
/// `points` holds one `(x, y, z, intensity, time_lag)` row per point, the
/// current frame first (`num_current_points` rows with zero time lag) followed
/// by ego-motion-compensated sweeps. `current_msg` keeps the current-frame
/// message alive for output stages that need its original point layout.
struct DensifiedCloud
{
  const float * points{nullptr};
  std::size_t num_points{0};
  std::size_t num_current_points{0};
  std::shared_ptr<const cuda_blackboard::CudaPointCloud2> current_msg{nullptr};
  CloudFormat current_format{CloudFormat::UNKNOWN};
};

/// Build the densified network input from the current frame and cached sweeps.
class SweepAggregator
{
public:
  SweepAggregator(const PTv3Config & config, cudaStream_t stream);

  /// Cache the incoming frame together with the world-to-lidar transform at its stamp.
  void enqueuePointCloud(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
    const Eigen::Affine3f & affine_world2current);

  /// Aggregate the cached frames into the densified feature buffer.
  DensifiedCloud aggregate();

private:
  PTv3Config config_;
  cudaStream_t stream_;

  std::unique_ptr<PointCloudDensification> densification_ptr_;
  autoware::cuda_utils::CudaUniquePtr<float[]> points_d_{nullptr};
  autoware::cuda_utils::CudaUniquePtr<float[]> affine_past2current_d_{nullptr};
};

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__PREPROCESS__SWEEP_AGGREGATOR_HPP_
