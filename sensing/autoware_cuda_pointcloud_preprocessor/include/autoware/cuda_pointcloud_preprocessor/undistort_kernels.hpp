// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__UNDISTORT_KERNELS_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__UNDISTORT_KERNELS_HPP_

#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"

#include <autoware/pointcloud_preprocessor/distortion_corrector/motion_table.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <cuda_runtime.h>
#include <thrust/device_vector.h>

#include <cstdint>
#include <deque>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{
using MotionBin2D = autoware::pointcloud_preprocessor::motion_table::Bin2D;
using MotionBin3D = autoware::pointcloud_preprocessor::motion_table::Bin3D;

/// Moves each point from the base frame at its own time into the base frame at the pointcloud's
/// header stamp. `motion_bins` holds `last_bin_index + 1` bins.
void undistort2DLaunch(
  InputPointType * input_points, int num_points, const MotionBin2D * motion_bins,
  std::uint32_t last_bin_index, std::uint8_t * output_mismatch_mask, int threads_per_block,
  int blocks_per_grid, cudaStream_t & stream);

void undistort3DLaunch(
  InputPointType * input_points, int num_points, const MotionBin3D * motion_bins,
  std::uint32_t last_bin_index, std::uint8_t * output_mismatch_mask, int threads_per_block,
  int blocks_per_grid, cudaStream_t & stream);

/// Builds the motion table of the pointcloud stamped `pointcloud_stamp_nsec` into
/// `host_motion_bins` and copies it to `device_motion_bins`, which must hold
/// `motion_table::max_bin_count` bins. Both queues must be sorted by stamp. The angular velocities
/// take precedence over those of the twists.
/// \return the index of the last bin.
std::uint32_t setupMotionBins2D(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
  const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue,
  std::uint64_t pointcloud_stamp_nsec, std::vector<MotionBin2D> & host_motion_bins,
  thrust::device_vector<MotionBin2D> & device_motion_bins, cudaStream_t & stream);

std::uint32_t setupMotionBins3D(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
  const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue,
  std::uint64_t pointcloud_stamp_nsec, std::vector<MotionBin3D> & host_motion_bins,
  thrust::device_vector<MotionBin3D> & device_motion_bins, cudaStream_t & stream);

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__UNDISTORT_KERNELS_HPP_
