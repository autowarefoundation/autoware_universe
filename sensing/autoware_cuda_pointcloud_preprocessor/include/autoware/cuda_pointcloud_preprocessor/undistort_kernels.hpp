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
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <cuda_runtime.h>
#include <thrust/device_vector.h>

#include <deque>

namespace autoware::cuda_pointcloud_preprocessor
{
void undistort2DLaunch(
  InputPointType * input_points, int num_points, TwistStruct2D * twist_structs, int num_twists,
  std::uint8_t * output_mismatch_mask, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream);

void undistort3DLaunch(
  InputPointType * input_points, int num_points, TwistStruct3D * twist_structs, int num_twists,
  std::uint8_t * output_mismatch_mask, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream);

void setupTwist2DStructs(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
  const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue,
  const std::uint64_t pointcloud_stamp_nsec, const std::uint32_t first_point_rel_stamp_nsec,
  thrust::device_vector<TwistStruct2D> & device_twist_2d_structs, cudaStream_t & stream);

void setupTwist3DStructs(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
  const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue,
  const std::uint64_t pointcloud_stamp_nsec, const std::uint32_t first_point_rel_stamp_nsec,
  thrust::device_vector<TwistStruct3D> & device_twist_3d_structs, cudaStream_t & stream);

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__UNDISTORT_KERNELS_HPP_
