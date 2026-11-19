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

#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/queue_bounds.hpp"
#include "autoware/cuda_pointcloud_preprocessor/undistort_kernels.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <deque>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor
{
namespace motion_table = autoware::pointcloud_preprocessor::motion_table;

namespace
{

__device__ void undistortPoint(const MotionBin2D & bin, float remainder, InputPointType & point)
{
  motion_table::undistort(bin, remainder, point.x, point.y);
}

__device__ void undistortPoint(const MotionBin3D & bin, float remainder, InputPointType & point)
{
  motion_table::undistort(bin, remainder, point.x, point.y, point.z);
}

template <typename BinT>
__global__ void undistortKernel(
  InputPointType * __restrict__ input_points, int num_points, const BinT * __restrict__ motion_bins,
  std::uint32_t last_bin_index, std::uint8_t * __restrict__ output_mismatch_mask)
{
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) {
    return;
  }

  InputPointType & point = input_points[idx];
  float remainder = 0.f;
  const BinT & bin = motion_table::lookup(motion_bins, last_bin_index, point.time_stamp, remainder);

  output_mismatch_mask[idx] =
    static_cast<std::uint8_t>((bin.flags & motion_table::flag_mismatch) != 0U);
  undistortPoint(bin, remainder, point);
}

/// The samples of `queue` stamped at or after `pointcloud_stamp_nsec`, or its newest sample if
/// there are none. A time takes its velocity from the first sample at or after it and no point
/// precedes the header stamp, so the samples before these have no effect.
template <typename QueueT>
auto firstRelevantSample(const QueueT & queue, std::uint64_t pointcloud_stamp_nsec)
{
  const auto it = std::lower_bound(
    queue.begin(), queue.end(), pointcloud_stamp_nsec,
    [](const auto & sample, std::uint64_t stamp) { return detail::stamp_nsec(sample) < stamp; });
  return it == queue.end() && !queue.empty() ? queue.end() - 1 : it;
}

template <typename MessageT>
std::int64_t relativeStamp(const MessageT & message, std::uint64_t pointcloud_stamp_nsec)
{
  return static_cast<std::int64_t>(detail::stamp_nsec(message)) -
         static_cast<std::int64_t>(pointcloud_stamp_nsec);
}

template <typename IntegratorT>
std::uint32_t setupMotionBins(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
  const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue,
  std::uint64_t pointcloud_stamp_nsec, std::vector<typename IntegratorT::Bin> & host_motion_bins,
  thrust::device_vector<typename IntegratorT::Bin> & device_motion_bins, cudaStream_t & stream)
{
  std::vector<motion_table::TwistSample> twists;
  for (auto it = firstRelevantSample(twist_queue, pointcloud_stamp_nsec); it != twist_queue.end();
       ++it) {
    const auto & linear = it->twist.twist.linear;
    const auto & angular = it->twist.twist.angular;
    twists.push_back(
      {relativeStamp(*it, pointcloud_stamp_nsec),
       Eigen::Vector3d(linear.x, linear.y, linear.z).cast<float>(),
       Eigen::Vector3d(angular.x, angular.y, angular.z).cast<float>()});
  }

  std::vector<motion_table::AngularVelocitySample> angular_velocities;
  for (auto it = firstRelevantSample(angular_velocity_queue, pointcloud_stamp_nsec);
       it != angular_velocity_queue.end(); ++it) {
    angular_velocities.push_back(
      {relativeStamp(*it, pointcloud_stamp_nsec),
       Eigen::Vector3d(it->vector.x, it->vector.y, it->vector.z).cast<float>()});
  }

  motion_table::build<IntegratorT>(
    twists, angular_velocities, !angular_velocities.empty(), host_motion_bins);

  assert(host_motion_bins.size() <= device_motion_bins.size());
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    thrust::raw_pointer_cast(device_motion_bins.data()), host_motion_bins.data(),
    host_motion_bins.size() * sizeof(typename IntegratorT::Bin), cudaMemcpyHostToDevice, stream));
  return static_cast<std::uint32_t>(host_motion_bins.size() - 1);
}

}  // namespace

void undistort2DLaunch(
  InputPointType * input_points, int num_points, const MotionBin2D * motion_bins,
  std::uint32_t last_bin_index, std::uint8_t * output_mismatch_mask, int threads_per_block,
  int blocks_per_grid, cudaStream_t & stream)
{
  undistortKernel<MotionBin2D><<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, num_points, motion_bins, last_bin_index, output_mismatch_mask);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

void undistort3DLaunch(
  InputPointType * input_points, int num_points, const MotionBin3D * motion_bins,
  std::uint32_t last_bin_index, std::uint8_t * output_mismatch_mask, int threads_per_block,
  int blocks_per_grid, cudaStream_t & stream)
{
  undistortKernel<MotionBin3D><<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, num_points, motion_bins, last_bin_index, output_mismatch_mask);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

std::uint32_t setupMotionBins2D(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
  const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue,
  std::uint64_t pointcloud_stamp_nsec, std::vector<MotionBin2D> & host_motion_bins,
  thrust::device_vector<MotionBin2D> & device_motion_bins, cudaStream_t & stream)
{
  return setupMotionBins<motion_table::Integrator2D>(
    twist_queue, angular_velocity_queue, pointcloud_stamp_nsec, host_motion_bins,
    device_motion_bins, stream);
}

std::uint32_t setupMotionBins3D(
  const std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> & twist_queue,
  const std::deque<geometry_msgs::msg::Vector3Stamped> & angular_velocity_queue,
  std::uint64_t pointcloud_stamp_nsec, std::vector<MotionBin3D> & host_motion_bins,
  thrust::device_vector<MotionBin3D> & device_motion_bins, cudaStream_t & stream)
{
  return setupMotionBins<motion_table::Integrator3D>(
    twist_queue, angular_velocity_queue, pointcloud_stamp_nsec, host_motion_bins,
    device_motion_bins, stream);
}

}  // namespace autoware::cuda_pointcloud_preprocessor
