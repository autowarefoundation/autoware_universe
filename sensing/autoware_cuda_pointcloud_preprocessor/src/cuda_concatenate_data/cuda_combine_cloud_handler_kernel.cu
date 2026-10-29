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

#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_combine_cloud_handler_kernel.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/point_types/types.hpp>

#include <cuda_runtime.h>

#include <cstdint>
#include <optional>

namespace autoware::pointcloud_preprocessor
{

using autoware::point_types::PointXYZIRC;
using autoware::point_types::PointXYZIRCT;

namespace
{

constexpr int threads_per_block = 256;

/// Writes the transformed x/y/z and the untouched i/r/c of `input` to `output`. The fields that
/// only some point formats have (time_stamp) are the caller's job.
template <typename PointT>
__device__ void transform_xyzirc(
  const PointT & input, const TransformStruct & transform, PointT & output)
{
  const float x = input.x;
  const float y = input.y;
  const float z = input.z;

  output.x = transform.m11 * x + transform.m12 * y + transform.m13 * z + transform.translation_x;
  output.y = transform.m21 * x + transform.m22 * y + transform.m23 * z + transform.translation_y;
  output.z = transform.m31 * x + transform.m32 * y + transform.m33 * z + transform.translation_z;
  output.intensity = input.intensity;
  output.return_type = input.return_type;
  output.channel = input.channel;
}

__global__ void transform_kernel(
  const PointXYZIRC * input_points, int num_points, TransformStruct transform,
  PointXYZIRC * output_points)
{
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    transform_xyzirc(input_points[idx], transform, output_points[idx]);
  }
}

__global__ void transform_kernel(
  const PointXYZIRCT * input_points, int num_points, TransformStruct transform,
  std::uint32_t time_offset_ns, PointXYZIRCT * output_points)
{
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    transform_xyzirc(input_points[idx], transform, output_points[idx]);
    output_points[idx].time_stamp = input_points[idx].time_stamp + time_offset_ns;
  }
}

/// Returns the grid size for `num_points`, or nullopt if there is nothing to launch:
/// `num_points <= 0` would result in an invalid `blocks_per_grid`, causing a
/// `cudaErrorInvalidConfiguration`.
std::optional<int> blocks_per_grid_for(int num_points)
{
  if (num_points <= 0) {
    return std::nullopt;
  }
  return (num_points + threads_per_block - 1) / threads_per_block;
}

}  // namespace

void transform_launch(
  const PointXYZIRC * input_points, int num_points, TransformStruct transform,
  PointXYZIRC * output_points, cudaStream_t & stream)
{
  const auto blocks_per_grid = blocks_per_grid_for(num_points);
  if (!blocks_per_grid) {
    return;
  }

  transform_kernel<<<*blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, num_points, transform, output_points);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

void transform_launch(
  const PointXYZIRCT * input_points, int num_points, TransformStruct transform,
  std::uint32_t time_offset_ns, PointXYZIRCT * output_points, cudaStream_t & stream)
{
  const auto blocks_per_grid = blocks_per_grid_for(num_points);
  if (!blocks_per_grid) {
    return;
  }

  transform_kernel<<<*blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, num_points, transform, time_offset_ns, output_points);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

}  // namespace autoware::pointcloud_preprocessor
