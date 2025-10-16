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

#include "autoware/cuda_pointcloud_preprocessor/outlier_kernels.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

namespace autoware::cuda_pointcloud_preprocessor
{

__global__ void ringOutlierFilterKernel(
  const InputPointType * points, std::uint32_t * output_mask, int num_rings,
  int max_points_per_ring, float distance_ratio, float object_length_threshold_squared)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int j = idx / max_points_per_ring;
  int i = idx % max_points_per_ring;

  if (j >= num_rings || i >= max_points_per_ring) {
    return;
  }

  const int window_size = 5;
  int min_i = max(i - window_size, 0);
  int max_i = min(i + window_size, max_points_per_ring);

  int left_idx = min_i;
  int right_idx = min_i + 1;

  for (int k = min_i; k < max_i - 1; k++) {
    const InputPointType & left_point = points[j * max_points_per_ring + k];
    const InputPointType & right_point = points[j * max_points_per_ring + k + 1];

    // Find biggest walk that passes through i
    float azimuth_diff = right_point.azimuth - left_point.azimuth;
    azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 2 * M_PI : azimuth_diff;

    if (
      max(left_point.distance, right_point.distance) <
        min(left_point.distance, right_point.distance) * distance_ratio &&
      azimuth_diff < 1.f * M_PI / 180.f) {
      // Determined to be included in the same walk
      right_idx++;
    } else if (k >= i) {
      break;
    } else {
      left_idx = k + 1;
      right_idx = k + 2;  // this is safe since we break if k >= i
    }
  }

  const InputPointType & left_point = points[j * max_points_per_ring + left_idx];
  const InputPointType & right_point = points[j * max_points_per_ring + right_idx - 1];
  const float x = left_point.x - right_point.x;
  const float y = left_point.y - right_point.y;
  const float z = left_point.z - right_point.z;

  output_mask[j * max_points_per_ring + i] =
    static_cast<std::uint32_t>((x * x + y * y + z * z >= object_length_threshold_squared));
}

void ringOutlierFilterLaunch(
  const InputPointType * points, std::uint32_t * output_mask, int num_rings,
  int max_points_per_ring, float distance_ratio, float object_length_threshold_squared,
  int threads_per_block, int blocks_per_grid, cudaStream_t & stream)
{
  ringOutlierFilterKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    points, output_mask, num_rings, max_points_per_ring, distance_ratio,
    object_length_threshold_squared);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

}  // namespace autoware::cuda_pointcloud_preprocessor
