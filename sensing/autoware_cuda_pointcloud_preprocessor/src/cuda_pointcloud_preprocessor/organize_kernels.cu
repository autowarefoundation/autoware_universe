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

#include "autoware/cuda_pointcloud_preprocessor/organize_kernels.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <cub/cub.cuh>

namespace autoware::cuda_pointcloud_preprocessor
{

__global__ void organizeKernel(
  const InputPointType * __restrict__ input_points, std::uint32_t * index_tensor,
  std::int32_t * ring_indexes, std::int32_t initial_max_rings, std::int32_t * output_max_rings,
  std::int32_t initial_max_points_per_ring, std::int32_t * output_max_points_per_ring,
  int num_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) {
    return;
  }

  auto ring = input_points[idx].channel;

  if (ring >= initial_max_rings) {
    atomicMax(output_max_rings, ring);
    return;
  }

  int next_offset = atomicAdd(&ring_indexes[ring], 1);

  if (next_offset >= initial_max_points_per_ring) {
    atomicMax(output_max_points_per_ring, next_offset);
    return;
  }

  index_tensor[ring * initial_max_points_per_ring + next_offset] = idx;
}

__global__ void gatherKernel(
  const InputPointType * __restrict__ input_points, const std::uint32_t * __restrict__ index_tensor,
  InputPointType * __restrict__ output_points, int num_rings, int max_points_per_ring,
  std::uint32_t * __restrict__ is_valid_point, size_t num_raw_points)
{
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_rings * max_points_per_ring) {
    return;
  }

  const int ring = idx / max_points_per_ring;
  const int point = idx % max_points_per_ring;

  const std::uint32_t input_idx = index_tensor[ring * max_points_per_ring + point];

  if (input_idx < num_raw_points) {
    output_points[ring * max_points_per_ring + point] = input_points[input_idx];
    is_valid_point[idx] = true;
  } else {
    output_points[ring * max_points_per_ring + point].distance = 0.0f;
    is_valid_point[idx] = false;
  }
}

std::size_t querySortWorkspace(
  int num_items, int num_segments, int * offsets_device, std::uint32_t * keys_in_device,
  std::uint32_t * keys_out_device, cudaStream_t & stream)
{
  // Determine temporary device storage requirements
  void * temp_storage = nullptr;
  size_t temp_storage_bytes = 0;

  // Same as the default SortKeys arguments. Needed because the `stream` argument comes after them,
  // so they have to be handed to the function as well.
  int begin_bit = 0;
  int end_bit = sizeof(std::uint32_t) * 8;

  CHECK_CUDA_ERROR(
    cub::DeviceSegmentedRadixSort::SortKeys(
      temp_storage, temp_storage_bytes, keys_in_device, keys_out_device, num_items, num_segments,
      offsets_device, offsets_device + 1, begin_bit, end_bit, stream));

  return temp_storage_bytes;
}

void organizeLaunch(
  const InputPointType * input_points, std::uint32_t * index_tensor, std::int32_t * ring_indexes,
  std::int32_t initial_max_rings, std::int32_t * output_max_rings,
  std::int32_t initial_max_points_per_ring, std::int32_t * output_max_points_per_ring,
  int num_points, int threads_per_block, int blocks_per_grid, cudaStream_t & stream)
{
  organizeKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, index_tensor, ring_indexes, initial_max_rings, output_max_rings,
    initial_max_points_per_ring, output_max_points_per_ring, num_points);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

void gatherLaunch(
  const InputPointType * input_points, const std::uint32_t * index_tensor,
  InputPointType * output_points, int num_rings, int max_points_per_ring,
  std::uint32_t * is_valid_point, size_t num_raw_points, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream)
{
  gatherKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, index_tensor, output_points, num_rings, max_points_per_ring, is_valid_point,
    num_raw_points);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

}  // namespace autoware::cuda_pointcloud_preprocessor
