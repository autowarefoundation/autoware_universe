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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__ORGANIZE_KERNELS_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__ORGANIZE_KERNELS_HPP_

#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"

#include <cuda_runtime.h>

namespace autoware::cuda_pointcloud_preprocessor
{

std::size_t querySortWorkspace(
  int num_items, int num_segments, int * offsets_device, std::uint32_t * keys_in_device,
  std::uint32_t * keys_out_device, cudaStream_t & stream);

void organizeLaunch(
  const InputPointType * input_points, std::uint32_t * index_tensor, std::int32_t * ring_indexes,
  std::int32_t initial_max_rings, std::int32_t * output_max_rings,
  std::int32_t initial_max_points_per_ring, std::int32_t * output_max_points_per_ring,
  int num_points, int threads_per_block, int blocks_per_grid, cudaStream_t & stream);

void gatherLaunch(
  const InputPointType * input_points, const std::uint32_t * index_tensor,
  InputPointType * output_points, int num_rings, int max_points_per_ring,
  std::uint32_t * is_valid_point, size_t num_raw_points, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream);
}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__ORGANIZE_KERNELS_HPP_
