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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__OUTLIER_KERNELS_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__OUTLIER_KERNELS_HPP_

#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"

#include <cuda_runtime.h>

namespace autoware::cuda_pointcloud_preprocessor
{
void ringOutlierFilterLaunch(
  const InputPointType * points, std::uint32_t * output_mask, int num_rings,
  int max_points_per_ring, float distance_ratio, float object_length_threshold_squared,
  int threads_per_block, int blocks_per_grid, cudaStream_t & stream);

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__OUTLIER_KERNELS_HPP_
