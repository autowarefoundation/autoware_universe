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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__COMMON_KERNELS_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__COMMON_KERNELS_HPP_

#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"

#include <cuda_runtime.h>

namespace autoware::cuda_pointcloud_preprocessor
{
void transformPointsLaunch(
  const InputPointType * input_points, InputPointType * output_points, int num_points,
  TransformStruct transform, int threads_per_block, int blocks_per_grid, cudaStream_t & stream);

void cropBoxLaunch(
  InputPointType * d_points, std::uint32_t * output_crop_mask, std::uint8_t * output_nan_mask,
  int num_points, const CropBoxParameters * crop_box_parameters_ptr, int num_crop_boxes,
  int threads_per_block, int blocks_per_grid, cudaStream_t & stream);

void combineMasksLaunch(
  const std::uint32_t * mask1, const std::uint32_t * mask2, int num_points,
  std::uint32_t * output_mask, int threads_per_block, int blocks_per_grid, cudaStream_t & stream);

void extractPointsLaunch(
  InputPointType * input_points, std::uint32_t * masks, std::uint32_t * indices, int num_points,
  OutputPointType * output_points, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream);

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__COMMON_KERNELS_HPP_
