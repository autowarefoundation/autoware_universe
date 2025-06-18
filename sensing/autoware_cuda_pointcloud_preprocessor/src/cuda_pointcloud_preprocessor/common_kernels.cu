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

#include "autoware/cuda_pointcloud_preprocessor/common_kernels.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"

namespace autoware::cuda_pointcloud_preprocessor
{
__global__ void transformPointsKernel(
  const InputPointType * __restrict__ input_points, InputPointType * output_points, int num_points,
  TransformStruct transform)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    output_points[idx] = input_points[idx];

    const float x = input_points[idx].x;
    const float y = input_points[idx].y;
    const float z = input_points[idx].z;

    output_points[idx].x = transform.m11 * x + transform.m12 * y + transform.m13 * z + transform.x;
    output_points[idx].y = transform.m21 * x + transform.m22 * y + transform.m23 * z + transform.y;
    output_points[idx].z = transform.m31 * x + transform.m32 * y + transform.m33 * z + transform.z;
  }
}

__global__ void cropBoxKernel(
  InputPointType * __restrict__ d_points, std::uint32_t * __restrict__ output_crop_mask,
  std::uint8_t * __restrict__ output_nan_mask, int num_points,
  const CropBoxParameters * __restrict__ crop_box_parameters_ptr, int num_crop_boxes)
{
  for (int idx = blockIdx.x * blockDim.x + threadIdx.x; idx < num_points;
       idx += blockDim.x * gridDim.x) {
    if (d_points[idx].distance == 0.0f) {
      continue;
    }
    const float x = d_points[idx].x;
    const float y = d_points[idx].y;
    const float z = d_points[idx].z;

    if (!isfinite(x) || !isfinite(y) || !isfinite(z)) {
      output_nan_mask[idx] = 1;
      continue;
    }

    std::uint32_t passed_crop_box_mask = 1;

    for (int i = 0; i < num_crop_boxes; i++) {
      const CropBoxParameters & crop_box_parameters = crop_box_parameters_ptr[i];
      const float & min_x = crop_box_parameters.min_x;
      const float & min_y = crop_box_parameters.min_y;
      const float & min_z = crop_box_parameters.min_z;
      const float & max_x = crop_box_parameters.max_x;
      const float & max_y = crop_box_parameters.max_y;
      const float & max_z = crop_box_parameters.max_z;
      passed_crop_box_mask &=
        (x <= min_x || x >= max_x) || (y <= min_y || y >= max_y) || (z <= min_z || z >= max_z);
    }

    output_crop_mask[idx] = passed_crop_box_mask;
  }
}

__global__ void combineMasksKernel(
  const std::uint32_t * __restrict__ mask1, const std::uint32_t * __restrict__ mask2,
  int num_points, std::uint32_t * __restrict__ output_mask)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    output_mask[idx] = mask1[idx] & mask2[idx];
  }
}

__global__ void extractPointsKernel(
  InputPointType * __restrict__ input_points, std::uint32_t * __restrict__ masks,
  std::uint32_t * __restrict__ indices, int num_points,
  OutputPointType * __restrict__ output_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points && masks[idx] == 1) {
    InputPointType & input_point = input_points[idx];
    OutputPointType & output_point = output_points[indices[idx] - 1];
    output_point.x = input_point.x;
    output_point.y = input_point.y;
    output_point.z = input_point.z;
    output_point.intensity = input_point.intensity;
    output_point.return_type = input_point.return_type;
    output_point.channel = input_point.channel;
  }
}

void transformPointsLaunch(
  const InputPointType * input_points, InputPointType * output_points, int num_points,
  TransformStruct transform, int threads_per_block, int blocks_per_grid, cudaStream_t & stream)
{
  transformPointsKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, output_points, num_points, transform);
}

void cropBoxLaunch(
  InputPointType * d_points, std::uint32_t * output_crop_mask, std::uint8_t * output_nan_mask,
  int num_points, const CropBoxParameters * crop_box_parameters_ptr, int num_crop_boxes,
  int threads_per_block, int blocks_per_grid, cudaStream_t & stream)
{
  cropBoxKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    d_points, output_crop_mask, output_nan_mask, num_points, crop_box_parameters_ptr,
    num_crop_boxes);
}

void combineMasksLaunch(
  const std::uint32_t * mask1, const std::uint32_t * mask2, int num_points,
  std::uint32_t * output_mask, int threads_per_block, int blocks_per_grid, cudaStream_t & stream)
{
  combineMasksKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    mask1, mask2, num_points, output_mask);
}

void extractPointsLaunch(
  InputPointType * input_points, std::uint32_t * masks, std::uint32_t * indices, int num_points,
  OutputPointType * output_points, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream)
{
  extractPointsKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, masks, indices, num_points, output_points);
}

}  // namespace autoware::cuda_pointcloud_preprocessor
