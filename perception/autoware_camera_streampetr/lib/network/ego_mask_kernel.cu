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

#include "autoware/camera_streampetr/network/camera_ego_mask.hpp"
#include "autoware/camera_streampetr/utils.hpp"

#include <cstdint>

namespace autoware::camera_streampetr
{

__global__ void apply_ego_mask_kernel(
  std::uint8_t * __restrict__ image, const std::uint8_t * __restrict__ mask, int height, int width,
  std::uint8_t fill_r, std::uint8_t fill_g, std::uint8_t fill_b)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= width || y >= height) {
    return;
  }

  const int mask_idx = y * width + x;
  if (mask[mask_idx] == 0) {
    return;
  }

  const int img_idx = mask_idx * 3;
  image[img_idx] = fill_r;
  image[img_idx + 1] = fill_g;
  image[img_idx + 2] = fill_b;
}

cudaError_t apply_ego_mask_launch(
  std::uint8_t * image, const std::uint8_t * mask, int height, int width, std::uint8_t fill_r,
  std::uint8_t fill_g, std::uint8_t fill_b, cudaStream_t stream)
{
  dim3 threads(16, 16);
  dim3 blocks(divup(width, threads.x), divup(height, threads.y));
  apply_ego_mask_kernel<<<blocks, threads, 0, stream>>>(
    image, mask, height, width, fill_r, fill_g, fill_b);
  return cudaGetLastError();
}

}  // namespace autoware::camera_streampetr
