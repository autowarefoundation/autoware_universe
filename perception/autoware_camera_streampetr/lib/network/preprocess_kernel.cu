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
/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES.
 * All rights reserved. SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <autoware/camera_streampetr/network/preprocess.hpp>
#include <autoware/camera_streampetr/utils.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>

namespace autoware::camera_streampetr
{

__global__ void resizeAndExtractRoi_kernel(
  const std::uint8_t * __restrict__ input_img, float * __restrict__ output_img,
  int camera_offset,                // Camera offset in the input image
  int H, int W,                     // Original image dimensions (Height, Width)
  int H2, int W2,                   // Resized image dimensions (Height, Width)
  int H3, int W3,                   // ROI dimensions (Height, Width)
  int y_start, int x_start,         // ROI top-left coordinates in resized image
  const float * channel_wise_mean,  // Channel-wise mean values
  const float * channel_wise_std    // Channel-wise standard deviation values
)
{
  // Calculate the global thread indices
  int i = blockIdx.x * blockDim.x + threadIdx.x;  // Width index in output (ROI) image
  int j = blockIdx.y * blockDim.y + threadIdx.y;  // Height index in output (ROI) image

  // Check if the thread corresponds to a valid pixel in the ROI
  if (i >= W3 || j >= H3) return;

  // Compute scaling factors from original to resized image
  auto scale_y = static_cast<float>(H) / H2;
  auto scale_x = static_cast<float>(W) / W2;

  // Map output pixel (i, j) in ROI to position in resized image
  int i_resized = i + x_start;
  int j_resized = j + y_start;

  // Map position in resized image back to position in original image
  float i_original = (i_resized + 0.5f) * scale_x - 0.5f;
  float j_original = (j_resized + 0.5f) * scale_y - 0.5f;

  // Compute coordinates for bilinear interpolation
  auto i0 = static_cast<int>(floorf(i_original));
  auto j0 = static_cast<int>(floorf(j_original));
  int i1 = i0 + 1;
  int j1 = j0 + 1;

  // Compute interpolation weights
  float di = i_original - i0;
  float dj = j_original - j0;

  float w00 = (1.0f - di) * (1.0f - dj);
  float w01 = (1.0f - di) * dj;
  float w10 = di * (1.0f - dj);
  float w11 = di * dj;

  // Loop over the three color channels
  for (int c = 0; c < 3; ++c) {
    float v00 = 0.0f, v01 = 0.0f, v10 = 0.0f, v11 = 0.0f;

    // Boundary checks for each neighboring pixel
    if (i0 >= 0 && i0 < W && j0 >= 0 && j0 < H)
      v00 = static_cast<float>(input_img[(j0 * W + i0) * 3 + c]);
    if (i0 >= 0 && i0 < W && j1 >= 0 && j1 < H)
      v01 = static_cast<float>(input_img[(j1 * W + i0) * 3 + c]);
    if (i1 >= 0 && i1 < W && j0 >= 0 && j0 < H)
      v10 = static_cast<float>(input_img[(j0 * W + i1) * 3 + c]);
    if (i1 >= 0 && i1 < W && j1 >= 0 && j1 < H)
      v11 = static_cast<float>(input_img[(j1 * W + i1) * 3 + c]);

    // Compute the interpolated pixel value
    float value = w00 * v00 + w01 * v01 + w10 * v10 + w11 * v11;

    // Store the result in the output ROI image
    output_img[camera_offset + (c * H3 * W3 + j * W3 + i)] =
      (value - channel_wise_mean[c]) / channel_wise_std[c];
  }
}

cudaError_t resizeAndExtractRoi_launch(
  const std::uint8_t * input_img, float * output_img,
  int camera_offset,         // Camera offset in the input image
  int H, int W,              // Original image dimensions
  int H2, int W2,            // Resized image dimensions
  int H3, int W3,            // ROI dimensions
  int y_start, int x_start,  // ROI top-left coordinates in resized image
  const float * channel_wise_mean, const float * channel_wise_std, cudaStream_t stream)
{
  // Define the block and grid dimensions
  dim3 threads(16, 16);
  dim3 blocks(divup(W3, threads.x), divup(H3, threads.y));

  // Launch the kernel
  resizeAndExtractRoi_kernel<<<blocks, threads, 0, stream>>>(
    input_img, output_img, camera_offset, H, W, H2, W2, H3, W3, y_start, x_start, channel_wise_mean,
    channel_wise_std);

  // Check for errors
  return cudaGetLastError();
}

}  // namespace autoware::camera_streampetr
