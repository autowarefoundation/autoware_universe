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
// -------------------------------------------------------------------------
// Fused Kernel: Anti-Aliased Resize -> Crop -> Normalize -> CHW Layout
// -------------------------------------------------------------------------
// This kernel mimics PIL's resize (Bilinear/Triangle filter with adaptive support).
// For downscaling, it expands the kernel window to cover all contributing pixels
// (Anti-aliasing). For upscaling, it acts as standard bilinear interpolation.
__global__ void resizeAndExtractRoi_kernel(
  const std::uint8_t * __restrict__ input_img, float * __restrict__ output_img,
  int camera_offset,                 // Offset in output buffer (for multi-camera batching)
  int in_h, int in_w,                // Input dimensions
  int resize_h, int resize_w,        // Target resize dimensions (before cropping)
  int roi_h, int roi_w,              // Output ROI dimensions
  int roi_y_start, int roi_x_start,  // Top-left of ROI in the resized coordinate space
  const float * __restrict__ mean,   // Device pointer to 3 floats
  const float * __restrict__ std     // Device pointer to 3 floats
)
{
  // 1. Calculate thread target pixel in the ROI (Output Image)
  int out_x = blockIdx.x * blockDim.x + threadIdx.x;
  int out_y = blockIdx.y * blockDim.y + threadIdx.y;

  if (out_x >= roi_w || out_y >= roi_h) return;

  // 2. Map ROI pixel to the virtual Resized Image coordinates
  //    (The image that *would* exist if we resized the whole thing)
  float resized_x = static_cast<float>(out_x + roi_x_start);
  float resized_y = static_cast<float>(out_y + roi_y_start);

  // 3. Calculate Scale Factors (Input / Resized)
  //    scale > 1.0 means downsampling
  float scale_x = static_cast<float>(in_w) / static_cast<float>(resize_w);
  float scale_y = static_cast<float>(in_h) / static_cast<float>(resize_h);

  // 4. Map to Input Image Center Point
  //    Standard alignment: (x + 0.5) * scale - 0.5
  float center_x = (resized_x + 0.5f) * scale_x - 0.5f;
  float center_y = (resized_y + 0.5f) * scale_y - 0.5f;

  // 5. Determine Support Size (Kernel Radius)
  //    For downsampling, support scales with the downsampling factor (Anti-aliasing).
  //    For upsampling (scale < 1), support is 1.0 (Standard Bilinear).
  float support_x = (scale_x > 1.0f) ? scale_x : 1.0f;
  float support_y = (scale_y > 1.0f) ? scale_y : 1.0f;

  // 6. Define Sampling Bounds in Input
  //    We loop over pixels within [center - support, center + support]
  int x_min = static_cast<int>(ceilf(center_x - support_x));
  int x_max = static_cast<int>(floorf(center_x + support_x));
  int y_min = static_cast<int>(ceilf(center_y - support_y));
  int y_max = static_cast<int>(floorf(center_y + support_y));

  // Clamp to input image boundaries
  x_min = max(0, x_min);
  x_max = min(in_w - 1, x_max);
  y_min = max(0, y_min);
  y_max = min(in_h - 1, y_max);

  // Accumulators
  float sum_r = 0.0f;
  float sum_g = 0.0f;
  float sum_b = 0.0f;
  float sum_weight = 0.0f;

  // 7. Convolution Loop (Triangle/Bilinear Filter)
  // TODO(SamratThapa120): If necessary, optimize with im2col, separate horizontal and vertical
  // filtering, etc.
  for (int y = y_min; y <= y_max; ++y) {
    // Y-weight: Triangle filter (1 - distance / support)
    float dy = (center_y - y);
    float wy = max(0.0f, 1.0f - abs(dy) / support_y);

    // Optimization: Pre-calculate Y-offset index
    int row_offset = y * in_w;

    for (int x = x_min; x <= x_max; ++x) {
      // X-weight
      float dx = (center_x - x);
      float wx = max(0.0f, 1.0f - abs(dx) / support_x);

      // Combined weight
      float w = wx * wy;

      // Skip negligible weights
      if (w > 0.0f) {
        int idx = (row_offset + x) * 3;
        sum_r += input_img[idx] * w;
        sum_g += input_img[idx + 1] * w;
        sum_b += input_img[idx + 2] * w;
        sum_weight += w;
      }
    }
  }

  // 8. Normalize Weights and Write Output
  //    Avoid division by zero if weight sum is tiny (shouldn't happen inside valid ROI)
  if (sum_weight > 0.0f) {
    sum_r /= sum_weight;
    sum_g /= sum_weight;
    sum_b /= sum_weight;
  }

  // 9. Normalize (Mean/Std) and Write to Output (Planar CHW)
  //    Output layout: [Batch/Camera, Channel, Height, Width]
  int area = roi_h * roi_w;
  int out_idx = out_y * roi_w + out_x;

  // Channel 0 (R)
  output_img[camera_offset + (0 * area + out_idx)] = (sum_r - mean[0]) / std[0];
  // Channel 1 (G)
  output_img[camera_offset + (1 * area + out_idx)] = (sum_g - mean[1]) / std[1];
  // Channel 2 (B)
  output_img[camera_offset + (2 * area + out_idx)] = (sum_b - mean[2]) / std[2];
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

__global__ void remap_kernel(
  const std::uint8_t * __restrict__ input_img, std::uint8_t * __restrict__ output_img,
  int output_height, int output_width,  // Output (destination) image dimensions
  int input_height, int input_width,    // Input (source) image dimensions
  const float * __restrict__ map_x, const float * __restrict__ map_y)
{
  // Calculate the global thread indices for output image
  int x = blockIdx.x * blockDim.x + threadIdx.x;  // Width index in output
  int y = blockIdx.y * blockDim.y + threadIdx.y;  // Height index in output

  // Check if the thread corresponds to a valid pixel in output
  if (x >= output_width || y >= output_height) return;

  // Get the mapping coordinates for this output pixel.
  // Skip the first y rows with (y * output_width) and then skip the first x columns by adding x
  int map_idx = y * output_width + x;
  float src_x = map_x[map_idx];
  float src_y = map_y[map_idx];

  // Check if the mapped coordinates are valid in the input image
  // Skip the first y rows with (y * output_width) and then skip the first x columns by adding x.
  // Then skip the first 3 channels by multiplying by 3
  int out_idx = (y * output_width + x) * 3;
  if (src_x < 0 || src_y < 0 || src_x >= input_width || src_y >= input_height) {
    // Set to black for out-of-bounds pixels
    output_img[out_idx] = 0;
    output_img[out_idx + 1] = 0;
    output_img[out_idx + 2] = 0;
    return;
  }

  // Bilinear interpolation in unit square (0,0), (0,1), (1,0), (1,1)
  int x0 = static_cast<int>(floorf(src_x));
  int y0 = static_cast<int>(floorf(src_y));
  int x1 = x0 + 1;
  int y1 = y0 + 1;

  float dx = src_x - x0;
  float dy = src_y - y0;

  // Calculate interpolation weights
  float w00 = (1.0f - dx) * (1.0f - dy);
  float w01 = (1.0f - dx) * dy;
  float w10 = dx * (1.0f - dy);
  float w11 = dx * dy;

// Process each color channel
#pragma unroll
  for (int c = 0; c < 3; ++c) {
    float v00 = 0.0f, v01 = 0.0f, v10 = 0.0f, v11 = 0.0f;

    // Get pixel values with boundary checks
    if (x0 >= 0 && x0 < input_width && y0 >= 0 && y0 < input_height)
      v00 = static_cast<float>(input_img[(y0 * input_width + x0) * 3 + c]);
    if (x0 >= 0 && x0 < input_width && y1 >= 0 && y1 < input_height)
      v01 = static_cast<float>(input_img[(y1 * input_width + x0) * 3 + c]);
    if (x1 >= 0 && x1 < input_width && y0 >= 0 && y0 < input_height)
      v10 = static_cast<float>(input_img[(y0 * input_width + x1) * 3 + c]);
    if (x1 >= 0 && x1 < input_width && y1 >= 0 && y1 < input_height)
      v11 = static_cast<float>(input_img[(y1 * input_width + x1) * 3 + c]);

    // Interpolate and store the result
    float interpolated_value = w00 * v00 + w01 * v01 + w10 * v10 + w11 * v11;
    // Clamp the interpolated value to [0, 255] range
    interpolated_value = fmaxf(0.0f, fminf(255.0f, interpolated_value));
    output_img[out_idx + c] = static_cast<std::uint8_t>(interpolated_value);
  }
}

cudaError_t remap_launch(
  const std::uint8_t * input_img, std::uint8_t * output_img, int output_height,
  int output_width,                   // Output (destination) image dimensions
  int input_height, int input_width,  // Input (source) image dimensions
  const float * map_x, const float * map_y, cudaStream_t stream)
{
  // Define the block and grid dimensions based on output size
  dim3 threads(16, 16);
  dim3 blocks(divup(output_width, threads.x), divup(output_height, threads.y));

  // Launch the kernel
  remap_kernel<<<blocks, threads, 0, stream>>>(
    input_img, output_img, output_height, output_width, input_height, input_width, map_x, map_y);

  // Check for errors
  return cudaGetLastError();
}

}  // namespace autoware::camera_streampetr
