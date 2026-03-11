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

#include "autoware/bevfusion/camera/camera_preprocess.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <cstddef>
#include <cstdint>
#include <iostream>

namespace autoware::bevfusion
{

CameraPreprocess::CameraPreprocess(
  cudaStream_t stream, const ImagePreProcessingParams & image_pre_processing_params)
: stream_(stream), image_pre_processing_params_(image_pre_processing_params)
{
}

CameraPreprocess::~CameraPreprocess()
{
  if (stream_ != nullptr) {
    cudaStreamDestroy(stream_);
    stream_ = nullptr;
  }
}

// -------------------------------------------------------------------------
// Fused Kernel: Anti-Aliased Resize -> Crop -> Normalize -> CHW Layout
// -------------------------------------------------------------------------
// This kernel mimics PIL's resize (Bilinear/Triangle filter with adaptive support).
// For downscaling, it expands the kernel window to cover all contributing pixels
// (Anti-aliasing). For upscaling, it acts as standard bilinear interpolation.
template <
  std::size_t FIRST_CHANNEL_OFFSET, std::size_t SECOND_CHANNEL_OFFSET,
  std::size_t LAST_CHANNEL_OFFSET>
__global__ void resizeAndExtractRoi_kernel(
  const std::uint8_t * __restrict__ input_img, std::uint8_t * __restrict__ output_img, int in_h,
  int in_w,                         // Input dimensions
  int resize_h, int resize_w,       // Target resize dimensions (before cropping)
  int roi_h, int roi_w,             // Output ROI dimensions
  int roi_y_start, int roi_x_start  // Top-left of ROI in the resized coordinate space
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
  //    float sum_r = 0.0f;
  //    float sum_g = 0.0f;
  //    float sum_b = 0.0f;
  float sum_first_channel = 0.0f;
  float sum_second_channel = 0.0f;
  float sum_third_channel = 0.0f;
  float sum_weight = 0.0f;

  // 7. Convolution Loop (Triangle/Bilinear Filter)
  // TODO(KokSeang): If necessary, optimize with im2col, separate horizontal and vertical
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
        // To save the image in CHW format, the index is calculated as (row_offset + x) * 3.
        int idx = (row_offset + x) * 3;
        // Change the channel accordingly, if flip_image_channels is set to True,
        // FIRST_CHANNEL_OFFSET will be 2 and LAST_CHANNEL_OFFSET will be 1 to swap them. Otherwise,
        // it will be 0, 1, 2 respectively.
        sum_first_channel += input_img[idx + FIRST_CHANNEL_OFFSET] * w;
        sum_second_channel += input_img[idx + SECOND_CHANNEL_OFFSET] * w;
        sum_third_channel += input_img[idx + LAST_CHANNEL_OFFSET] * w;
        // Input is BGR (from ROS bgr8), read as RGB for the model
        //  sum_r += input_img[idx + 2] * w;  // R is at BGR offset 2
        //  sum_g += input_img[idx + 1] * w;  // G is at BGR offset 1
        //  sum_b += input_img[idx] * w;      // B is at BGR offset 0
        sum_weight += w;
      }
    }
  }

  // 8. Normalize Weights and Write Output
  //    Avoid division by zero if weight sum is tiny (shouldn't happen inside valid ROI)
  if (sum_weight > 0.0f) {
    sum_first_channel /= sum_weight;
    sum_second_channel /= sum_weight;
    sum_third_channel /= sum_weight;
  }

  // 9. Normalize (Mean/Std) and Write to Output (Planar CHW)
  //    Output layout: [Batch/Camera, Channel, Height, Width]
  int area = roi_h * roi_w;
  int out_idx = out_y * roi_w + out_x;

  // Channel 0
  output_img[(0 * area + out_idx)] = sum_first_channel;
  // Channel 1
  output_img[(1 * area + out_idx)] = sum_second_channel;
  // Channel 2
  output_img[(2 * area + out_idx)] = sum_third_channel;
}

cudaError_t CameraPreprocess::resizeAndExtractRoi_launch(
  const std::uint8_t * input_img, std::uint8_t * output_img, int H,
  int W,                     // Original image dimensions
  int H2, int W2,            // Resized image dimensions
  int H3, int W3,            // ROI dimensions
  int y_start, int x_start,  // ROI top-left coordinates in resized image
  bool flip_image_channels  // Set to True if the image channels are to be flipped, for example, BGR
                            // to RGB
)
{
  // Define the block and grid dimensions
  dim3 threads(16, 16);
  dim3 blocks(divup(W3, threads.x), divup(H3, threads.y));

  // Launch the kernel
  if (flip_image_channels) {
    resizeAndExtractRoi_kernel<2, 1, 0><<<blocks, threads, 0, stream_>>>(
      input_img, output_img, H, W, H2, W2, H3, W3, y_start, x_start);
  } else {
    resizeAndExtractRoi_kernel<0, 1, 2><<<blocks, threads, 0, stream_>>>(
      input_img, output_img, H, W, H2, W2, H3, W3, y_start, x_start);
  }

  // Check for errors
  return cudaGetLastError();
}

}  // namespace autoware::bevfusion
