// Copyright 2025 TIER IV.
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

#include "networks/preprocess/multi_camera_preprocess_kernel.hpp"  // Configuration structures and declarations for CUDA kernel

namespace autoware::tensorrt_vad
{

/**
 * @brief Optimized device function for bilinear interpolation (forceinlined for performance)
 *
 * @param src_img Source image pixel data (BGR)
 * @param src_width Source image width
 * @param src_height Source image height
 * @param dst_x Output image x-coordinate
 * @param dst_y Output image y-coordinate
 * @param scale_x Scale factor in x direction (src_width / dst_width)
 * @param scale_y Scale factor in y direction (src_height / dst_height)
 * @return uint3 Resized pixel value (BGR)
 */
__device__ __forceinline__ uint3 bilinear_interpolation(
  uint8_t * src_img, int32_t src_width, int32_t src_height, int32_t dst_x, int32_t dst_y,
  float scale_x, float scale_y)
{
  // Convert output coordinates to input coordinate system
  const float src_x = (dst_x + 0.5f) * scale_x - 0.5f;
  const float src_y = (dst_y + 0.5f) * scale_y - 0.5f;

  // Clamp boundaries
  const float clamped_x = fmaxf(0.0f, fminf(src_x, src_width - 1.0f));
  const float clamped_y = fmaxf(0.0f, fminf(src_y, src_height - 1.0f));

  // Coordinates of four neighboring pixels (using GPU optimized functions)
  const int32_t x0 = __float2int_rd(clamped_x);
  const int32_t y0 = __float2int_rd(clamped_y);
  const int32_t x1 = min(x0 + 1, src_width - 1);
  const int32_t y1 = min(y0 + 1, src_height - 1);

  // Interpolation weights
  const float dx = clamped_x - x0;
  const float dy = clamped_y - y0;

  // Precompute weight products for better instruction-level parallelism
  const float w00 = (1.0f - dx) * (1.0f - dy);
  const float w10 = dx * (1.0f - dy);
  const float w01 = (1.0f - dx) * dy;
  const float w11 = dx * dy;

  // Calculate pixel indices
  const int32_t idx_00 = (y0 * src_width + x0) * 3;
  const int32_t idx_10 = (y0 * src_width + x1) * 3;
  const int32_t idx_01 = (y1 * src_width + x0) * 3;
  const int32_t idx_11 = (y1 * src_width + x1) * 3;

  uint3 result;

  // Interpolate each channel (B, G, R) using GPU optimized rounding
  result.x = __float2uint_rn(
    src_img[idx_00] * w00 + src_img[idx_10] * w10 + src_img[idx_01] * w01 + src_img[idx_11] * w11);

  result.y = __float2uint_rn(
    src_img[idx_00 + 1] * w00 + src_img[idx_10 + 1] * w10 + src_img[idx_01 + 1] * w01 +
    src_img[idx_11 + 1] * w11);

  result.z = __float2uint_rn(
    src_img[idx_00 + 2] * w00 + src_img[idx_10 + 2] * w10 + src_img[idx_01 + 2] * w01 +
    src_img[idx_11 + 2] * w11);

  return result;
}

/**
 * @brief CUDA kernel for resizing multiple camera images using bilinear interpolation
 *
 * This kernel is specialized for resize operations only, storing results in intermediate buffer.
 * Each thread handles one pixel of the output image.
 *
 * @param d_input_images Array of pointers to input images on device (BGR uint8_t format)
 * @param d_resized_images Array of pointers to output resized images on device (BGR uint8_t format)
 * @param config Configuration structure containing parameters needed for preprocessing
 */
__global__ void multi_camera_resize_kernel(
  uint8_t ** d_input_images, uint8_t ** d_resized_images, const MultiCameraPreprocessConfig config)
{
  const int32_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const int32_t y = blockIdx.y * blockDim.y + threadIdx.y;
  const int32_t camera_idx = blockIdx.z;

  // Early exit for out-of-bounds threads
  if (x >= config.output_width || y >= config.output_height || camera_idx >= config.num_cameras) {
    return;
  }

  // Get input and output images for this camera
  uint8_t * input_img = d_input_images[camera_idx];
  uint8_t * output_img = d_resized_images[camera_idx];

  // Perform bilinear interpolation (now inlined for performance)
  uint3 resized_pixel = bilinear_interpolation(
    input_img, config.input_width, config.input_height, x, y, config.scale_x, config.scale_y);

  // Calculate output index for this pixel (keep BGR format)
  const int32_t out_idx = (y * config.output_width + x) * 3;

  // Write resized pixel to output buffer (maintain BGR format for next stage)
  output_img[out_idx] = resized_pixel.x;      // B
  output_img[out_idx + 1] = resized_pixel.y;  // G
  output_img[out_idx + 2] = resized_pixel.z;  // R
}

/**
 * @brief CUDA kernel for BGR->RGB conversion, normalization, and CHW formatting
 *
 * This kernel is specialized for normalization operations, reading from intermediate buffer.
 * Each thread handles one pixel of the resized image.
 *
 * @param d_resized_images Array of pointers to resized images on device (BGR uint8_t format)
 * @param d_output Final output buffer (RGB float CHW format)
 * @param config Configuration structure containing parameters needed for preprocessing
 */
__global__ void multi_camera_normalize_kernel(
  uint8_t ** d_resized_images, float * d_output, const MultiCameraPreprocessConfig config)
{
  const int32_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const int32_t y = blockIdx.y * blockDim.y + threadIdx.y;
  const int32_t camera_idx = blockIdx.z;

  // Early exit for out-of-bounds threads
  if (x >= config.output_width || y >= config.output_height || camera_idx >= config.num_cameras) {
    return;
  }

  // Get resized image for this camera
  uint8_t * resized_img = d_resized_images[camera_idx];

  // Calculate input index (BGR format)
  const int32_t in_idx = (y * config.output_width + x) * 3;

  // Read BGR pixel values
  const uint8_t b = resized_img[in_idx];
  const uint8_t g = resized_img[in_idx + 1];
  const uint8_t r = resized_img[in_idx + 2];

  // Convert to float and normalize (BGR -> RGB conversion)
  const float norm_r = (static_cast<float>(r) - config.mean[0]) * config.inverse_std[0];
  const float norm_g = (static_cast<float>(g) - config.mean[1]) * config.inverse_std[1];
  const float norm_b = (static_cast<float>(b) - config.mean[2]) * config.inverse_std[2];

  // Calculate output indices for CHW planar format
  const int32_t single_camera_plane_size = config.output_width * config.output_height;
  const int32_t single_camera_total_size = 3 * single_camera_plane_size;
  const int32_t out_pixel_offset = y * config.output_width + x;
  const int32_t out_idx_base = camera_idx * single_camera_total_size;

  // Write to output buffer (CHW format: R, G, B planes)
  d_output[out_idx_base + 0 * single_camera_plane_size + out_pixel_offset] = norm_r;
  d_output[out_idx_base + 1 * single_camera_plane_size + out_pixel_offset] = norm_g;
  d_output[out_idx_base + 2 * single_camera_plane_size + out_pixel_offset] = norm_b;
}

/**
 * @brief Launch resize kernel to resize input images to target dimensions
 */
cudaError_t launch_multi_camera_resize_kernel(
  uint8_t ** d_input_images, uint8_t ** d_resized_images,
  const MultiCameraPreprocessConfig & config, cudaStream_t stream)
{
  // Define threads per block (e.g., 16x16 = 256 threads)
  const dim3 threads_per_block(16, 16, 1);

  // Calculate required number of blocks (3D grid: width, height, number of cameras)
  const dim3 blocks_per_grid(
    (config.output_width + threads_per_block.x - 1) / threads_per_block.x,
    (config.output_height + threads_per_block.y - 1) / threads_per_block.y, config.num_cameras);

  // Launch resize kernel
  multi_camera_resize_kernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    d_input_images, d_resized_images, config);

  // Check and return errors related to kernel launch
  return cudaGetLastError();
}

/**
 * @brief Launch normalization kernel to convert BGR->RGB, normalize, and format as CHW
 */
cudaError_t launch_multi_camera_normalize_kernel(
  uint8_t ** d_resized_images, float * d_output, const MultiCameraPreprocessConfig & config,
  cudaStream_t stream)
{
  // Define threads per block (e.g., 16x16 = 256 threads)
  const dim3 threads_per_block(16, 16, 1);

  // Calculate required number of blocks (3D grid: width, height, number of cameras)
  const dim3 blocks_per_grid(
    (config.output_width + threads_per_block.x - 1) / threads_per_block.x,
    (config.output_height + threads_per_block.y - 1) / threads_per_block.y, config.num_cameras);

  // Launch normalization kernel
  multi_camera_normalize_kernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    d_resized_images, d_output, config);

  // Check and return errors related to kernel launch
  return cudaGetLastError();
}

}  // namespace autoware::tensorrt_vad
