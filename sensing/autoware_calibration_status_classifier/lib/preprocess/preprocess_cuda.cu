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

#include "autoware/calibration_status_classifier/preprocess_cuda.hpp"

#include <autoware/cuda_utils/cuda_utils.hpp>

#include <cmath>
#include <cstdint>

namespace autoware::calibration_status_classifier
{
constexpr size_t r_channel = 0;
constexpr size_t g_channel = 1;
constexpr size_t b_channel = 2;
constexpr size_t depth_channel = 3;
constexpr size_t intensity_channel = 4;
constexpr size_t kernel_1d_dim = 256;
constexpr size_t kernel_2d_dim = 16;

PreprocessCuda::PreprocessCuda(
  const double max_depth, const uint32_t dilation_size, const uint32_t max_width,
  const uint32_t max_height, cudaStream_t & stream)
: max_depth_(max_depth), dilation_size_(static_cast<int>(dilation_size)), stream_(stream)
{
  depth_intensity_buffer_ = cuda_utils::make_unique<unsigned long long[]>(max_width * max_height);
};

__global__ void copy_image_kernel(
  const InputImageBGR8Type * __restrict__ input_image, const size_t width, const size_t height,
  InputImageBGR8Type * __restrict__ output_image, float * output_array)
{
  const size_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const size_t y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < width && y < height) {
    const InputImageBGR8Type px = input_image[y * width + x];
    output_image[y * width + x] = px;
    output_array[r_channel * height * width + y * width + x] = static_cast<float>(px.r) / 255.0f;
    output_array[g_channel * height * width + y * width + x] = static_cast<float>(px.g) / 255.0f;
    output_array[b_channel * height * width + y * width + x] = static_cast<float>(px.b) / 255.0f;
    output_array[depth_channel * height * width + y * width + x] = 0.0f;
    output_array[intensity_channel * height * width + y * width + x] = 0.0f;
  }
}

cudaError_t PreprocessCuda::copy_image_launch(
  const InputImageBGR8Type * input_image, const size_t width, const size_t height,
  InputImageBGR8Type * output_image, float * output_array)
{
  dim3 threads(kernel_2d_dim, kernel_2d_dim);
  dim3 blocks((width + threads.x - 1) / threads.x, (height + threads.y - 1) / threads.y);

  copy_image_kernel<<<blocks, threads, 0, stream_>>>(
    input_image, width, height, output_image, output_array);
  return cudaGetLastError();
}

/**
 * @brief Performs bilinear interpolation, mimicking OpenCV's edge handling.
 *
 * This function samples a color from the source image at a non-integer
 * coordinate (u, v) by blending the four nearest pixels. It clamps
 * coordinates to stay within the image boundaries, preventing a black
 * border artifact at the edges (similar to BORDER_REPLICATE).
 *
 * @param image Pointer to the source image data on the device.
 * @param u The floating-point x-coordinate to sample.
 * @param v The floating-point y-coordinate to sample.
 * @param width The width of the source image.
 * @param height The height of the source image.
 * @return The interpolated BGR pixel as an InputImageBGR8Type.
 */
__device__ inline InputImageBGR8Type bilinear_sample(
  const InputImageBGR8Type * __restrict__ image, const float u, const float v, const size_t width,
  const size_t height)
{
  // Check if the coordinate is completely outside the image bounds
  if (u < 0.0f || u >= static_cast<float>(width) || v < 0.0f || v >= static_cast<float>(height)) {
    return {0, 0, 0};
  }

  // Find the integer coordinates of the top-left pixel
  size_t x1 = floorf(u);
  size_t y1 = floorf(v);

  // Clamp coordinates to ensure the 2x2 grid is always valid
  if (x1 >= width - 1) x1 = width - 2;
  if (y1 >= height - 1) y1 = height - 2;

  size_t x2 = x1 + 1u;
  size_t y2 = y1 + 1u;

  // Calculate the fractional parts (weights for interpolation)
  float u_ratio = u - static_cast<float>(x1);
  float v_ratio = v - static_cast<float>(y1);
  float u_opposite = 1.0f - u_ratio;
  float v_opposite = 1.0f - v_ratio;

  // Fetch the four neighboring pixels
  const InputImageBGR8Type p11 = image[y1 * width + x1];
  const InputImageBGR8Type p21 = image[y1 * width + x2];
  const InputImageBGR8Type p12 = image[y2 * width + x1];
  const InputImageBGR8Type p22 = image[y2 * width + x2];

  // Interpolate for each channel
  float b = (p11.b * u_opposite + p21.b * u_ratio) * v_opposite +
            (p12.b * u_opposite + p22.b * u_ratio) * v_ratio;
  float g = (p11.g * u_opposite + p21.g * u_ratio) * v_opposite +
            (p12.g * u_opposite + p22.g * u_ratio) * v_ratio;
  float r = (p11.r * u_opposite + p21.r * u_ratio) * v_opposite +
            (p12.r * u_opposite + p22.r * u_ratio) * v_ratio;

  // Cast back to unsigned char for the final pixel value
  return {
    static_cast<uint8_t>(round(b)), static_cast<uint8_t>(round(g)), static_cast<uint8_t>(round(r))};
}

/**
 * @brief Main kernel to perform image undistortion.
 *
 * This kernel iterates over each pixel of the destination image, calculates
 * its corresponding location in the distorted source image, and samples the
 * color using bilinear interpolation.
 *
 * @param input_image Pointer to the original, distorted image.
 * @param dist_coeffs Pointer to the distortion coefficients array [k1, k2, p1, p2, k3, k4, k5, k6].
 * @param camera_matrix Pointer to the original camera intrinsic matrix (3x3 row-major).
 * @param projection_matrix Pointer to the new projection/camera matrix for the output image (3x4
 * row-major).
 * @param width The width of the images.
 * @param height The height of the images.
 * @param output_image Pointer to the destination image where the undistorted result is stored.
 * @param output_array Pointer to the output array for RGB, depth, and intensity data.
 */
__global__ void undistort_image_kernel(
  const InputImageBGR8Type * __restrict__ input_image, const double * __restrict__ dist_coeffs,
  const double * __restrict__ camera_matrix, const double * __restrict__ projection_matrix,
  const size_t width, const size_t height, InputImageBGR8Type * __restrict__ output_image,
  float * __restrict__ output_array)
{
  const size_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const size_t y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < width && y < height) {
    // Project destination pixel to normalized image coordinates
    const double & fx_new = projection_matrix[0];  // P[0,0]
    const double & fy_new = projection_matrix[5];  // P[1,1]
    const double & cx_new = projection_matrix[2];  // P[0,2]
    const double & cy_new = projection_matrix[6];  // P[1,2]

    const double x_proj = (static_cast<double>(x) - cx_new) / fx_new;
    const double y_proj = (static_cast<double>(y) - cy_new) / fy_new;

    // Apply forward distortion model
    const double & k1 = dist_coeffs[0];
    const double & k2 = dist_coeffs[1];
    const double & p1 = dist_coeffs[2];
    const double & p2 = dist_coeffs[3];
    const double & k3 = dist_coeffs[4];
    const double & k4 = dist_coeffs[5];
    const double & k5 = dist_coeffs[6];
    const double & k6 = dist_coeffs[7];

    const double r2 = x_proj * x_proj + y_proj * y_proj;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;

    const double radial_num = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    const double radial_den = 1.0 + k4 * r2 + k5 * r4 + k6 * r6;
    const double radial_dist = radial_num / radial_den;

    const double dx = 2.0 * p1 * x_proj * y_proj + p2 * (r2 + 2.0 * x_proj * x_proj);
    const double dy = p1 * (r2 + 2.0 * y_proj * y_proj) + 2.0 * p2 * x_proj * y_proj;

    const double x_distorted = x_proj * radial_dist + dx;
    const double y_distorted = y_proj * radial_dist + dy;

    // Project distorted point to source image pixel coordinates
    const double & fx_orig = camera_matrix[0];  // K[0,0]
    const double & fy_orig = camera_matrix[4];  // K[1,1]
    const double & cx_orig = camera_matrix[2];  // K[0,2]
    const double & cy_orig = camera_matrix[5];  // K[1,2]

    const double u = fx_orig * x_distorted + cx_orig;
    const double v = fy_orig * y_distorted + cy_orig;

    // Sample source image and write to output
    InputImageBGR8Type px = {0, 0, 0};
    if (u >= 0 && v >= 0 && u < static_cast<double>(width) && v < static_cast<double>(height)) {
      px =
        bilinear_sample(input_image, static_cast<float>(u), static_cast<float>(v), width, height);
    }

    output_image[y * width + x] = px;
    output_array[r_channel * height * width + y * width + x] = static_cast<float>(px.r) / 255.0f;
    output_array[g_channel * height * width + y * width + x] = static_cast<float>(px.g) / 255.0f;
    output_array[b_channel * height * width + y * width + x] = static_cast<float>(px.b) / 255.0f;
    output_array[depth_channel * height * width + y * width + x] = 0.0f;
    output_array[intensity_channel * height * width + y * width + x] = 0.0f;
  }
}

cudaError_t PreprocessCuda::undistort_image_launch(
  const InputImageBGR8Type * input_image, const double * dist_coeffs, const double * camera_matrix,
  const double * projection_matrix, const size_t width, const size_t height,
  InputImageBGR8Type * output_image, float * output_array)
{
  dim3 threads(kernel_2d_dim, kernel_2d_dim);
  dim3 blocks((width + threads.x - 1) / threads.x, (height + threads.y - 1) / threads.y);

  undistort_image_kernel<<<blocks, threads, 0, stream_>>>(
    input_image, dist_coeffs, camera_matrix, projection_matrix, width, height, output_image,
    output_array);
  return cudaGetLastError();
}

/**
 * @brief Applies a JET colormap to a value for visualization.
 *
 * @param v The input color value (0.0 - 255.0).
 * @return An InputImageBGR8Type pixel representing the color.
 */
__device__ inline InputImageBGR8Type jet_colormap(float v)
{
  constexpr float one_third_255 = 255.0f / 3.0f;
  constexpr float two_third_255 = 2.0f * one_third_255;

  v = fminf(fmaxf(v, 0.0f), 255.0f);

  unsigned char r = 0, g = 0, b = 0;

  if (v < one_third_255) {
    b = 255;
    g = static_cast<unsigned char>(v * 3.0f);
  } else if (v < two_third_255) {
    r = static_cast<unsigned char>((v - one_third_255) * 3.0f);
    g = 255;
    b = static_cast<unsigned char>(255.0f - (v - one_third_255) * 3.0f);
  } else {
    r = 255;
    g = static_cast<unsigned char>(255.0f - (v - two_third_255) * 3.0f);
  }
  return {b, g, r};
}

/**
 * @brief Encodes depth and intensity into a single 64-bit unsigned integer.
 *
 * The depth and intensity values are first converted to their IEEE 754
 * binary representation as 32-bit unsigned integers. The depth is stored
 * in the higher 32 bits, and the intensity in the lower 32 bits.
 *
 * @tparam T1 Type of the depth value (should be convertible to float).
 * @tparam T2 Type of the intensity value (should be convertible to float).
 * @param depth The depth value to encode.
 * @param intensity The intensity value to encode.
 * @return A 64-bit unsigned integer containing both encoded values.
 */
template <typename T1, typename T2>
__device__ inline unsigned long long encode_depth_intensity(T1 depth, T2 intensity)
{
  unsigned int depth_int = __float_as_uint(static_cast<float>(depth));
  unsigned int intensity_int = __float_as_uint(static_cast<float>(intensity));

  return (static_cast<unsigned long long>(depth_int) << 32) | intensity_int;
}

/**
 * @brief Decodes depth and intensity from a single 64-bit unsigned integer.
 *
 * The function extracts the higher 32 bits for depth and the lower 32 bits
 * for intensity, converting them back to float values using their IEEE 754
 * binary representation.
 *
 * @param packed The 64-bit unsigned integer containing encoded depth and intensity.
 * @return A pair of floats: the first is depth, the second is intensity.
 */
__device__ inline std::pair<float, float> decode_depth_intensity(unsigned long long packed)
{
  unsigned int depth_int = packed >> 32;
  unsigned int intensity_int = packed & 0xFFFFFFFF;

  float depth = __uint_as_float(depth_int);
  float intensity = __uint_as_float(intensity_int);

  return {depth, intensity};
}

/**
 * @brief Projects 3D points into the image plane and updates the depth-intensity buffer.
 *
 * Each thread processes one point, transforming it to the camera frame,
 * projecting it onto the image plane, and updating the depth-intensity
 * buffer with atomic operations to ensure thread safety.
 *
 * @param input_points Pointer to the array of 3D points with intensity.
 * @param tf_matrix Pointer to the 4x4 transformation matrix (row-major).
 * @param projection_matrix Pointer to the 3x4 projection matrix (row-major).
 * @param num_points The total number of input points.
 * @param width The width of the image.
 * @param height The height of the image.
 * @param max_depth The maximum depth threshold for valid points.
 * @param depth_intensity_buffer Pointer to the depth-intensity buffer to be updated.
 * @param num_points_projected Pointer to a counter for the number of projected points.
 */
__global__ void project_points_kernel(
  const InputPointType * __restrict__ input_points, const double * __restrict__ tf_matrix,
  const double * __restrict__ projection_matrix, const size_t num_points, const size_t width,
  const size_t height, const double max_depth,
  unsigned long long * __restrict__ depth_intensity_buffer,
  uint32_t * __restrict__ num_points_projected)
{
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;

  const double & fx_new = projection_matrix[0];  // P[0,0]
  const double & fy_new = projection_matrix[5];  // P[1,1]
  const double & cx_new = projection_matrix[2];  // P[0,2]
  const double & cy_new = projection_matrix[6];  // P[1,2]

  if (idx >= num_points) {
    return;
  }

  const InputPointType p = input_points[idx];

  const double p_cam_x =
    tf_matrix[0] * p.x + tf_matrix[1] * p.y + tf_matrix[2] * p.z + tf_matrix[3];
  const double p_cam_y =
    tf_matrix[4] * p.x + tf_matrix[5] * p.y + tf_matrix[6] * p.z + tf_matrix[7];
  const double p_cam_z =
    tf_matrix[8] * p.x + tf_matrix[9] * p.y + tf_matrix[10] * p.z + tf_matrix[11];

  if (p_cam_z < 0.0 || p_cam_z >= max_depth) {
    return;
  }

  const int64_t u = lround((fx_new * p_cam_x + cx_new * p_cam_z) / p_cam_z);
  const int64_t v = lround((fy_new * p_cam_y + cy_new * p_cam_z) / p_cam_z);

  if (u < 0 || u >= static_cast<int64_t>(width) || v < 0 || v >= static_cast<int64_t>(height)) {
    return;
  }

  auto packed = encode_depth_intensity(p_cam_z, p.intensity);
  auto old = atomicMin(&depth_intensity_buffer[v * width + u], packed);
  if (old != packed) {
    atomicAdd(num_points_projected, 1);
  }
}

/**
 * @brief Applies dilation to the depth-intensity buffer and generates the final output.
 *
 * Each thread processes one pixel in the image, checking its neighborhood
 * in the depth-intensity buffer to find the closest point. If a valid point
 * is found, it updates the output arrays and undistorted image accordingly.
 *
 * @param undistorted_image Pointer to the undistorted image to be updated.
 * @param width The width of the image.
 * @param height The height of the image.
 * @param max_depth The maximum depth threshold for scaling.
 * @param dilation_size The size of the dilation kernel (in pixels).
 * @param depth_intensity_buffer Pointer to the depth-intensity buffer.
 * @param output_array Pointer to the output array for RGB, depth, and intensity data.
 */
__global__ void dilate_kernel(
  InputImageBGR8Type * __restrict__ undistorted_image, const size_t width, const size_t height,
  const double max_depth, const int dilation_size,
  const unsigned long long * __restrict__ depth_intensity_buffer, float * __restrict__ output_array)
{
  const size_t u = blockIdx.x * blockDim.x + threadIdx.x;
  const size_t v = blockIdx.y * blockDim.y + threadIdx.y;

  if (u >= width || v >= height) {
    return;
  }

  unsigned long long min_packed = ULLONG_MAX;

  for (int dy = -dilation_size; dy <= dilation_size; ++dy) {
    for (int dx = -dilation_size; dx <= dilation_size; ++dx) {
      int neighbor_u = u + dx;
      int neighbor_v = v + dy;

      if (
        neighbor_u >= 0 && neighbor_v >= 0 && neighbor_u < static_cast<int>(width) &&
        neighbor_v < static_cast<int>(height)) {
        unsigned long long packed = depth_intensity_buffer[neighbor_v * width + neighbor_u];
        if (packed < min_packed) {
          min_packed = packed;
        }
      }
    }
  }

  if (min_packed != ULLONG_MAX) {
    auto [depth, intensity] = decode_depth_intensity(min_packed);
    const auto depth_scaled = static_cast<float>(depth / max_depth);
    const auto intensity_scaled = static_cast<float>(intensity / 255.0f);

    InputImageBGR8Type color = jet_colormap(intensity);
    size_t pixel_idx = v * width + u;

    output_array[3 * height * width + pixel_idx] = depth_scaled;
    output_array[4 * height * width + pixel_idx] = intensity_scaled;
    undistorted_image[pixel_idx] = color;
  }
}

cudaError_t PreprocessCuda::project_points_launch(
  const InputPointType * input_points, InputImageBGR8Type * undistorted_image,
  const double * tf_matrix, const double * projection_matrix, const size_t num_points,
  const size_t width, const size_t height, float * output_array, uint32_t * num_points_projected)
{
  // Reset buffer to max value - closest points are promoted
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    depth_intensity_buffer_.get(), 0xFF, sizeof(unsigned long long) * height * width, stream_));

  dim3 threads_proj(kernel_1d_dim);
  dim3 blocks_proj((num_points + threads_proj.x - 1) / threads_proj.x);
  project_points_kernel<<<blocks_proj, threads_proj, 0, stream_>>>(
    input_points, tf_matrix, projection_matrix, num_points, width, height, max_depth_,
    depth_intensity_buffer_.get(), num_points_projected);

  dim3 threads_dilate(kernel_2d_dim, kernel_2d_dim);
  dim3 blocks_dilate(
    (width + threads_dilate.x - 1) / threads_dilate.x,
    (height + threads_dilate.y - 1) / threads_dilate.y);
  dilate_kernel<<<blocks_dilate, threads_dilate, 0, stream_>>>(
    undistorted_image, width, height, max_depth_, dilation_size_, depth_intensity_buffer_.get(),
    output_array);

  return cudaGetLastError();
}

}  // namespace autoware::calibration_status_classifier
