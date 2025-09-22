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

#include "autoware/calibration_status/preprocess_cuda.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cmath>
#include <cstdint>

namespace autoware::calibration_status
{

PreprocessCuda::PreprocessCuda(
  const double max_depth, const uint32_t dilation_size, cudaStream_t & stream)
: max_depth_(max_depth), dilation_size_(static_cast<int>(dilation_size)), stream_(stream) {};

__global__ void copyImage_kernel(
  const InputImageBGR8Type * __restrict__ input_image, const size_t width, const size_t height,
  InputImageBGR8Type * __restrict__ output_image, float * output_array)
{
  const size_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const size_t y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x < width && y < height) {
    const InputImageBGR8Type px = input_image[y * width + x];
    output_image[y * width + x] = px;
    output_array[0 * height * width + y * width + x] = static_cast<float>(px.r) / 255.0f;
    output_array[1 * height * width + y * width + x] = static_cast<float>(px.g) / 255.0f;
    output_array[2 * height * width + y * width + x] = static_cast<float>(px.b) / 255.0f;
    output_array[3 * height * width + y * width + x] = 0.0f;
    output_array[4 * height * width + y * width + x] = 0.0f;
  }
}

cudaError_t PreprocessCuda::copyImage_launch(
  const InputImageBGR8Type * input_image, const size_t width, const size_t height,
  InputImageBGR8Type * output_image, float * output_array)
{
  dim3 threads(16, 16);
  dim3 blocks((width + threads.x - 1) / threads.x, (height + threads.y - 1) / threads.y);

  copyImage_kernel<<<blocks, threads, 0, stream_>>>(
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
  if (u < 0.0f || u >= width || v < 0.0f || v >= height) {
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
  float u_ratio = u - x1;
  float v_ratio = v - y1;
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
__global__ void undistortImage_kernel(
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

    const double x_proj = (x - cx_new) / fx_new;
    const double y_proj = (y - cy_new) / fy_new;

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
    InputImageBGR8Type px;
    if (u >= 0 && v >= 0 && u < width && v < height) {
      px = bilinear_sample(input_image, u, v, width, height);
    } else {
      px = {0, 0, 0};
    }

    output_image[y * width + x] = px;
    output_array[0 * height * width + y * width + x] = static_cast<float>(px.r) / 255.0f;
    output_array[1 * height * width + y * width + x] = static_cast<float>(px.g) / 255.0f;
    output_array[2 * height * width + y * width + x] = static_cast<float>(px.b) / 255.0f;
    output_array[3 * height * width + y * width + x] = 0.0f;
    output_array[4 * height * width + y * width + x] = 0.0f;
  }
}

cudaError_t PreprocessCuda::undistortImage_launch(
  const InputImageBGR8Type * input_image, const double * dist_coeffs, const double * camera_matrix,
  const double * projection_matrix, const size_t width, const size_t height,
  InputImageBGR8Type * output_image, float * output_array)
{
  dim3 threads(16, 16);
  dim3 blocks((width + threads.x - 1) / threads.x, (height + threads.y - 1) / threads.y);

  undistortImage_kernel<<<blocks, threads, 0, stream_>>>(
    input_image, dist_coeffs, camera_matrix, projection_matrix, width, height, output_image,
    output_array);
  return cudaGetLastError();
}

/**
 * @brief Applies a JET colormap to a value for visualization.
 *
 * @param v The input value, normalized between 0.0 and 1.0.
 * @return An InputImageBGR8Type pixel representing the color.
 */
__device__ inline InputImageBGR8Type jet_colormap(float v)
{
  v = fminf(fmaxf(v, 0.0f), 1.0f);

  unsigned char r = 0, g = 0, b = 0;

  if (v < 0.33f) {
    b = 255;
    g = (unsigned char)(255.0f * (v / 0.33f));
  } else if (v < 0.66f) {
    r = (unsigned char)(255.0f * ((v - 0.33f) / 0.33f));
    g = 255;
    b = (unsigned char)(255.0f * (1.0f - (v - 0.33f) / 0.33f));
  } else {
    r = 255;
    g = (unsigned char)(255.0f * (1.0f - (v - 0.66f) / 0.34f));
  }
  return {b, g, r};
}

/**
 * @brief Projects a point cloud and performs a depth-aware dilation in a single pass.
 * This kernel produces a final 5-channel uint8 output, matching the Python script's logic.
 *
 * @param input_points Pointer to the input point cloud data.
 * @param undistorted_image Pointer to the undistorted image to draw on.
 * @param tf_matrix The 4x4 transformation matrix (row-major) to transform points.
 * @param projection_matrix The new projection matrix (P) for the undistorted image.
 * @param num_points The total number of points in the input cloud.
 * @param width The width of the output image and data array.
 * @param height The height of the output image and data array.
 * @param max_depth The maximum range to use for depth normalization.
 * @param metric_depth_buffer A temporary buffer of size (width*height) initialized to 0.0f, used
 * for Z-buffering.
 * @param output_array The final output 2D grid where uint8 data is stored.
 * @param num_points_projected A counter for the number of successfully projected points.
 */
__global__ void projectPoints_kernel(
  const InputPointType * __restrict__ input_points,
  InputImageBGR8Type * __restrict__ undistorted_image, const double * __restrict__ tf_matrix,
  const double * __restrict__ projection_matrix, const size_t num_points, const size_t width,
  const size_t height, const double max_depth, const int dilation_size,
  float * __restrict__ metric_depth_buffer, float * __restrict__ output_array,
  uint32_t * __restrict__ num_points_projected)
{
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;

  const double & fx_new = projection_matrix[0];  // P[0,0]
  const double & fy_new = projection_matrix[5];  // P[1,1]
  const double & cx_new = projection_matrix[2];  // P[0,2]
  const double & cy_new = projection_matrix[6];  // P[1,2]

  if (idx < num_points) {
    const InputPointType p = input_points[idx];

    // Transform the point into the camera coordinate frame
    const double p_cam_x =
      tf_matrix[0] * p.x + tf_matrix[1] * p.y + tf_matrix[2] * p.z + tf_matrix[3];
    const double p_cam_y =
      tf_matrix[4] * p.x + tf_matrix[5] * p.y + tf_matrix[6] * p.z + tf_matrix[7];
    const double p_cam_z =
      tf_matrix[8] * p.x + tf_matrix[9] * p.y + tf_matrix[10] * p.z + tf_matrix[11];

    // Ignore points that are behind or too far to the camera
    if (p_cam_z < 0.0 || p_cam_z >= max_depth) {
      return;
    }

    // Project 3D point onto the 2D undistorted image plane
    const int64_t u = lround((fx_new * p_cam_x + cx_new * p_cam_z) / p_cam_z);
    const int64_t v = lround((fy_new * p_cam_y + cy_new * p_cam_z) / p_cam_z);

    // Projection ROI validation
    if (u < 0 || u >= static_cast<int64_t>(width) || v < 0 || v >= static_cast<int64_t>(height)) {
      return;
    }
    const auto current_metric_depth = static_cast<float>(p_cam_z);

    // Main Projection Z-Buffer (using the separate metric buffer)
    auto target_addr_int = reinterpret_cast<uint32_t *>(&metric_depth_buffer[v * width + u]);
    float old_metric_depth = __uint_as_float(*target_addr_int);

    while (old_metric_depth == 0.0f || current_metric_depth < fabsf(old_metric_depth)) {
      unsigned int assumed_int = __float_as_uint(old_metric_depth);
      unsigned int returned_int =
        atomicCAS(target_addr_int, assumed_int, __float_as_uint(current_metric_depth));

      if (returned_int == assumed_int) {  // Race condition resolved
        InputImageBGR8Type color = jet_colormap(static_cast<float>(p.intensity) / 255.0f);
        size_t pixel_idx = v * width + u;

        output_array[3 * height * width + pixel_idx] = current_metric_depth / max_depth;
        output_array[4 * height * width + pixel_idx] = static_cast<float>(p.intensity) / 255.0f;

        undistorted_image[pixel_idx] = color;
        atomicAdd(num_points_projected, 1);

        for (int dy = -dilation_size; dy <= dilation_size; ++dy) {
          for (int dx = -dilation_size; dx <= dilation_size; ++dx) {
            if (dx == 0 && dy == 0) continue;

            int neighbor_u = u + dx;
            int neighbor_v = v + dy;

            if (
              neighbor_u >= 0 && neighbor_v >= 0 && neighbor_u < static_cast<int64_t>(width) &&
              neighbor_v < static_cast<int64_t>(height)) {
              size_t neighbor_idx = neighbor_v * width + neighbor_u;
              auto neighbor_addr_int =
                reinterpret_cast<uint32_t *>(&metric_depth_buffer[neighbor_idx]);
              float neighbor_old_metric_depth = __uint_as_float(*neighbor_addr_int);

              while (neighbor_old_metric_depth == 0.0f ||
                     current_metric_depth < fabsf(neighbor_old_metric_depth)) {
                unsigned int neighbor_assumed_int = __float_as_uint(neighbor_old_metric_depth);
                unsigned int neighbor_returned_int = atomicCAS(
                  neighbor_addr_int, neighbor_assumed_int, __float_as_uint(-current_metric_depth));

                if (neighbor_returned_int == neighbor_assumed_int) {
                  output_array[3 * height * width + neighbor_idx] =
                    current_metric_depth / max_depth;
                  output_array[4 * height * width + neighbor_idx] =
                    static_cast<float>(p.intensity) / 255.0f;
                  undistorted_image[neighbor_idx] = color;
                  break;
                }
                neighbor_old_metric_depth = __uint_as_float(neighbor_returned_int);
              }
            }
          }
        }
        break;
      }
      old_metric_depth = __uint_as_float(returned_int);
    }
  }
}

cudaError_t PreprocessCuda::projectPoints_launch(
  const InputPointType * input_points, InputImageBGR8Type * undistorted_image,
  const double * tf_matrix, const double * projection_matrix, const size_t num_points,
  const size_t width, const size_t height, float * output_array, uint32_t * num_points_projected)
{
  dim3 threads(256);
  dim3 blocks((num_points + threads.x - 1) / threads.x);

  auto metric_depth_buffer = cuda_utils::make_unique<float[]>(width * height);

  projectPoints_kernel<<<blocks, threads, 0, stream_>>>(
    input_points, undistorted_image, tf_matrix, projection_matrix, num_points, width, height,
    max_depth_, dilation_size_, metric_depth_buffer.get(), output_array, num_points_projected);

  return cudaGetLastError();
}

}  // namespace autoware::calibration_status
