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

#ifndef AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__PREPROCESS_CUDA_HPP_
#define AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__PREPROCESS_CUDA_HPP_

#include "autoware/calibration_status_classifier/data_type.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_runtime_api.h>

// cspell:ignoreWords Conrady
namespace autoware::calibration_status_classifier
{

/**
 * @brief CUDA-based preprocessing class for calibration status detection
 *
 * This class provides GPU-accelerated preprocessing operations required for
 * LiDAR-camera calibration validation:
 * 1. Image undistortion using camera intrinsic parameters
 * 2. Point cloud projection onto undistorted image plane with depth buffering
 * 3. Generation of 5-channel input data (RGB + depth + intensity) for neural network
 *
 * All operations are implemented as CUDA kernels for high performance processing
 * of large point clouds and high-resolution images.
 */
class PreprocessCuda
{
public:
  /**
   * @brief Constructor for PreprocessCuda
   * @param max_depth Maximum depth for projected LiDAR points in the camera frame
   * @param dilation_size Size of morphological dilation kernel for point projection
   * @param max_width Maximum expected image width in pixels
   * @param max_height Maximum expected image height in pixels
   * @param stream CUDA stream for asynchronous kernel launches
   */
  explicit PreprocessCuda(
    const double max_depth, const uint32_t dilation_size, const uint32_t max_width,
    const uint32_t max_height, cudaStream_t & stream);

  cudaError_t copy_image_launch(
    const InputImageBGR8Type * input_image, const size_t width, const size_t height,
    InputImageBGR8Type * output_image, float * output_array);

  /**
   * @brief Launch image undistortion kernel
   *
   * Performs camera image undistortion using Brown-Conrady distortion model.
   * Generates both undistorted image and initial RGB channels for neural network input.
   *
   * @param input_image Original distorted image (BGR8 format)
   * @param dist_coeffs Distortion coefficients [k1, k2, p1, p2, k3, k4, k5, k6]
   * @param camera_matrix Original camera intrinsic matrix (3x3, row-major)
   * @param projection_matrix New projection matrix for undistorted image (3x4, row-major)
   * @param width Image width in pixels
   * @param height Image height in pixels
   * @param output_image Undistorted image output
   * @param output_array 5-channel array with RGB channels filled
   * @return cudaError_t CUDA error status
   */
  cudaError_t undistort_image_launch(
    const InputImageBGR8Type * input_image, const double * dist_coeffs,
    const double * camera_matrix, const double * projection_matrix, const size_t width,
    const size_t height, InputImageBGR8Type * output_image, float * output_array);

  /**
   * @brief Launch point cloud projection kernel
   *
   * Projects 3D LiDAR points onto undistorted image plane with depth buffering.
   * Applies morphological dilation and JET colormap for visualization.
   * Fills depth and intensity channels of the 5-channel neural network input.
   *
   * @param input_points Input point cloud data with XYZ coordinates and intensity
   * @param undistorted_image Undistorted image for visualization overlay
   * @param tf_matrix 4x4 transformation matrix from LiDAR to camera frame (column-major)
   * @param projection_matrix Camera projection matrix (3x4, row-major)
   * @param num_points Total number of points in input cloud
   * @param width Image width in pixels
   * @param height Image height in pixels
   * @param output_array 5-channel array with depth and intensity channels filled
   * @param num_points_projected Output counter for successfully projected points
   * @return cudaError_t CUDA error status
   */
  cudaError_t project_points_launch(
    const InputPointType * input_points, InputImageBGR8Type * undistorted_image,
    const double * tf_matrix, const double * projection_matrix, const size_t num_points,
    const size_t width, const size_t height, float * output_array, uint32_t * num_points_projected);

private:
  double max_depth_;
  int dilation_size_;
  cuda_utils::CudaUniquePtr<unsigned long long[]> depth_intensity_buffer_;
  cudaStream_t stream_;
};

}  // namespace autoware::calibration_status_classifier

#endif  // AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__PREPROCESS_CUDA_HPP_
