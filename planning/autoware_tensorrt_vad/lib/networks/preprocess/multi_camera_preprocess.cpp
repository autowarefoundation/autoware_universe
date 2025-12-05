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

#include "../src/networks/preprocess/multi_camera_preprocess.hpp"

#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace autoware::tensorrt_vad
{

MultiCameraPreprocessor::~MultiCameraPreprocessor()
{
  cleanup_cuda_resources();
}

void MultiCameraPreprocessor::cleanup_cuda_resources()
{
  if (d_input_buffer_) {
    cudaFree(d_input_buffer_);
    d_input_buffer_ = nullptr;
  }
  if (d_resized_buffer_) {
    cudaFree(d_resized_buffer_);
    d_resized_buffer_ = nullptr;
  }
  if (d_input_image_ptrs_) {
    cudaFree(d_input_image_ptrs_);
    d_input_image_ptrs_ = nullptr;
  }
  if (d_resized_image_ptrs_) {
    cudaFree(d_resized_image_ptrs_);
    d_resized_image_ptrs_ = nullptr;
  }
}

cudaError_t MultiCameraPreprocessor::validate_input(
  const std::vector<cv::Mat> & camera_images) const
{
  // Check number of cameras
  if (camera_images.size() != static_cast<size_t>(config_.num_cameras)) {
    logger_->error(
      "Camera count mismatch: expected " + std::to_string(config_.num_cameras) + ", got " +
      std::to_string(camera_images.size()));
    return cudaErrorInvalidValue;
  }

  const size_t expected_input_size =
    static_cast<size_t>(config_.input_width) * config_.input_height * 3;

  for (size_t i = 0; i < camera_images.size(); ++i) {
    const auto & img = camera_images.at(i);

    // Check if image is empty
    if (img.empty()) {
      logger_->error("Camera " + std::to_string(i) + ": image is empty");
      return cudaErrorInvalidValue;
    }

    // Check if image memory is continuous
    if (!img.isContinuous()) {
      logger_->error("Camera " + std::to_string(i) + ": image memory is not continuous");
      return cudaErrorInvalidValue;
    }

    // Check image size and format
    const size_t actual_size = img.total() * img.elemSize();
    if (actual_size != expected_input_size) {
      logger_->error(
        "Camera " + std::to_string(i) + ": size mismatch - expected " +
        std::to_string(expected_input_size) + ", got " + std::to_string(actual_size) +
        " (dims: " + std::to_string(img.cols) + "x" + std::to_string(img.rows) + "x" +
        std::to_string(img.channels()) + ", elemSize: " + std::to_string(img.elemSize()) + ")");
      return cudaErrorInvalidValue;
    }

    // Check image format (should be BGR, 3 channels, 8UC3)
    if (img.channels() != 3 || img.depth() != CV_8U) {
      logger_->error(
        "Camera " + std::to_string(i) +
        ": invalid format - channels: " + std::to_string(img.channels()) +
        ", depth: " + std::to_string(img.depth()) + " (expected: 3 channels, CV_8U)");
      return cudaErrorInvalidValue;
    }
  }

  return cudaSuccess;
}

cudaError_t MultiCameraPreprocessor::preprocess_images(
  const std::vector<cv::Mat> & camera_images, float * d_output_buffer, cudaStream_t stream)
{
  logger_->debug(
    "Starting separated CUDA preprocessing for " + std::to_string(camera_images.size()) +
    " cameras");

  // Step 0: Validate input data
  cudaError_t validation_status = validate_input(camera_images);
  if (validation_status != cudaSuccess) {
    logger_->error(
      "Input validation failed with error: " + std::string(cudaGetErrorString(validation_status)));
    return validation_status;
  }

  logger_->debug("Input validation passed");

  // Step 1: Copy image data from host (cv::Mat) to pre-allocated device buffer
  const size_t single_input_size =
    static_cast<size_t>(config_.input_width) * config_.input_height * 3;
  for (int32_t i = 0; i < config_.num_cameras; ++i) {
    const auto & img = camera_images.at(i);
    uint8_t * d_dst = d_input_buffer_ + i * single_input_size;

    cudaError_t copy_result =
      cudaMemcpyAsync(d_dst, img.data, single_input_size, cudaMemcpyHostToDevice, stream);
    if (copy_result != cudaSuccess) {
      logger_->error(
        "cudaMemcpyAsync failed for camera " + std::to_string(i) + ": " +
        cudaGetErrorString(copy_result));
      return copy_result;
    }
  }

  logger_->debug("Image data copied to GPU");

  // Step 2: Launch resize kernel
  cudaError_t resize_result =
    launch_multi_camera_resize_kernel(d_input_image_ptrs_, d_resized_image_ptrs_, config_, stream);

  if (resize_result != cudaSuccess) {
    logger_->error(
      "Resize kernel launch failed: " + std::string(cudaGetErrorString(resize_result)));
    return resize_result;
  }

  logger_->debug("Resize kernel completed");

  // Step 3: Launch normalization kernel
  cudaError_t normalize_result =
    launch_multi_camera_normalize_kernel(d_resized_image_ptrs_, d_output_buffer, config_, stream);

  if (normalize_result != cudaSuccess) {
    logger_->error(
      "Normalization kernel launch failed: " + std::string(cudaGetErrorString(normalize_result)));
    return normalize_result;
  }

  logger_->debug("Separated kernel preprocessing completed successfully");
  return cudaSuccess;
}

}  // namespace autoware::tensorrt_vad
