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

#ifndef NETWORKS__PREPROCESS__MULTI_CAMERA_PREPROCESS_HPP_
#define NETWORKS__PREPROCESS__MULTI_CAMERA_PREPROCESS_HPP_

#include "networks/preprocess/multi_camera_preprocess_kernel.hpp"
#include "ros_vad_logger.hpp"

#include <opencv2/core/mat.hpp>

#include <cuda_runtime.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::tensorrt_vad
{

/**
 * @class MultiCameraPreprocessor
 * @brief GPU preprocessing pipeline for multiple camera images
 *
 * This class encapsulates the entire flow from receiving cv::Mat images from host,
 * copying to device, custom CUDA kernel-based resize/BGR2RGB convert/normalize/concatenate
 * operations. Resources are allocated in constructor (RAII) and released in destructor.
 */
class MultiCameraPreprocessor
{
public:
  // Template constructor to accept shared_ptr<LoggerType>
  template <typename LoggerType>
  MultiCameraPreprocessor(
    const MultiCameraPreprocessConfig & config, std::shared_ptr<LoggerType> logger);

  ~MultiCameraPreprocessor();

  // Prohibit copy constructor and copy assignment operator to prevent double deallocation
  MultiCameraPreprocessor(const MultiCameraPreprocessor &) = delete;
  MultiCameraPreprocessor & operator=(const MultiCameraPreprocessor &) = delete;

  /**
   * @brief Batch preprocess multiple camera images (cv::Mat) on GPU.
   * @param camera_images Host-side input images (cv::Mat) vector. Should be BGR8 format.
   * @param d_output_buffer Pointer to device output buffer. Results are written here in CHW format.
   * @param stream CUDA stream to use for execution.
   * @return cudaError_t Execution status.
   */
  cudaError_t preprocess_images(
    const std::vector<cv::Mat> & camera_images, float * d_output_buffer, cudaStream_t stream);

private:
  /**
   * @brief Validates input image vector (size, format, etc.).
   * @param camera_images Image vector to validate.
   * @return cudaError_t Validation result. cudaSuccess if successful.
   */
  cudaError_t validate_input(const std::vector<cv::Mat> & camera_images) const;

  /**
   * @brief Cleanup allocated CUDA memory resources.
   * Called when allocation fails or in destructor.
   */
  void cleanup_cuda_resources();

  MultiCameraPreprocessConfig config_;
  std::shared_ptr<autoware::tensorrt_vad::VadLogger> logger_;  // Direct VadLogger pointer

  // --- GPU Buffers ---
  // Input buffers (allocated in constructor)
  uint8_t * d_input_buffer_{nullptr};  // Single continuous buffer storing all raw input images
  uint8_t ** d_input_image_ptrs_{
    nullptr};  // Pointer array pointing to start positions of each image within d_input_buffer_

  // Intermediate buffers for separated kernel processing
  uint8_t * d_resized_buffer_{nullptr};       // Buffer for resized images (BGR uint8_t format)
  uint8_t ** d_resized_image_ptrs_{nullptr};  // Pointer array for resized image positions
};

// Template implementations
template <typename LoggerType>
MultiCameraPreprocessor::MultiCameraPreprocessor(
  const MultiCameraPreprocessConfig & config, std::shared_ptr<LoggerType> logger)
: config_(config), logger_(std::static_pointer_cast<autoware::tensorrt_vad::VadLogger>(logger))
{
  // Logger accepts only classes that inherit from VadLogger
  static_assert(
    std::is_base_of_v<autoware::tensorrt_vad::VadLogger, LoggerType>,
    "LoggerType must be VadLogger or derive from VadLogger.");

  logger_->debug(
    "MultiCameraPreprocessor config: input=" + std::to_string(config_.input_width) + "x" +
    std::to_string(config_.input_height) + ", output=" + std::to_string(config_.output_width) +
    "x" + std::to_string(config_.output_height) +
    ", cameras=" + std::to_string(config_.num_cameras));

  // --- Allocate Input Buffers ---
  const size_t single_input_size =
    static_cast<size_t>(config_.input_width) * config_.input_height * 3;
  const size_t total_input_size = single_input_size * config_.num_cameras;

  // --- Allocate Resized Buffers ---
  const size_t single_resized_size =
    static_cast<size_t>(config_.output_width) * config_.output_height * 3;
  const size_t total_resized_size = single_resized_size * config_.num_cameras;

  cudaError_t err = cudaMalloc(&d_input_buffer_, total_input_size);
  if (err != cudaSuccess) {
    std::string error_msg = "Failed to allocate input buffer of size " +
                            std::to_string(total_input_size) + ": " + cudaGetErrorString(err);
    logger_->error(error_msg);
    throw std::runtime_error(error_msg);
  }

  err = cudaMalloc(&d_resized_buffer_, total_resized_size);
  if (err != cudaSuccess) {
    std::string error_msg = "Failed to allocate resized buffer of size " +
                            std::to_string(total_resized_size) + ": " + cudaGetErrorString(err);
    logger_->error(error_msg);
    cleanup_cuda_resources();
    throw std::runtime_error(error_msg);
  }

  err = cudaMalloc(&d_input_image_ptrs_, config_.num_cameras * sizeof(uint8_t *));
  if (err != cudaSuccess) {
    std::string error_msg =
      "Failed to allocate input image pointers: " + std::string(cudaGetErrorString(err));
    logger_->error(error_msg);
    cleanup_cuda_resources();
    throw std::runtime_error(error_msg);
  }

  err = cudaMalloc(&d_resized_image_ptrs_, config_.num_cameras * sizeof(uint8_t *));
  if (err != cudaSuccess) {
    std::string error_msg =
      "Failed to allocate resized image pointers: " + std::string(cudaGetErrorString(err));
    logger_->error(error_msg);
    cleanup_cuda_resources();
    throw std::runtime_error(error_msg);
  }

  // Setup pointer arrays for both input and resized buffers
  std::vector<uint8_t *> h_input_ptrs(config_.num_cameras);
  std::vector<uint8_t *> h_resized_ptrs(config_.num_cameras);
  for (int32_t i = 0; i < config_.num_cameras; ++i) {
    h_input_ptrs[i] = d_input_buffer_ + i * single_input_size;
    h_resized_ptrs[i] = d_resized_buffer_ + i * single_resized_size;
  }

  err = cudaMemcpy(
    d_input_image_ptrs_, h_input_ptrs.data(), config_.num_cameras * sizeof(uint8_t *),
    cudaMemcpyHostToDevice);
  if (err != cudaSuccess) {
    std::string error_msg =
      "Failed to copy input image pointers to device: " + std::string(cudaGetErrorString(err));
    logger_->error(error_msg);
    cleanup_cuda_resources();
    throw std::runtime_error(error_msg);
  }

  err = cudaMemcpy(
    d_resized_image_ptrs_, h_resized_ptrs.data(), config_.num_cameras * sizeof(uint8_t *),
    cudaMemcpyHostToDevice);
  if (err != cudaSuccess) {
    std::string error_msg =
      "Failed to copy resized image pointers to device: " + std::string(cudaGetErrorString(err));
    logger_->error(error_msg);
    cleanup_cuda_resources();
    throw std::runtime_error(error_msg);
  }

  logger_->info("MultiCameraPreprocessor initialized successfully with separated kernel support");
}

}  // namespace autoware::tensorrt_vad

#endif  // NETWORKS__PREPROCESS__MULTI_CAMERA_PREPROCESS_HPP_
