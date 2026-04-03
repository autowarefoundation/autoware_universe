// Copyright 2026 TIER IV
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

#ifndef AUTOWARE__BEVFUSION__CAMERA__CAMERA_DATA_HPP_
#define AUTOWARE__BEVFUSION__CAMERA__CAMERA_DATA_HPP_

#include "autoware/bevfusion/camera/camera_matrices.hpp"
#include "autoware/bevfusion/camera/camera_preprocess.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>

namespace autoware::bevfusion
{
using autoware::cuda_utils::CudaUniquePtr;

struct ImagePreProcessingParams
{
  std::int64_t original_image_height;
  std::int64_t original_image_width;
  std::int64_t roi_height;
  std::int64_t roi_width;
  float image_aug_scale_y;
  float image_aug_scale_x;
  bool run_image_undistortion;
  std::int64_t crop_height;
  std::int64_t crop_width;
  std::int64_t resized_height;
  std::int64_t resized_width;
  std::int64_t roi_start_y;
  std::int64_t roi_start_x;

  ImagePreProcessingParams(
    const std::int64_t original_image_height, const std::int64_t original_image_width,
    const std::int64_t roi_height, const std::int64_t roi_width, const float image_aug_scale_y,
    const float image_aug_scale_x, const bool run_image_undistortion);
};

class CameraData
{
public:
  CameraData(
    rclcpp::Node * node, const int camera_id,
    const ImagePreProcessingParams & image_pre_processing_params,
    const std::shared_ptr<CameraMatrices> & camera_matrices_ptr);
  ~CameraData();

  void update_image_msg(const sensor_msgs::msg::Image::ConstSharedPtr & input_camera_image_msg);
  void update_camera_info(const sensor_msgs::msg::CameraInfo & input_camera_info_msg);
  std::optional<sensor_msgs::msg::CameraInfo> camera_info() const;
  sensor_msgs::msg::CameraInfo camera_info_value() const;

  sensor_msgs::msg::Image::ConstSharedPtr image_msg() const;
  bool is_image_msg_available() const;
  bool is_camera_info_available() const;
  std::size_t output_img_offset() const;

  bool preprocess_image(std::uint8_t * output_img);
  bool is_camera_matrices_ready() const;
  bool is_image_encoding_supported() const;
  cudaError_t sync_cuda_stream();

private:
  // Camera info buffer
  std::optional<sensor_msgs::msg::CameraInfo> camera_info_;

  // Image buffer
  // TODO(KokSeang): Remove this once we move preprocessing to subscribers
  sensor_msgs::msg::Image::ConstSharedPtr image_msg_;

  // GPU Memory for preprocessed image
  // image buffers
  CudaUniquePtr<std::uint8_t[]> image_buffer_d_{nullptr};
  CudaUniquePtr<std::uint8_t[]> undistorted_image_buffer_d_{nullptr};

  rclcpp::Logger logger_;
  int camera_id_;
  ImagePreProcessingParams image_pre_processing_params_;
  std::size_t output_img_offset_;

  cudaStream_t stream_{nullptr};
  std::unique_ptr<CameraPreprocess> camera_preprocess_ptr_{nullptr};
  std::shared_ptr<CameraMatrices> camera_matrices_ptr_{nullptr};

  // Supported image encoding, only RGB8 is supported for now
  const std::string supported_image_encoding_{sensor_msgs::image_encodings::RGB8};
};
}  // namespace autoware::bevfusion
#endif  // AUTOWARE__BEVFUSION__CAMERA__CAMERA_DATA_HPP_
