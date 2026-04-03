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

#include "autoware/bevfusion/camera/camera_data.hpp"

#include "autoware/bevfusion/bevfusion_config.hpp"
#include "autoware/bevfusion/camera/camera_preprocess.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>

namespace autoware::bevfusion
{

ImagePreProcessingParams::ImagePreProcessingParams(
  const std::int64_t image_height, const std::int64_t image_width, const std::int64_t roi_height,
  const std::int64_t roi_width, const float image_aug_scale_y, const float image_aug_scale_x,
  const bool run_image_undistortion)
: original_image_height(image_height),
  original_image_width(image_width),
  roi_height(roi_height),
  roi_width(roi_width),
  image_aug_scale_y(image_aug_scale_y),
  image_aug_scale_x(image_aug_scale_x),
  run_image_undistortion(run_image_undistortion)
{
  resized_height = original_image_height * image_aug_scale_y;
  resized_width = original_image_width * image_aug_scale_x;
  crop_height = resized_height - roi_height;
  crop_width = std::max(
    static_cast<std::int64_t>(0), static_cast<std::int64_t>((resized_width - roi_width) / 2));

  roi_start_x = std::max(static_cast<std::int64_t>(0), crop_width);
  roi_start_y = std::max(static_cast<std::int64_t>(0), crop_height);
}

CameraData::CameraData(
  rclcpp::Node * node, const int camera_id,
  const ImagePreProcessingParams & image_pre_processing_params,
  const std::shared_ptr<CameraMatrices> & camera_matrices_ptr)
: logger_(node->get_logger()),
  camera_id_(camera_id),
  image_pre_processing_params_(image_pre_processing_params),
  camera_matrices_ptr_(camera_matrices_ptr)
{
  // Initialization cuda stream
  cudaStreamCreate(&stream_);

  camera_preprocess_ptr_ = std::make_unique<CameraPreprocess>(stream_, camera_id_);
  image_buffer_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(
    image_pre_processing_params_.original_image_height *
    image_pre_processing_params_.original_image_width * BEVFusionConfig::kNumRGBChannels);

  if (image_pre_processing_params_.run_image_undistortion) {
    undistorted_image_buffer_d_ = autoware::cuda_utils::make_unique<std::uint8_t[]>(
      image_pre_processing_params_.original_image_height *
      image_pre_processing_params_.original_image_width * BEVFusionConfig::kNumRGBChannels);
  }

  output_img_offset_ = camera_id_ * BEVFusionConfig::kNumRGBChannels *
                       image_pre_processing_params_.roi_height *
                       image_pre_processing_params_.roi_width;
}

CameraData::~CameraData()
{
  if (stream_ != nullptr) {
    // 1. Wait for ALL pending GPU work to finish
    cudaStreamSynchronize(stream_);

    // 2. Destroy the stream handle
    cudaStreamDestroy(stream_);

    // 3. Mark it as null
    stream_ = nullptr;
  }
}

void CameraData::update_image_msg(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_camera_image_msg)
{
  // Update image message
  image_msg_ = input_camera_image_msg;
}

void CameraData::update_camera_info(const sensor_msgs::msg::CameraInfo & camera_info_msg)
{
  // Update camera info message
  camera_info_ = camera_info_msg;

  // Dont need to compute matrices and undistortion if it's not enabled
  if (!image_pre_processing_params_.run_image_undistortion) {
    return;
  }

  // Check if undistorted map x and y are already computed
  if (
    camera_matrices_ptr_->undistorted_map_x_d_ != nullptr &&
    camera_matrices_ptr_->undistorted_map_y_d_ != nullptr) {
    return;
  }

  // Update camera matrices
  camera_matrices_ptr_->update_camera_matrices(camera_info_msg);

  // Compute undistorted map x and y
  camera_matrices_ptr_->compute_undistorted_map_x_y(stream_);
}

bool CameraData::is_image_msg_available() const
{
  return image_msg_ != nullptr;
}

bool CameraData::is_camera_info_available() const
{
  return camera_info_.has_value();
}

std::optional<sensor_msgs::msg::CameraInfo> CameraData::camera_info() const
{
  return camera_info_;
}

sensor_msgs::msg::CameraInfo CameraData::camera_info_value() const
{
  if (!camera_info_) {
    throw std::runtime_error("Camera info is not available");
  }
  return *(camera_info_);
}

sensor_msgs::msg::Image::ConstSharedPtr CameraData::image_msg() const
{
  return image_msg_;
}

std::size_t CameraData::output_img_offset() const
{
  return output_img_offset_;
}

bool CameraData::preprocess_image(std::uint8_t * output_img)
{
  // 1. Copy image from CPU to GPU
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    image_buffer_d_.get(), image_msg_->data.data(),
    image_pre_processing_params_.original_image_height *
      image_pre_processing_params_.original_image_width * BEVFusionConfig::kNumRGBChannels,
    cudaMemcpyHostToDevice, stream_));

  if (image_pre_processing_params_.run_image_undistortion) {
    // 1. Check if original image shape is the same as the original image shape
    if (
      image_msg_->height != image_pre_processing_params_.original_image_height ||
      image_msg_->width != image_pre_processing_params_.original_image_width) {
      RCLCPP_ERROR(logger_, "Input image shape is not the same as the original image shape");
      return false;
    }

    // 2. Launch remap kernel for undistortion
    camera_preprocess_ptr_->remap_launch(
      image_buffer_d_.get(), undistorted_image_buffer_d_.get(),
      // Output dimensions (full resolution)
      image_pre_processing_params_.original_image_height,
      image_pre_processing_params_.original_image_width,
      // Input dimensions (full resolution)
      image_msg_->height, image_msg_->width,
      // Undistorted map x and y
      camera_matrices_ptr_->undistorted_map_x_d_.get(),
      camera_matrices_ptr_->undistorted_map_y_d_.get());

    // 3. Resize and extract ROI, and then saving to output_img
    // output_img is expected to be in the size of roi_height * roi_width * image channels (usually,
    camera_preprocess_ptr_->resizeAndExtractRoi_launch(
      undistorted_image_buffer_d_.get(), output_img,
      image_pre_processing_params_.original_image_height,
      image_pre_processing_params_.original_image_width,
      image_pre_processing_params_.resized_height, image_pre_processing_params_.resized_width,
      image_pre_processing_params_.roi_height, image_pre_processing_params_.roi_width,
      image_pre_processing_params_.roi_start_y, image_pre_processing_params_.roi_start_x);
  } else {
    // Skip the undistortion step and directly resize and extract ROI
    camera_preprocess_ptr_->resizeAndExtractRoi_launch(
      image_buffer_d_.get(), output_img, image_pre_processing_params_.original_image_height,
      image_pre_processing_params_.original_image_width,
      image_pre_processing_params_.resized_height, image_pre_processing_params_.resized_width,
      image_pre_processing_params_.roi_height, image_pre_processing_params_.roi_width,
      image_pre_processing_params_.roi_start_y, image_pre_processing_params_.roi_start_x);
  }

  return true;
}

bool CameraData::is_camera_matrices_ready() const
{
  if (!image_pre_processing_params_.run_image_undistortion) {
    return true;
  } else {
    return camera_matrices_ptr_->matrices_ready;
  }
}

bool CameraData::is_image_encoding_supported() const
{
  return image_msg_->encoding == supported_image_encoding_;
}

cudaError_t CameraData::sync_cuda_stream()
{
  return cudaStreamSynchronize(stream_);
}

}  // namespace autoware::bevfusion
