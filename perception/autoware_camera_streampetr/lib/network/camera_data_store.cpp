// Copyright 2025 TIER IV
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

#include "autoware/camera_streampetr/network/camera_data_store.hpp"

#include "autoware/camera_streampetr/network/preprocess.hpp"

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/opencv.hpp>
/* `#include <Eigen/Dense>` is including the Eigen library's Dense module. Eigen is a C++ template
library for linear algebra: matrices, vectors, numerical solvers, and related algorithms. The Dense
module provides classes and functions for dense matrices and vectors, as well as various linear
algebra operations. */
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::camera_streampetr
{
static void updateIntrinsics(float * K_4x4, const Eigen::Matrix3f & ida_mat)
{
  Eigen::Matrix3f K;
  K << K_4x4[0], K_4x4[1], K_4x4[2], K_4x4[4], K_4x4[5], K_4x4[6], K_4x4[8], K_4x4[9], K_4x4[10];

  Eigen::Matrix3f K_new = ida_mat * K;

  K_4x4[0] = K_new(0, 0);  // fx'
  K_4x4[1] = K_new(0, 1);
  K_4x4[2] = K_new(0, 2);
  K_4x4[4] = K_new(1, 0);
  K_4x4[5] = K_new(1, 1);  // fy'
  K_4x4[6] = K_new(1, 2);
  K_4x4[8] = K_new(2, 0);
  K_4x4[9] = K_new(2, 1);
  K_4x4[10] = K_new(2, 2);
}

CameraDataStore::CameraDataStore(
  rclcpp::Node * node, const int rois_number, const int image_height, const int image_width,
  const int anchor_camera_id, const bool is_distorted_image, const double downsample_factor)
: rois_number_(rois_number),
  image_height_(image_height),
  image_width_(image_width),
  anchor_camera_id_(anchor_camera_id),
  preprocess_time_ms_(0.0f),
  is_distorted_image_(is_distorted_image),
  downsample_factor_(downsample_factor),
  logger_(node->get_logger())
{
  image_input_ = std::make_shared<Tensor>(
    "image_input", nvinfer1::Dims{5, 1, rois_number, 3, image_height, image_width},
    nvinfer1::DataType::kFLOAT);  // {num_dims, batch_size, rois_number, num_channels, height,
                                  // width}

  image_input_mean_ =
    std::make_shared<Tensor>("image_input_mean", nvinfer1::Dims{1, 3}, nvinfer1::DataType::kFLOAT);
  image_input_mean_->load_from_vector({103.530, 116.280, 123.675});
  image_input_std_ =
    std::make_shared<Tensor>("image_input_std", nvinfer1::Dims{1, 3}, nvinfer1::DataType::kFLOAT);
  image_input_std_->load_from_vector({57.375, 57.120, 58.395});

  camera_image_timestamp_ = std::vector<double>(rois_number, -1.0);
  camera_link_names_ = std::vector<std::string>(rois_number, "");
  start_timestamp_ = -1.0;

  camera_info_list_ = std::vector<CameraInfo::ConstSharedPtr>(rois_number, nullptr);

  streams_.resize(rois_number);
  for (int i = 0; i < rois_number; ++i) {
    cudaStreamCreate(&streams_[i]);
  }

  is_frozen_ = false;
  active_updates_ = 0;

  if (is_distorted_image_ && (downsample_factor_ <= 0.0 || downsample_factor_ > 1.0))
    throw std::runtime_error(
      "downsample_factor must be in range (0,1] when is_distorted_image is true");
}

void CameraDataStore::update_camera_image(
  const int camera_id, const Image::ConstSharedPtr & input_camera_image_msg)
{
  {
    std::unique_lock<std::mutex> lock(freeze_mutex_);
    freeze_cv_.wait(lock, [&]() { return !is_frozen_; });  // Wait if frozen
    ++active_updates_;
  }

  auto start_time = std::chrono::high_resolution_clock::now();
  int original_height = input_camera_image_msg->height;
  int original_width = input_camera_image_msg->width;

  const float scaleH = static_cast<float>(image_height_) / static_cast<float>(original_height);
  const float scaleW = static_cast<float>(image_width_) / static_cast<float>(original_width);
  const float resize = std::max(scaleH, scaleW);

  const int newW = static_cast<int>(original_width * resize);
  const int newH = static_cast<int>(original_height * resize);
  float bottom_crop_portion = 0.0f;  // what portion of bottom to crop. We only crop at the top
                                     // along height, so hardcoded to 0.0f
  int crop_h = static_cast<int>((1.0f - bottom_crop_portion) * newH) - image_height_;
  if (crop_h < 0) crop_h = 0;
  int crop_w = std::max(0, (newW - image_width_) / 2);

  int start_x = std::max(0, crop_w);
  int start_y = std::max(0, crop_h);

  const int camera_offset = camera_id * 3 * image_height_ * image_width_;

  std::unique_ptr<Tensor> image_input_tensor;
  if (is_distorted_image_ && camera_info_list_[camera_id]) {
    auto camera_info = camera_info_list_[camera_id];

    // Create camera matrix K from camera_info
    cv::Mat K =
      (cv::Mat_<double>(3, 3) << camera_info->k[0], camera_info->k[1], camera_info->k[2],
       camera_info->k[3], camera_info->k[4], camera_info->k[5], camera_info->k[6],
       camera_info->k[7], camera_info->k[8]);

    // Create distortion coefficients matrix D from camera_info
    const auto & d_vec = camera_info->d;
    cv::Mat D(1, static_cast<int>(d_vec.size()), CV_64F);
    for (size_t i = 0; i < d_vec.size(); ++i) {
      D.at<double>(0, static_cast<int>(i)) = d_vec[i];
    }

    // Create projection matrix P from camera_info (first 3x3 part)
    cv::Mat P =
      (cv::Mat_<double>(3, 3) << camera_info->p[0], camera_info->p[1], camera_info->p[2],
       camera_info->p[4], camera_info->p[5], camera_info->p[6], camera_info->p[8],
       camera_info->p[9], camera_info->p[10]);

    P.at<double>(0, 0) *= downsample_factor_;  // fx
    P.at<double>(0, 2) *= downsample_factor_;  // cx
    P.at<double>(1, 1) *= downsample_factor_;  // fy
    P.at<double>(1, 2) *= downsample_factor_;  // cy

    cv::Mat input_image(
      original_height, original_width, CV_8UC3,
      const_cast<uint8_t *>(input_camera_image_msg->data.data()));

    original_height = static_cast<int>(original_height * downsample_factor_);
    original_width = static_cast<int>(original_width * downsample_factor_);

    cv::Mat undistort_map_x, undistort_map_y;
    cv::initUndistortRectifyMap(
      K, D, cv::Mat(), P, cv::Size(original_width, original_height), CV_32FC1, undistort_map_x,
      undistort_map_y);

    cv::Mat undistorted_image;

    cv::remap(input_image, undistorted_image, undistort_map_x, undistort_map_y, cv::INTER_LANCZOS4);

    image_input_tensor = std::make_unique<Tensor>(
      "camera_img", nvinfer1::Dims{3, original_height, original_width, 3},
      nvinfer1::DataType::kUINT8);
    cudaMemcpyAsync(
      image_input_tensor->ptr, undistorted_image.data, image_input_tensor->nbytes(),
      cudaMemcpyHostToDevice, streams_.at(camera_id));

  } else {
    image_input_tensor = std::make_unique<Tensor>(
      "camera_img", nvinfer1::Dims{3, original_height, original_width, 3},
      nvinfer1::DataType::kUINT8);
    cudaMemcpyAsync(
      image_input_tensor->ptr, input_camera_image_msg->data.data(), image_input_tensor->nbytes(),
      cudaMemcpyHostToDevice, streams_.at(camera_id));
  }

  auto err = resizeAndExtractRoi_launch(
    static_cast<std::uint8_t *>(image_input_tensor->ptr), static_cast<float *>(image_input_->ptr),
    camera_offset, original_height, original_width, newH, newW, image_height_, image_width_,
    start_y, start_x, static_cast<const float *>(image_input_mean_->ptr),
    static_cast<const float *>(image_input_std_->ptr), streams_.at(camera_id));

  if (err != cudaSuccess) {
    RCLCPP_ERROR(
      logger_, "resizeAndExtractRoi_launch failed with error: %s", cudaGetErrorString(err));
  }

  camera_image_timestamp_[camera_id] =
    input_camera_image_msg->header.stamp.sec + input_camera_image_msg->header.stamp.nanosec * 1e-9;
  camera_link_names_[camera_id] = input_camera_image_msg->header.frame_id;

  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  preprocess_time_ms_ = duration.count();

  // // Save the processed image for debugging during development
  // std::string filename = "/home/autoware/workspace/StreamPETR_TensorRT_ROS2/data/camera_" +
  // std::to_string(camera_id) + ".jpg"; save_processed_image(camera_id, filename);

  {
    std::lock_guard<std::mutex> lock(freeze_mutex_);
    --active_updates_;
    if (is_frozen_ && active_updates_ == 0)
      freeze_cv_.notify_all();  // Notify freeze_updates() to continue
  }
}

void CameraDataStore::update_camera_info(
  const int camera_id, const CameraInfo::ConstSharedPtr & input_camera_info_msg)
{
  camera_info_list_[camera_id] = input_camera_info_msg;
}

bool CameraDataStore::check_if_all_camera_info_received() const
{
  for (const auto & camera_info : camera_info_list_) {
    if (!camera_info) return false;
  }

  return true;
}

bool CameraDataStore::check_if_all_camera_image_received() const
{
  for (const auto & camera_info_timestamp : camera_image_timestamp_) {
    if (camera_info_timestamp < 0) return false;
  }
  return true;
}

float CameraDataStore::check_if_all_images_synced() const
{
  if (!check_if_all_camera_info_received() || !check_if_all_camera_image_received()) return -1.0;

  double min_time = std::numeric_limits<double>::max();
  double max_time = std::numeric_limits<double>::min();

  for (size_t camera_id = 0; camera_id < camera_image_timestamp_.size(); ++camera_id) {
    if (camera_image_timestamp_[camera_id] < min_time) {
      min_time = camera_image_timestamp_[camera_id];
    }
    if (camera_image_timestamp_[camera_id] > max_time) {
      max_time = camera_image_timestamp_[camera_id];
    }
  }
  return max_time - min_time;
}

std::vector<float> CameraDataStore::get_camera_info_vector() const
{
  std::vector<float> intrinsics_all;

  int fH = image_height_;
  int fW = image_width_;

  for (size_t camera_id = 0; camera_id < camera_info_list_.size(); ++camera_id) {
    const auto & camera_info_msg = camera_info_list_[camera_id];
    if (!camera_info_msg) {
      throw std::runtime_error(
        "CameraInfo not received for camera ID: " + std::to_string(camera_id));
    }

    int rawW = camera_info_msg->width;
    int rawH = camera_info_msg->height;

    std::vector<float> K_4x4 = {
      static_cast<float>(camera_info_msg->p[0]),
      static_cast<float>(camera_info_msg->p[1]),
      static_cast<float>(camera_info_msg->p[2]),
      static_cast<float>(camera_info_msg->p[3]),
      static_cast<float>(camera_info_msg->p[4]),
      static_cast<float>(camera_info_msg->p[5]),
      static_cast<float>(camera_info_msg->p[6]),
      static_cast<float>(camera_info_msg->p[7]),
      static_cast<float>(camera_info_msg->p[8]),
      static_cast<float>(camera_info_msg->p[9]),
      static_cast<float>(camera_info_msg->p[10]),
      static_cast<float>(camera_info_msg->p[11]),
      0.f,
      0.f,
      0.f,
      1.f};

    float resize = std::max(
      static_cast<float>(fH) / static_cast<float>(rawH),
      static_cast<float>(fW) / static_cast<float>(rawW));
    int newW = static_cast<int>(rawW * resize);
    int newH = static_cast<int>(rawH * resize);
    int crop_h = static_cast<int>(newH) - fH;
    int crop_w = std::max(0, newW - fW) / 2;

    Eigen::Matrix3f S = Eigen::Matrix3f::Identity();
    S(0, 0) = resize;
    S(1, 1) = resize;

    Eigen::Matrix3f T = Eigen::Matrix3f::Identity();
    T(0, 2) = -static_cast<float>(crop_w);
    T(1, 2) = -static_cast<float>(crop_h);

    Eigen::Matrix3f transform_mat = T * S;

    updateIntrinsics(K_4x4.data(), transform_mat);

    intrinsics_all.insert(intrinsics_all.end(), K_4x4.begin(), K_4x4.end());
  }

  return intrinsics_all;
}

float CameraDataStore::get_preprocess_time_ms() const
{
  return preprocess_time_ms_;
}

std::vector<float> CameraDataStore::get_image_shape() const
{
  std::vector<float> vec{static_cast<float>(image_height_), static_cast<float>(image_width_), 3.0f};
  return vec;
}

std::shared_ptr<cuda::Tensor> CameraDataStore::get_image_input() const
{
  return image_input_;
}

float CameraDataStore::get_timestamp()
{
  const float time_difference = camera_image_timestamp_[anchor_camera_id_] - start_timestamp_;

  if (start_timestamp_ < 0.0 || time_difference > MAX_PERMISSIONED_CAMERA_TIME_DIFF) {
    start_timestamp_ = camera_image_timestamp_[anchor_camera_id_];
    return 0.0;
  }

  return time_difference;
}

std::vector<std::string> CameraDataStore::get_camera_link_names() const
{
  return camera_link_names_;
}

void CameraDataStore::restart()
{
  start_timestamp_ = -1.0;
  camera_image_timestamp_.assign(rois_number_, -1.0);
  camera_link_names_.assign(rois_number_, "");
}

void CameraDataStore::save_processed_image(const int camera_id, const std::string & filename) const
{
  // Check if camera_id is valid
  if (camera_id < 0 || camera_id >= static_cast<int>(rois_number_)) {
    RCLCPP_ERROR(logger_, "Invalid camera_id: %d", camera_id);
    return;
  }

  // Calculate the offset for this camera in the image_input_ tensor
  const int camera_offset = camera_id * 3 * image_height_ * image_width_;
  const int image_size = 3 * image_height_ * image_width_;

  // Allocate CPU memory for the processed image data
  std::vector<float> cpu_image_data(image_size);

  // Copy the processed image data from GPU to CPU
  cudaError_t err = cudaMemcpyAsync(
    cpu_image_data.data(), static_cast<const float *>(image_input_->ptr) + camera_offset,
    image_size * sizeof(float), cudaMemcpyDeviceToHost, streams_.at(camera_id));

  if (err != cudaSuccess) {
    RCLCPP_ERROR(logger_, "Failed to copy image data from GPU to CPU: %s", cudaGetErrorString(err));
    return;
  }

  // Synchronize the CUDA stream to ensure the copy is complete
  cudaStreamSynchronize(streams_.at(camera_id));

  // Create OpenCV Mat from the float data
  cv::Mat processed_image(image_height_, image_width_, CV_32FC3);

  const std::vector<float> image_input_std = {57.375, 57.120, 58.395};
  const std::vector<float> image_input_mean = {103.530, 116.280, 123.675};
  // Copy data to OpenCV Mat (data is in CHW format, need to convert to HWC)
  for (int h = 0; h < image_height_; ++h) {
    for (int w = 0; w < image_width_; ++w) {
      for (int c = 0; c < 3; ++c) {
        processed_image.at<cv::Vec3f>(h, w)[c] =
          cpu_image_data[c * image_height_ * image_width_ + h * image_width_ + w] *
            image_input_std[c] +
          image_input_mean[c];
      }
    }
  }

  // Convert from float (0-255) to uint8 (0-255)
  cv::Mat uint8_image;
  processed_image.convertTo(uint8_image, CV_8UC3);

  // Create directory if it doesn't exist
  std::filesystem::path file_path(filename);
  std::filesystem::path dir_path = file_path.parent_path();
  if (!std::filesystem::exists(dir_path)) {
    std::filesystem::create_directories(dir_path);
  }

  // Save the image
  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  compression_params.push_back(95);  // High quality JPEG

  bool success = cv::imwrite(filename, uint8_image, compression_params);

  if (success) {
    RCLCPP_INFO(logger_, "Processed image for camera %d saved to: %s", camera_id, filename.c_str());
  } else {
    RCLCPP_ERROR(
      logger_, "Failed to save processed image for camera %d to: %s", camera_id, filename.c_str());
  }
}

void CameraDataStore::freeze_updates()
{
  std::unique_lock<std::mutex> lock(freeze_mutex_);
  is_frozen_ = true;
  freeze_cv_.wait(lock, [&]() { return active_updates_ == 0; });
}

void CameraDataStore::unfreeze_updates()
{
  std::unique_lock<std::mutex> lock(freeze_mutex_);
  is_frozen_ = false;
  freeze_cv_.notify_all();  // Let blocked A()s continue
}

}  // namespace autoware::camera_streampetr
