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

#include "autoware/calibration_status_classifier/calibration_status_classifier.hpp"

#include "autoware/calibration_status_classifier/data_type.hpp"

#include <Eigen/Dense>
#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/cuda_utils.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::calibration_status_classifier
{

CalibrationStatusClassifier::CalibrationStatusClassifier(
  const std::string & onnx_path, const std::string & trt_precision, int64_t cloud_capacity,
  const std::vector<double> & ego_box, const CalibrationStatusClassifierConfig & config)
: cloud_capacity_(static_cast<size_t>(cloud_capacity)), ego_box_(), config_(config)
{
  if (!ego_box.empty() && ego_box.size() != 6) {
    throw std::invalid_argument(
      "ego_box must have exactly 6 elements [x_min, y_min, z_min, x_max, y_max, z_max]");
  }
  if (ego_box.size() == 6) {
    const bool all_zeros =
      std::all_of(ego_box.begin(), ego_box.end(), [](double v) { return v == 0.0; });
    if (!all_zeros) {
      if (ego_box[0] >= ego_box[3] || ego_box[1] >= ego_box[4] || ego_box[2] >= ego_box[5]) {
        throw std::invalid_argument("ego_box min values must be less than max values");
      }
      ego_box_ = ego_box;
    }
  }

  tensorrt_common::TrtCommonConfig trt_config(onnx_path, trt_precision);

  std::vector<autoware::tensorrt_common::NetworkIO> network_io{
    autoware::tensorrt_common::NetworkIO("input", {4, {1, config_.channels, -1, -1}}),
    autoware::tensorrt_common::NetworkIO("output", {2, {1, 2}})};

  std::vector<autoware::tensorrt_common::ProfileDims> profile_dims{
    autoware::tensorrt_common::ProfileDims(
      "input", {4, {1, config_.channels, config_.height.at(0), config_.width.at(0)}},
      {4, {1, config_.channels, config_.height.at(1), config_.width.at(1)}},
      {4, {1, config_.channels, config_.height.at(2), config_.width.at(2)}})};

  auto network_io_ptr =
    std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io);
  auto profile_dims_ptr =
    std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>(profile_dims);

  network_trt_ptr_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(trt_config);
  if (!network_trt_ptr_->setup(std::move(profile_dims_ptr), std::move(network_io_ptr))) {
    throw std::runtime_error("Failed to setup CalibrationStatusClassifier TensorRT engine.");
  }

  in_d_ =
    cuda_utils::make_unique<float[]>(config_.channels * config_.height.at(2) * config_.width.at(2));
  out_d_ = cuda_utils::make_unique<float[]>(2);
  cloud_d_ = cuda_utils::make_unique<InputPointType[]>(cloud_capacity_);
  image_d_ =
    cuda_utils::make_unique<InputImageBGR8Type[]>(config_.height.at(2) * config_.width.at(2));
  image_undistorted_d_ =
    cuda_utils::make_unique<InputImageBGR8Type[]>(config_.height.at(2) * config_.width.at(2));
  dist_coeffs_d_ = cuda_utils::make_unique<double[]>(dist_coeffs_size);
  camera_matrix_d_ = cuda_utils::make_unique<double[]>(camera_matrix_size);
  projection_matrix_d_ = cuda_utils::make_unique<double[]>(projection_matrix_size);
  tf_matrix_d_ = cuda_utils::make_unique<double[]>(tf_matrix_size);
  num_points_projected_d_ = cuda_utils::make_unique<uint32_t>();

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
  preprocess_ptr_ = std::make_unique<PreprocessCuda>(
    config.max_depth, config.dilation_size, config_.width.at(2), config_.height.at(2), stream_);
}

CalibrationStatusClassifierResult CalibrationStatusClassifier::process(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const CameraLidarInfo & camera_lidar_info, uint8_t * preview_img_data)
{
  auto t1 = std::chrono::steady_clock::now();

  // Prepare data
  cuda_utils::clear_async(in_d_.get(), image_msg->height * image_msg->width, stream_);
  cuda_utils::clear_async(out_d_.get(), 2, stream_);
  cuda_utils::clear_async(cloud_d_.get(), cloud_capacity_, stream_);
  cuda_utils::clear_async(image_d_.get(), image_msg->height * image_msg->width, stream_);
  cuda_utils::clear_async(
    image_undistorted_d_.get(), image_msg->height * image_msg->width, stream_);
  cuda_utils::clear_async(dist_coeffs_d_.get(), dist_coeffs_size, stream_);
  cuda_utils::clear_async(camera_matrix_d_.get(), camera_matrix_size, stream_);
  cuda_utils::clear_async(projection_matrix_d_.get(), projection_matrix_size, stream_);
  cuda_utils::clear_async(tf_matrix_d_.get(), tf_matrix_size, stream_);
  cuda_utils::clear_async(num_points_projected_d_.get(), 1, stream_);

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    cloud_d_.get(), cloud_msg->data.data(),
    sizeof(InputPointType) * cloud_msg->width * cloud_msg->height, cudaMemcpyHostToDevice,
    stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    image_d_.get(), image_msg->data.data(),
    sizeof(InputImageBGR8Type) * image_msg->height * image_msg->width, cudaMemcpyHostToDevice,
    stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    dist_coeffs_d_.get(), camera_lidar_info.d.data(), sizeof(double) * dist_coeffs_size,
    cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    camera_matrix_d_.get(), camera_lidar_info.k.data(), sizeof(double) * camera_matrix_size,
    cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    projection_matrix_d_.get(), camera_lidar_info.p.data(), sizeof(double) * projection_matrix_size,
    cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    tf_matrix_d_.get(), camera_lidar_info.tf_camera_to_lidar.data(),
    sizeof(double) * tf_matrix_size, cudaMemcpyHostToDevice, stream_));

  // Undistort image or just copy with appropiate memory pattern
  if (camera_lidar_info.to_undistort) {
    CHECK_CUDA_ERROR(preprocess_ptr_->undistort_image_launch(
      image_d_.get(), dist_coeffs_d_.get(), camera_matrix_d_.get(), projection_matrix_d_.get(),
      image_msg->width, image_msg->height, image_undistorted_d_.get(), in_d_.get()));
  } else {
    CHECK_CUDA_ERROR(preprocess_ptr_->copy_image_launch(
      image_d_.get(), image_msg->width, image_msg->height, image_undistorted_d_.get(),
      in_d_.get()));
  }

  // Generate ego occlusion mask on first call per pair (lazy init)
  const uint8_t * ego_mask_ptr = nullptr;
  if (!ego_box_.empty()) {
    const std::string mask_key =
      camera_lidar_info.camera_frame_id + "_" + camera_lidar_info.lidar_frame_id + "_" +
      std::to_string(image_msg->width) + "x" + std::to_string(image_msg->height);

    auto it = ego_masks_d_.find(mask_key);
    if (it == ego_masks_d_.end()) {  // lazy init, only generate mask if not already stored
      auto mask =
        generate_ego_occlusion_mask(camera_lidar_info, image_msg->width, image_msg->height);
      auto mask_d = cuda_utils::make_unique<uint8_t[]>(image_msg->width * image_msg->height);
      CHECK_CUDA_ERROR(cudaMemcpyAsync(
        mask_d.get(), mask.data(), sizeof(uint8_t) * image_msg->width * image_msg->height,
        cudaMemcpyHostToDevice, stream_));
      it = ego_masks_d_.emplace(mask_key, std::move(mask_d)).first;
    }
    ego_mask_ptr = it->second.get();
  }

  // Project points
  CHECK_CUDA_ERROR(preprocess_ptr_->project_points_launch(
    cloud_d_.get(), image_undistorted_d_.get(), tf_matrix_d_.get(), projection_matrix_d_.get(),
    cloud_msg->width * cloud_msg->height, image_msg->width, image_msg->height, in_d_.get(),
    num_points_projected_d_.get(), ego_mask_ptr));
  uint32_t num_points_projected = 0;
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &num_points_projected, num_points_projected_d_.get(), sizeof(uint32_t), cudaMemcpyDeviceToHost,
    stream_));
  if (preview_img_data != nullptr) {
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      preview_img_data, image_undistorted_d_.get(),
      sizeof(InputImageBGR8Type) * image_msg->height * image_msg->width, cudaMemcpyDeviceToHost,
      stream_));
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  auto t2 = std::chrono::steady_clock::now();

  // Run inference
  network_trt_ptr_->setTensor(
    "input", in_d_.get(),
    {4,
     {1, config_.channels, static_cast<int32_t>(image_msg->height),
      static_cast<int32_t>(image_msg->width)}});
  network_trt_ptr_->setTensor("output", out_d_.get());
  network_trt_ptr_->enqueueV3(stream_);
  std::vector<float> output(2);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    output.data(), out_d_.get(), sizeof(float) * 2, cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  auto t3 = std::chrono::steady_clock::now();

  // Process output
  auto miscalibration_confidence = output.at(0);
  auto calibration_confidence = output.at(1);
  auto time_preproc_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
  auto time_inference_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();

  CalibrationStatusClassifierResult result{};
  result.calibration_confidence = calibration_confidence;
  result.miscalibration_confidence = miscalibration_confidence;
  result.preprocessing_time_ms = static_cast<double>(time_preproc_us) * 1e-3;
  result.inference_time_ms = static_cast<double>(time_inference_us) * 1e-3;
  result.num_points_projected = num_points_projected;
  return result;
}

std::vector<uint8_t> CalibrationStatusClassifier::generate_ego_occlusion_mask(
  const CameraLidarInfo & camera_lidar_info, std::size_t image_width,
  std::size_t image_height) const
{
  constexpr double occlusion_adjust_margin = 0.01;

  std::vector<uint8_t> mask(image_width * image_height, 1);

  if (ego_box_.empty()) {
    return mask;
  }

  // Extract rotation and translation from the lidar-to-camera transform (row-major)
  const auto & tf = camera_lidar_info.tf_camera_to_lidar;
  Eigen::Matrix3d R = tf.block<3, 3>(0, 0);
  Eigen::Vector3d t = tf.block<3, 1>(0, 3);

  // Camera center in lidar frame: c = -R^T * t
  Eigen::Vector3d camera_center_lidar = -R.transpose() * t;

  // Prepare box bounds (copy to allow adjustment)
  Eigen::Vector3d box_min(ego_box_[0], ego_box_[1], ego_box_[2]);
  Eigen::Vector3d box_max(ego_box_[3], ego_box_[4], ego_box_[5]);

  // Adjust box if camera is inside it (shrink closest X/Y wall)
  if (
    (camera_center_lidar.array() >= box_min.array()).all() &&
    (camera_center_lidar.array() <= box_max.array()).all()) {
    Eigen::Vector3d d_min = camera_center_lidar - box_min;
    Eigen::Vector3d d_max = box_max - camera_center_lidar;

    // Only adjust X (0) and Y (1) axes, leave Z untouched
    for (int i = 0; i < 2; ++i) {
      if (d_min[i] < d_max[i]) {
        box_min[i] = camera_center_lidar[i] + occlusion_adjust_margin;
      } else {
        box_max[i] = camera_center_lidar[i] - occlusion_adjust_margin;
      }
    }
  }

  // Build back-projection: for each pixel (u, v), compute ray direction in lidar frame
  const auto & P = camera_lidar_info.p;
  Eigen::Matrix3d P_3x3 = P.block<3, 3>(0, 0);
  Eigen::Matrix3d P_3x3_inv = P_3x3.inverse();

  // R^T transforms directions from camera frame to lidar frame
  Eigen::Matrix3d R_T = R.transpose();

  for (std::size_t v = 0; v < image_height; ++v) {
    for (std::size_t u = 0; u < image_width; ++u) {
      // Back-project pixel to ray direction in camera frame
      Eigen::Vector3d pixel_hom(static_cast<double>(u), static_cast<double>(v), 1.0);
      Eigen::Vector3d dir_cam = P_3x3_inv * pixel_hom;

      // Transform ray direction to lidar frame
      Eigen::Vector3d dir_lidar = R_T * dir_cam;

      // Slab intersection test with ego box
      double t_enter = -std::numeric_limits<double>::infinity();
      double t_exit = std::numeric_limits<double>::infinity();

      bool valid = true;
      for (int i = 0; i < 3; ++i) {
        if (std::abs(dir_lidar[i]) < 1e-12) {
          if (camera_center_lidar[i] < box_min[i] || camera_center_lidar[i] > box_max[i]) {
            valid = false;
            break;
          }
        } else {
          double inv_d = 1.0 / dir_lidar[i];
          double t1 = (box_min[i] - camera_center_lidar[i]) * inv_d;
          double t2 = (box_max[i] - camera_center_lidar[i]) * inv_d;

          if (t1 > t2) {
            std::swap(t1, t2);
          }

          t_enter = std::max(t_enter, t1);
          t_exit = std::min(t_exit, t2);

          if (t_enter > t_exit) {
            valid = false;
            break;
          }
        }
      }

      // Ray hits the ego box if intersection is valid and box is in front (t_exit >= 0)
      if (valid && t_exit >= 0.0 && t_enter <= t_exit) {
        mask[v * image_width + u] = 0;
      }
    }
  }

  return mask;
}

}  // namespace autoware::calibration_status_classifier
