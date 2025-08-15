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

#include "autoware/calibration_status/calibration_status.hpp"

#include "autoware/calibration_status/data_type.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/cuda_utils.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::calibration_status
{
constexpr size_t image_width = 2880;
constexpr size_t image_height = 1860;
constexpr size_t num_features = 5;
constexpr size_t max_cloud_size = 1'000'000;

CalibrationStatus::CalibrationStatus(
  const std::string & onnx_path, const CalibrationStatusConfig & config)
: config_(config)
{
  tensorrt_common::TrtCommonConfig trt_config(onnx_path);

  std::vector<autoware::tensorrt_common::NetworkIO> network_io{
    autoware::tensorrt_common::NetworkIO(
      "input", {4, {1, num_features, image_height, image_width}}),
    autoware::tensorrt_common::NetworkIO("output", {2, {1, 2}})};

  std::vector<autoware::tensorrt_common::ProfileDims> profile_dims{
    autoware::tensorrt_common::ProfileDims(
      "input", {4, {1, num_features, image_height, image_width}},
      {4, {1, num_features, image_height, image_width}},
      {4, {1, num_features, image_height, image_width}})};

  auto network_io_ptr =
    std::make_unique<std::vector<autoware::tensorrt_common::NetworkIO>>(network_io);
  auto profile_dims_ptr =
    std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>(profile_dims);

  network_trt_ptr_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(trt_config);
  if (!network_trt_ptr_->setup(std::move(profile_dims_ptr), std::move(network_io_ptr))) {
    throw std::runtime_error("Failed to setup CalibrationStatus TensorRT engine.");
  }

  in_d_ = cuda_utils::make_unique<InputArrayRGBDI[]>(image_height * image_width);
  out_d_ = cuda_utils::make_unique<float[]>(2);
  cloud_d_ = cuda_utils::make_unique<InputPointType[]>(max_cloud_size);
  image_d_ = cuda_utils::make_unique<InputImageBGR8Type[]>(image_height * image_width);
  image_undistorted_d_ = cuda_utils::make_unique<InputImageBGR8Type[]>(image_height * image_width);
  dist_coeffs_d_ = cuda_utils::make_unique<double[]>(8);
  camera_matrix_d_ = cuda_utils::make_unique<double[]>(9);
  projection_matrix_d_ = cuda_utils::make_unique<double[]>(12);
  tf_matrix_d_ = cuda_utils::make_unique<double[]>(16);
  num_points_projected_d_ = cuda_utils::make_unique<uint32_t>();

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
  preprocess_ptr_ = std::make_unique<PreprocessCuda>(stream_);
}

CalibrationStatusResult CalibrationStatus::process(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
  const Eigen::Affine3d & transform, uint8_t * preview_img_data)
{
  auto t1 = std::chrono::steady_clock::now();

  // Prepare data
  cuda_utils::clear_async(in_d_.get(), image_height * image_width, stream_);
  cuda_utils::clear_async(out_d_.get(), 2, stream_);
  cuda_utils::clear_async(cloud_d_.get(), 1'000'000, stream_);
  cuda_utils::clear_async(image_d_.get(), image_height * image_width, stream_);
  cuda_utils::clear_async(image_undistorted_d_.get(), image_height * image_width, stream_);
  cuda_utils::clear_async(dist_coeffs_d_.get(), 8, stream_);
  cuda_utils::clear_async(camera_matrix_d_.get(), 9, stream_);
  cuda_utils::clear_async(projection_matrix_d_.get(), 12, stream_);
  cuda_utils::clear_async(tf_matrix_d_.get(), 16, stream_);
  cuda_utils::clear_async(num_points_projected_d_.get(), 1, stream_);

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    cloud_d_.get(), cloud_msg->data.data(),
    sizeof(InputPointType) * cloud_msg->width * cloud_msg->height, cudaMemcpyHostToDevice,
    stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    image_d_.get(), image_msg->data.data(), sizeof(InputImageBGR8Type) * image_height * image_width,
    cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    dist_coeffs_d_.get(), camera_info_msg->d.data(), sizeof(double) * 8, cudaMemcpyHostToDevice,
    stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    camera_matrix_d_.get(), camera_info_msg->k.data(), sizeof(double) * 9, cudaMemcpyHostToDevice,
    stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    projection_matrix_d_.get(), camera_info_msg->p.data(), sizeof(double) * 12,
    cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    tf_matrix_d_.get(), transform.data(), sizeof(double) * 16, cudaMemcpyHostToDevice, stream_));

  // Undistort image
  CHECK_CUDA_ERROR(preprocess_ptr_->undistortImage_launch(
    image_d_.get(), dist_coeffs_d_.get(), camera_matrix_d_.get(), projection_matrix_d_.get(),
    image_msg->width, image_msg->height, image_undistorted_d_.get(), in_d_.get()));

  // Project points
  CHECK_CUDA_ERROR(preprocess_ptr_->projectPoints_launch(
    cloud_d_.get(), image_undistorted_d_.get(), tf_matrix_d_.get(), projection_matrix_d_.get(),
    cloud_msg->width * cloud_msg->height, image_msg->width, image_msg->height, in_d_.get(),
    num_points_projected_d_.get()));
  uint32_t num_points_projected = 0;
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &num_points_projected, num_points_projected_d_.get(), sizeof(uint32_t), cudaMemcpyDeviceToHost,
    stream_));
  if (preview_img_data != nullptr) {
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      preview_img_data, image_undistorted_d_.get(),
      sizeof(InputImageBGR8Type) * image_height * image_width, cudaMemcpyDeviceToHost, stream_));
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  auto t2 = std::chrono::steady_clock::now();

  // Run inference
  network_trt_ptr_->setTensor(
    "input", in_d_.get(), {4, {1, num_features, image_height, image_width}});
  network_trt_ptr_->setTensor("output", out_d_.get());
  network_trt_ptr_->enqueueV3(stream_);
  std::vector<float> output(2);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    output.data(), out_d_.get(), sizeof(float) * 2, cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  auto t3 = std::chrono::steady_clock::now();

  // Process output
  auto miscalib_confidence = output.at(0);
  auto calib_confidence = output.at(1);
  auto is_calibrated = (calib_confidence > miscalib_confidence);
  auto time_preproc_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
  auto time_inference_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();

  CalibrationStatusResult result{};
  result.is_calibrated = is_calibrated;
  result.calibration_confidence = calib_confidence;
  result.miscalibration_confidence = miscalib_confidence;
  result.preprocessing_time_ms = static_cast<double>(time_preproc_us) * 1e-3;
  result.inference_time_ms = static_cast<double>(time_inference_us) * 1e-3;
  result.num_points_projected = num_points_projected;
  return result;
}

}  // namespace autoware::calibration_status
