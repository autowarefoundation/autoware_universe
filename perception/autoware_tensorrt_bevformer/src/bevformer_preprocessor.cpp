// Copyright 2025 The Autoware Contributors
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
//
/*
 * Copyright (c) 2025 Multicoreware, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "bevformer_preprocessor.hpp"

#include <Eigen/LU>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <memory>
#include <vector>

namespace autoware
{
namespace tensorrt_bevformer
{

BEVFormerPreprocessor::BEVFormerPreprocessor(const rclcpp::Logger & logger, rclcpp::Node * node)
: logger_(logger)
{
  auto mean_param = node->get_parameter("data_params.mean").as_double_array();
  mean_.assign(mean_param.begin(), mean_param.end());

  auto std_param = node->get_parameter("data_params.std").as_double_array();
  std_.assign(std_param.begin(), std_param.end());

  to_rgb_ = node->get_parameter("data_params.to_rgb").as_bool();

  pad_divisor_ = node->get_parameter("data_params.pad_divisor").as_int();

  scale_factor_ = static_cast<float>(node->get_parameter("data_params.scale_factor").as_double());

  try {
    pipeline_ = std::make_unique<bevformer::preprocessing::BEVPreprocessingPipeline>(
      mean_, std_, to_rgb_, pad_divisor_, scale_factor_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to create preprocessing pipeline: %s", e.what());
    throw;
  }
}

std::vector<cv::Mat> BEVFormerPreprocessor::createLidar2ImgMatrices(
  const std::vector<Eigen::Matrix3d> & sensor2lidar_rotation,
  const std::vector<Eigen::Vector3d> & sensor2lidar_translation,
  const std::vector<Eigen::Matrix3d> & cam_intrinsics)
{
  std::vector<cv::Mat> lidar2img_matrices;

  // Check input sizes
  if (
    sensor2lidar_rotation.size() != 6 || sensor2lidar_translation.size() != 6 ||
    cam_intrinsics.size() != 6) {
    RCLCPP_ERROR(
      logger_, "Invalid input sizes for createLidar2ImgMatrices: %zu, %zu, %zu",
      sensor2lidar_rotation.size(), sensor2lidar_translation.size(), cam_intrinsics.size());
    return lidar2img_matrices;
  }

  // Calculate transformation matrices for each camera
  for (size_t i = 0; i < sensor2lidar_rotation.size(); ++i) {
    Eigen::Matrix3d lidar2cam_r = sensor2lidar_rotation[i].inverse();
    Eigen::Vector3d lidar2cam_t = sensor2lidar_translation[i].transpose() * lidar2cam_r.transpose();

    // Create 4x4 lidar2cam matrix
    Eigen::Matrix4d lidar2cam_rt = Eigen::Matrix4d::Identity();
    lidar2cam_rt.block<3, 3>(0, 0) = lidar2cam_r.transpose();
    lidar2cam_rt.block<3, 1>(0, 3) = Eigen::Vector3d::Zero();
    lidar2cam_rt.block<1, 3>(3, 0) = -lidar2cam_t.transpose();
    lidar2cam_rt(3, 3) = 1.0;

    // Create viewpad with intrinsics
    Eigen::Matrix4d viewpad = Eigen::Matrix4d::Identity();
    viewpad.block<3, 3>(0, 0) = cam_intrinsics[i];

    Eigen::Matrix4d lidar2img_rt = viewpad * lidar2cam_rt.transpose();

    // Convert to OpenCV matrix (float, as OpenCV expects CV_32F)
    cv::Mat lidar2img_cv = cv::Mat::zeros(4, 4, CV_32F);
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        lidar2img_cv.at<float>(row, col) = static_cast<float>(lidar2img_rt(row, col));
      }
    }

    lidar2img_matrices.push_back(lidar2img_cv);
  }

  return lidar2img_matrices;
}

std::vector<float> BEVFormerPreprocessor::flattenLidar2ImgMatrices(
  const std::vector<cv::Mat> & lidar2img_matrices)
{
  std::vector<float> lidar2img_flat;
  lidar2img_flat.reserve(6 * 4 * 4);  // 6 cameras, 4x4 matrices

  for (const auto & mat : lidar2img_matrices) {
    if (mat.rows != 4 || mat.cols != 4) {
      RCLCPP_ERROR(logger_, "Invalid lidar2img matrix size: %dx%d", mat.rows, mat.cols);
      continue;
    }

    for (int i = 0; i < mat.rows; ++i) {
      for (int j = 0; j < mat.cols; ++j) {
        lidar2img_flat.push_back(mat.at<float>(i, j));
      }
    }
  }

  return lidar2img_flat;
}

BEVFormerStructuredInput BEVFormerPreprocessor::preprocessImages(
  const std::vector<cv::Mat> & raw_images,
  const std::vector<Eigen::Matrix3d> & sensor2lidar_rotation,
  const std::vector<Eigen::Vector3d> & sensor2lidar_translation,
  const std::vector<Eigen::Matrix3d> & cam_intrinsics)
{
  BEVFormerStructuredInput result;

  // Check if we have 6 images
  if (raw_images.size() != 6) {
    RCLCPP_ERROR(
      logger_, "Expected 6 images in bevformer preprocessor, got %zu", raw_images.size());
    return result;
  }

  // Create lidar2img matrices
  std::vector<cv::Mat> lidar2img_matrices =
    createLidar2ImgMatrices(sensor2lidar_rotation, sensor2lidar_translation, cam_intrinsics);

  if (lidar2img_matrices.size() != 6) {
    RCLCPP_ERROR(logger_, "Failed to create all 6 lidar2img matrices");
    return result;
  }

  // Create metadata for preprocessing
  bevformer::preprocessing::DataDict metadata;
  metadata["mean"] = mean_;
  metadata["std"] = std_;
  metadata["to_rgb"] = std::vector<float>{to_rgb_ ? 1.0f : 0.0f};

  // Run the preprocessing pipeline
  auto processed_results = pipeline_->processImages(raw_images, lidar2img_matrices, metadata);

  // Extract processed data
  std::vector<cv::Mat> processed_images;
  if (std::holds_alternative<std::vector<cv::Mat>>(processed_results["img"])) {
    processed_images = std::get<std::vector<cv::Mat>>(processed_results["img"]);
  } else {
    RCLCPP_ERROR(logger_, "Failed to get processed images from pipeline");
    return result;
  }

  // Get final lidar2img matrices
  std::vector<cv::Mat> final_lidar2img_matrices;
  if (
    processed_results.count("lidar2img") &&
    std::holds_alternative<std::vector<cv::Mat>>(processed_results["lidar2img"])) {
    final_lidar2img_matrices = std::get<std::vector<cv::Mat>>(processed_results["lidar2img"]);
  } else {
    final_lidar2img_matrices = lidar2img_matrices;
  }

  // Create image tensor and flatten lidar2img matrices
  result.img_tensor = data_loader_.createImageTensor(processed_images);
  result.lidar2img_flat = flattenLidar2ImgMatrices(final_lidar2img_matrices);

  return result;
}

}  // namespace tensorrt_bevformer
}  // namespace autoware
