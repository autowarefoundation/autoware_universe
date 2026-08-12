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

// cspell:ignore BEVFORMER

#ifndef BEVFORMER_PREPROCESSOR_HPP_
#define BEVFORMER_PREPROCESSOR_HPP_

#include "bevformer_data_loader.hpp"
#include "preprocessing/preprocessing_pipeline.hpp"

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <memory>
#include <vector>

namespace autoware
{
namespace tensorrt_bevformer
{

/**
 * @brief Structure to hold preprocessed data for BEVFormer inference
 */
struct BEVFormerStructuredInput
{
  cv::Mat img_tensor;                 // Processed image tensor in CHW format
  std::vector<float> lidar2img_flat;  // Flattened lidar2img matrices
};

class BEVFormerPreprocessor
{
public:
  /**
   * @brief Constructor
   * @param logger ROS logger for messages
   * @param node ROS node for parameter access
   */
  BEVFormerPreprocessor(const rclcpp::Logger & logger, rclcpp::Node * node);

  /**
   * @brief Process raw images and prepare them for inference
   * @param raw_images Vector of 6 raw camera images
   * @param sensor2lidar_rotation Rotation matrices for each camera
   * @param sensor2lidar_translation Translation vectors for each camera
   * @param cam_intrinsics Camera intrinsic matrices
   * @return Preprocessed data ready for inference
   */
  BEVFormerStructuredInput preprocessImages(
    const std::vector<cv::Mat> & raw_images,
    const std::vector<Eigen::Matrix3d> & sensor2lidar_rotation,
    const std::vector<Eigen::Vector3d> & sensor2lidar_translation,
    const std::vector<Eigen::Matrix3d> & cam_intrinsics);

private:
  /**
   * @brief Create lidar2img matrices from sensor calibration data
   */
  std::vector<cv::Mat> createLidar2ImgMatrices(
    const std::vector<Eigen::Matrix3d> & sensor2lidar_rotation,
    const std::vector<Eigen::Vector3d> & sensor2lidar_translation,
    const std::vector<Eigen::Matrix3d> & cam_intrinsics);

  /**
   * @brief Flatten lidar2img matrices for inference
   */
  std::vector<float> flattenLidar2ImgMatrices(const std::vector<cv::Mat> & lidar2img_matrices);

private:
  rclcpp::Logger logger_;
  std::unique_ptr<bevformer::preprocessing::BEVPreprocessingPipeline> pipeline_;
  BEVFormerDataLoader data_loader_;

  std::vector<float> mean_;
  std::vector<float> std_;
  bool to_rgb_;
  int pad_divisor_;
  float scale_factor_;
};

}  // namespace tensorrt_bevformer
}  // namespace autoware

#endif  // BEVFORMER_PREPROCESSOR_HPP_
