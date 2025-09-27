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

#include "bevformer_data_loader.hpp"

#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <iostream>
#include <numeric>
#include <vector>

namespace autoware
{
namespace tensorrt_bevformer
{

BEVFormerDataLoader::BEVFormerDataLoader() = default;

cv::Mat BEVFormerDataLoader::createImageTensor(const std::vector<cv::Mat> & images)
{
  if (images.size() != 6) {
    RCLCPP_ERROR(
      rclcpp::get_logger("bevformer_data_loader"), "Expected 6 images, got %zu", images.size());
    return cv::Mat();
  }

  // Get dimensions from first image (now in CHW format)
  const cv::Mat & first_img = images[0];
  const int channels = first_img.size[0];
  const int img_h = first_img.size[1];
  const int img_w = first_img.size[2];
  const size_t batch_size = 1;
  const size_t num_cams = 6;
  const size_t total_elements = batch_size * num_cams * channels * img_h * img_w;

  // Validate input images
  for (size_t i = 0; i < images.size(); ++i) {
    if (images[i].dims != 3) {  // Check for 3D tensor (CHW format)
      RCLCPP_ERROR(
        rclcpp::get_logger("bevformer_data_loader"),
        "Image %zu is not in CHW format (expected 3 dimensions)", i);
      return cv::Mat();
    }
    if (images[i].size[0] != channels || images[i].size[1] != img_h || images[i].size[2] != img_w) {
      RCLCPP_ERROR(
        rclcpp::get_logger("bevformer_data_loader"),
        "Image %zu has wrong dimensions: [%d, %d, %d], expected [%d, %d, %d]", i, images[i].size[0],
        images[i].size[1], images[i].size[2], channels, img_h, img_w);
      return cv::Mat();
    }
  }

  // Create output tensor
  cv::Mat tensor(1, total_elements, CV_32F);
  float * tensor_data = tensor.ptr<float>();

  // Fill tensor with correct memory layout [1, 6, 3, H, W]
  for (size_t cam_idx = 0; cam_idx < num_cams; cam_idx++) {
    const cv::Mat & img = images[cam_idx];

    // Since input is already in CHW format, we can copy directly
    size_t cam_offset = cam_idx * channels * img_h * img_w;
    const float * img_data = img.ptr<float>();

    // Copy the entire camera's data at once
    std::memcpy(tensor_data + cam_offset, img_data, channels * img_h * img_w * sizeof(float));
  }

  return tensor;
}

}  // namespace tensorrt_bevformer
}  // namespace autoware
