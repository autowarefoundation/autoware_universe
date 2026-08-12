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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_HPP_

#include <opencv2/core/mat.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

enum DiagnosticLevel { OK, WARN, ERROR, STALE };

struct DiagnosticAdditionalData
{
  std::string key;
  std::string value;
};

struct DiagnosticOutput
{
  DiagnosticLevel level;
  std::string message;
  std::vector<DiagnosticAdditionalData> additional_data;
};

/**
 * @brief Quantize a 16-bit image to 8-bit.
 *
 * The values are scaled by `1.0 / 256` to prevent overflow.
 *
 * @param image_16u The input 16-bit image.
 * @return cv::Mat The quantized 8-bit image. The data type is `CV_8UC1`.
 */
cv::Mat quantize_to_8u(const cv::Mat & image_16u);

/**
 * @brief Make a no-return mask from the input depth image.
 *
 * The mask is a binary image where 255 is no-return and 0 is return.
 *
 * @param depth_image The input depth image.
 * @return cv::Mat The no-return mask. The data type is `CV_8UC1`.
 */
cv::Mat make_no_return_mask(const cv::Mat & depth_image);

/**
 * @brief Segments a given mask into two masks, according to the ground/sky segmentation
 * parameters.
 *
 * @param mask The input mask. The data type is `CV_8UC1`.
 * @param horizontal_ring_id The ring ID that separates ground and sky.
 * @return std::pair<cv::Mat, cv::Mat> The pair {ground_mask, sky_mask}. The data type is
 * `CV_8UC1`.
 */
std::pair<cv::Mat, cv::Mat> segment_into_ground_and_sky(
  const cv::Mat & mask, int horizontal_ring_id);

/**
 * @brief Validate that the PointCloud2 message has required fields for blockage diagnosis.
 *
 * @param input The input point cloud.
 * @throws std::runtime_error if any required field is missing.
 */
void validate_pointcloud_fields(const sensor_msgs::msg::PointCloud2 & input);

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__BLOCKAGE_DIAG__BLOCKAGE_DIAG_HPP_
