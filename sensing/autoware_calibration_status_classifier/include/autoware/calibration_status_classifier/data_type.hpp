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

#ifndef AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__DATA_TYPE_HPP_
#define AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__DATA_TYPE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace autoware::calibration_status_classifier
{
static constexpr std::size_t dist_coeffs_size = 8;
static constexpr std::size_t camera_matrix_size = 9;
static constexpr std::size_t projection_matrix_size = 12;
static constexpr std::size_t tf_matrix_size = 16;

/**
 * @brief Point type for LiDAR input data
 *
 * Packed structure representing a single LiDAR point with spatial coordinates,
 * intensity value, return type, and channel information. Memory layout is
 * optimized for GPU processing.
 */
struct InputPointType
{
  float x;
  float y;
  float z;
  std::uint8_t intensity;
  std::uint8_t return_type;
  std::uint16_t channel;
} __attribute__((packed));

/**
 * @brief BGR8 image pixel type
 *
 * Packed structure for 8-bit BGR color pixels used in camera image processing.
 * Memory layout matches OpenCV Mat format for efficient data transfer.
 */
struct InputImageBGR8Type
{
  std::uint8_t b;
  std::uint8_t g;
  std::uint8_t r;
} __attribute__((packed));

/**
 * @brief Result structure for calibration status detection
 *
 * Contains the complete output from calibration validation including
 * classification results, confidence scores, and performance metrics.
 */
struct CalibrationStatusClassifierResult
{
  float calibration_confidence;
  float miscalibration_confidence;
  double preprocessing_time_ms;
  double inference_time_ms;
  uint32_t num_points_projected;
};

/**
 * @brief Configuration structure for camera and LiDAR topic pairing
 *
 * Contains topic names for a camera-LiDAR sensor pair along with configuration
 * parameters for synchronization and preprocessing.
 */
struct CameraLidarTopicsInfo
{
  std::string camera_topic;
  std::string camera_info_topic;
  std::string lidar_topic;
  std::string projected_points_topic;
  double approx_delta;
  bool already_rectified;
};

}  // namespace autoware::calibration_status_classifier

#endif  // AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__DATA_TYPE_HPP_
