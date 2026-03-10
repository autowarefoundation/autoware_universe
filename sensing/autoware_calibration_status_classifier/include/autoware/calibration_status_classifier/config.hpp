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

#ifndef AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CONFIG_HPP_
#define AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CONFIG_HPP_

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace autoware::calibration_status_classifier
{

/**
 * @brief Configuration parameters for calibration status processing
 */
struct CalibrationStatusClassifierConfig
{
  /**
   * @brief Constructor for CalibrationStatusClassifierConfig
   * @param max_depth Maximum depth for projected LiDAR points in the camera frame
   * @param dilation_size Size of morphological dilation kernel for point projection
   * @param height Height of the input image for optimization profile of the ML model, specified as
   * [min, opt, max].
   * @param width Width of the input image for optimization profile of the ML model, specified as
   * [min, opt, max].
   * @throws std::invalid_argument for invalid parameter values
   */
  CalibrationStatusClassifierConfig(
    const double max_depth, const int64_t dilation_size, const std::vector<int64_t> & height,
    const std::vector<int64_t> & width, const std::vector<double> & ego_box = {})
  {
    if (max_depth <= 0.0) {
      throw std::invalid_argument("Lidar range must be positive");
    }
    if (dilation_size < 0) {
      throw std::invalid_argument("Dilation size must be non-negative");
    }
    if (height.size() != 3) {
      throw std::invalid_argument("Height parameter must have exactly 3 elements [min, opt, max]");
    }
    if (width.size() != 3) {
      throw std::invalid_argument("Width parameter must have exactly 3 elements [min, opt, max]");
    }
    for (const auto h : height) {
      if (h <= 0) {
        throw std::invalid_argument("Height values must be positive");
      }
    }
    for (const auto w : width) {
      if (w <= 0) {
        throw std::invalid_argument("Width values must be positive");
      }
    }
    if (height[0] > height[1] || height[1] > height[2]) {
      throw std::invalid_argument("Height values must be in ascending order: min <= opt <= max");
    }
    if (width[0] > width[1] || width[1] > width[2]) {
      throw std::invalid_argument("Width values must be in ascending order: min <= opt <= max");
    }
    if (!ego_box.empty() && ego_box.size() != 6) {
      throw std::invalid_argument(
        "ego_box must have exactly 6 elements [x_min, y_min, z_min, x_max, y_max, z_max]");
    }
    if (ego_box.size() == 6) {
      // All-zeros means disabled
      const bool all_zeros =
        std::all_of(ego_box.begin(), ego_box.end(), [](double v) { return v == 0.0; });
      if (!all_zeros) {
        if (ego_box[0] >= ego_box[3] || ego_box[1] >= ego_box[4] || ego_box[2] >= ego_box[5]) {
          throw std::invalid_argument("ego_box min values must be less than max values");
        }
        this->ego_box = ego_box;
      }
    }

    this->max_depth = max_depth;
    this->dilation_size = static_cast<uint32_t>(dilation_size);
    this->height = {
      static_cast<int32_t>(height[0]), static_cast<int32_t>(height[1]),
      static_cast<int32_t>(height[2])};
    this->width = {
      static_cast<int32_t>(width[0]), static_cast<int32_t>(width[1]),
      static_cast<int32_t>(width[2])};
  }
  double max_depth;
  uint32_t dilation_size;
  std::vector<int32_t> height;
  std::vector<int32_t> width;
  std::vector<double> ego_box;
  int32_t channels{5};
};

}  // namespace autoware::calibration_status_classifier
#endif  // AUTOWARE__CALIBRATION_STATUS_CLASSIFIER__CONFIG_HPP_
