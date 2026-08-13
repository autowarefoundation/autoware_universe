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

// cspell:ignore BEVFORMER, Canbus

#ifndef BEVFORMER_DATA_MANAGER_HPP_
#define BEVFORMER_DATA_MANAGER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/logger.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware
{
namespace tensorrt_bevformer
{

/**
 * @brief Manages BEVFormer temporal data and state across frames
 */
class BEVFormerDataManager
{
public:
  /**
   * @brief Constructor
   *
   * @param logger ROS logger for messages
   */
  explicit BEVFormerDataManager(const rclcpp::Logger & logger);

  /**
   * @brief Initialize previous BEV with random values
   *
   * @param shape Shape of the previous BEV tensor
   */
  void initializePrevBev(const std::vector<int64_t> & shape);

  /**
   * @brief Process CAN bus data with temporal adjustment
   *
   * @param can_bus Raw CAN bus data
   * @param use_prev_bev Flag indicating whether to use previous BEV (0.0f or 1.0f)
   * @return std::vector<float> Processed CAN bus data
   */
  std::vector<float> processCanbusWithTemporal(
    const std::vector<float> & can_bus, float use_prev_bev);

  /**
   * @brief Update previous BEV with new data
   *
   * @param bev_embed New BEV embedding
   */
  void updatePrevBev(const std::vector<float> & bev_embed);

  void updatePrevFrameInfo();

  /**
   * @brief Get the previous BEV embedding
   *
   * @return const std::vector<float>& Previous BEV embedding
   */
  const std::vector<float> & getPrevBev() const { return prev_bev_; }

  /**
   * @brief Check if we should use previous BEV (for real-world continuous driving)
   * @return float 1.0 if we should use previous BEV, 0.0 for first frame
   */
  float getUsePrevBev();

  /**
   * @brief Process quaternion to extract yaw angle
   *
   * @param w W component
   * @param x X component
   * @param y Y component
   * @param z Z component
   * @return float Yaw angle in radians
   */
  static float quaternionToYaw(float w, float x, float y, float z);

  /**
   * @brief Process raw CAN bus data with ego2global transformations
   *
   * @param ego2global_translation Translation vector [x, y, z]
   * @param ego2global_rotation Quaternion [w, x, y, z]
   * @param raw_can_bus Raw CAN bus data
   * @return std::vector<float> Processed CAN bus data
   */
  std::vector<float> processCanBus(
    const std::vector<float> & ego2global_translation,
    const std::vector<float> & ego2global_rotation, const std::vector<float> & raw_can_bus);

private:
  rclcpp::Logger logger_;

  bool is_first_frame_ = true;  // Track first frame for real-world usage

  std::vector<float> current_tmp_pos_;
  float current_tmp_angle_;

  std::vector<float> prev_bev_;

  struct PrevFrameInfo
  {
    std::vector<float> prev_pos{0.0f, 0.0f, 0.0f};
    float prev_angle{0.0f};
  };
  PrevFrameInfo prev_frame_info_;

  /**
   * @brief Generate random BEV values using normal distribution
   */
  void generateRandomBev();
};

}  // namespace tensorrt_bevformer
}  // namespace autoware

#endif  // BEVFORMER_DATA_MANAGER_HPP_
