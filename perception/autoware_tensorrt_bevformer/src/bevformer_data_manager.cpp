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

// cspell:ignore randn, Canbus, siny, BEVFormer

#include "bevformer_data_manager.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

namespace autoware
{
namespace tensorrt_bevformer
{

BEVFormerDataManager::BEVFormerDataManager(const rclcpp::Logger & logger) : logger_(logger)
{
  prev_frame_info_.prev_pos = {0.0f, 0.0f, 0.0f};
  prev_frame_info_.prev_angle = 0.0f;

  current_tmp_angle_ = 0.0f;

  RCLCPP_INFO(logger_, "BEVFormerDataManager initialized");
}

void BEVFormerDataManager::initializePrevBev(const std::vector<int64_t> & shape)
{
  size_t total_size = 1;
  for (const auto & dim : shape) {
    total_size *= dim;
  }

  prev_bev_.resize(total_size);

  generateRandomBev();

  RCLCPP_INFO(logger_, "Initialized prev_bev with %zu random values", total_size);
}

void BEVFormerDataManager::generateRandomBev()
{
  if (prev_bev_.empty()) {
    RCLCPP_WARN(logger_, "Cannot generate random BEV - vector not allocated");
    return;
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> dist(0.0f, 1.0f);

  for (auto & val : prev_bev_) {
    val = dist(gen);
  }

  RCLCPP_DEBUG(logger_, "Generated random BEV with %zu values", prev_bev_.size());
}

float BEVFormerDataManager::getUsePrevBev()
{
  if (is_first_frame_) {
    RCLCPP_INFO(logger_, "First frame detected, using random BEV");
    is_first_frame_ = false;  // Set to false after first use
    return 0.0f;              // Don't use previous BEV for first frame
  }
  return 1.0f;  // To use previous BEV after first frame
}

std::vector<float> BEVFormerDataManager::processCanBus(
  const std::vector<float> & ego2global_translation, const std::vector<float> & ego2global_rotation,
  const std::vector<float> & raw_can_bus)
{
  // Start with the original CAN bus data
  std::vector<float> can_bus(18, 0.0f);

  // Copy existing CAN bus data if available
  if (!raw_can_bus.empty()) {
    for (size_t i = 0; i < std::min(raw_can_bus.size(), can_bus.size()); ++i) {
      can_bus[i] = raw_can_bus[i];
    }
  }

  // Validate input data
  if (ego2global_translation.size() < 3 || ego2global_rotation.size() < 4) {
    RCLCPP_WARN(
      logger_,
      "Insufficient ego2global data for CAN bus processing. "
      "Translation size: %zu (need 3), Rotation size: %zu (need 4)",
      ego2global_translation.size(), ego2global_rotation.size());
    return can_bus;
  }

  can_bus[0] = ego2global_translation[0];  // x
  can_bus[1] = ego2global_translation[1];  // y
  can_bus[2] = ego2global_translation[2];  // z

  can_bus[3] = ego2global_rotation[0];  // w
  can_bus[4] = ego2global_rotation[1];  // x
  can_bus[5] = ego2global_rotation[2];  // y
  can_bus[6] = ego2global_rotation[3];  // z

  float yaw_radians = quaternionToYaw(
    ego2global_rotation[0],  // w
    ego2global_rotation[1],  // x
    ego2global_rotation[2],  // y
    ego2global_rotation[3]   // z
  );
  float patch_angle = yaw_radians / M_PI * 180.0f;

  if (patch_angle < 0) {
    patch_angle += 360.0f;
  }

  can_bus[16] = patch_angle / 180.0f * M_PI;  // Convert back to radians
  can_bus[17] = patch_angle;                  // Keep in degrees

  return can_bus;
}

std::vector<float> BEVFormerDataManager::processCanbusWithTemporal(
  const std::vector<float> & can_bus, float use_prev_bev)
{
  if (can_bus.size() < 18) {
    RCLCPP_ERROR(logger_, "Invalid CAN bus size: %zu, expected at least 18", can_bus.size());
    return can_bus;
  }

  std::vector<float> processed_can_bus = can_bus;

  // Store current position and angle BEFORE modifications
  current_tmp_pos_ = {processed_can_bus[0], processed_can_bus[1], processed_can_bus[2]};
  current_tmp_angle_ = processed_can_bus[17];

  if (use_prev_bev == 1.0f) {
    // Use temporal data (subtract previous position/angle)
    processed_can_bus[0] -= prev_frame_info_.prev_pos[0];
    processed_can_bus[1] -= prev_frame_info_.prev_pos[1];
    processed_can_bus[2] -= prev_frame_info_.prev_pos[2];
    processed_can_bus[17] -= prev_frame_info_.prev_angle;
    RCLCPP_DEBUG(logger_, "Using temporal BEV data (subsequent frame)");
  } else {
    // First frame - zero out relative positions
    processed_can_bus[0] = 0.0f;
    processed_can_bus[1] = 0.0f;
    processed_can_bus[2] = 0.0f;
    processed_can_bus[17] = 0.0f;
    RCLCPP_INFO(logger_, "Using random BEV (first frame)");
  }

  return processed_can_bus;
}

void BEVFormerDataManager::updatePrevFrameInfo()
{
  prev_frame_info_.prev_pos = current_tmp_pos_;
  prev_frame_info_.prev_angle = current_tmp_angle_;
}

float BEVFormerDataManager::quaternionToYaw(float w, float x, float y, float z)
{
  float siny_cosp = 2.0f * (w * z + x * y);
  float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void BEVFormerDataManager::updatePrevBev(const std::vector<float> & bev_embed)
{
  prev_bev_ = bev_embed;
}

}  // namespace tensorrt_bevformer
}  // namespace autoware
