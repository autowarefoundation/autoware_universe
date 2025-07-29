// Copyright 2025 AutoCore, Inc.
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

#include "autoware/tensorrt_bevformer/bevformer_data_manager.hpp"

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
  // Initialize prev_frame_info
  prev_frame_info_.scene_token = "";
  prev_frame_info_.prev_pos = {0.0f, 0.0f, 0.0f};
  prev_frame_info_.prev_angle = 0.0f;

  RCLCPP_INFO(logger_, "BEVFormerDataManager initialized");
}

void BEVFormerDataManager::initializePrevBev(const std::vector<int64_t> & shape)
{
  // Calculate the total size
  size_t total_size = 1;
  for (const auto & dim : shape) {
    total_size *= dim;
  }

  // Initialize with random values following normal distribution (like np.random.randn())
  prev_bev_.resize(total_size);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> dist(0.0f, 1.0f);

  for (auto & val : prev_bev_) {
    val = dist(gen);
  }

  RCLCPP_INFO(logger_, "Initialized prev_bev with %zu random values", total_size);
}

std::vector<float> BEVFormerDataManager::processCanBus(
  const std::vector<float> & ego2global_translation, const std::vector<float> & ego2global_rotation,
  const std::vector<float> & raw_can_bus)
{
  // Start with the original CAN bus data (this is input_dict["can_bus"] in Python)
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
  const std::vector<float> & can_bus, const std::string & scene_token)
{
  if (can_bus.size() < 18) {
    RCLCPP_ERROR(logger_, "Invalid CAN bus size: %zu, expected at least 18", can_bus.size());
    return can_bus;
  }

  // Make a copy of the CAN bus data
  std::vector<float> processed_can_bus = can_bus;

  // Store current position and angle BEFORE modifications (Python: tmp_pos, tmp_angle)
  current_tmp_pos_ = {processed_can_bus[0], processed_can_bus[1], processed_can_bus[2]};
  current_tmp_angle_ = processed_can_bus[17];

  // Apply temporal adjustments based on whether we use prev_bev
  float use_prev_bev = getUsePrevBev(scene_token);

  if (use_prev_bev == 1.0f) {
    processed_can_bus[0] -= prev_frame_info_.prev_pos[0];
    processed_can_bus[1] -= prev_frame_info_.prev_pos[1];
    processed_can_bus[2] -= prev_frame_info_.prev_pos[2];
    processed_can_bus[17] -= prev_frame_info_.prev_angle;
  } else {
    processed_can_bus[0] = 0.0f;
    processed_can_bus[1] = 0.0f;
    processed_can_bus[2] = 0.0f;
    processed_can_bus[17] = 0.0f;

    RCLCPP_INFO(logger_, "Reset CAN bus to zeros for new scene");
  }

  // Update scene token
  prev_frame_info_.scene_token = scene_token;

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

float BEVFormerDataManager::getUsePrevBev(const std::string & scene_token)
{
  // If the scene token changes, don't use previous BEV
  if (prev_frame_info_.scene_token != scene_token) {
    RCLCPP_INFO(
      logger_, "Scene changed: '%s' -> '%s'. NOT using previous BEV.",
      prev_frame_info_.scene_token.c_str(), scene_token.c_str());
    return 0.0f;
  }
  return 1.0f;
}

void BEVFormerDataManager::updatePrevBev(const std::vector<float> & bev_embed)
{
  prev_bev_ = bev_embed;
}

}  // namespace tensorrt_bevformer
}  // namespace autoware
