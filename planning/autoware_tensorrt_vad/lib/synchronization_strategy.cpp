// Copyright 2025 TIER IV.
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

#include "../src/synchronization_strategy.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace autoware::tensorrt_vad
{

FrontCriticalSynchronizationStrategy::FrontCriticalSynchronizationStrategy(
  int32_t front_camera_id, double sync_tolerance_ms)
: front_camera_id_(front_camera_id), sync_tolerance_ms_(sync_tolerance_ms)
{
}

bool FrontCriticalSynchronizationStrategy::is_ready(
  const VadInputTopicData & vad_input_topic_data) const
{
  // Check if front camera data exists
  if (
    front_camera_id_ >= static_cast<int32_t>(vad_input_topic_data.images.size()) ||
    !vad_input_topic_data.images[front_camera_id_]) {
    return false;
  }

  // Check if synchronized
  return is_synchronized(vad_input_topic_data);
}

bool FrontCriticalSynchronizationStrategy::is_dropped(
  const VadInputTopicData & vad_input_topic_data) const
{
  const bool missing_image = std::any_of(
    vad_input_topic_data.images.begin(), vad_input_topic_data.images.end(),
    [](const auto & image) { return !image; });

  const bool missing_camera_info = std::any_of(
    vad_input_topic_data.camera_infos.begin(), vad_input_topic_data.camera_infos.end(),
    [](const auto & camera_info) { return !camera_info; });

  return missing_image || missing_camera_info;
}

std::optional<VadInputTopicData> FrontCriticalSynchronizationStrategy::fill_dropped_data(
  const VadInputTopicData & current_data)
{
  // Check if front camera image exists
  if (
    front_camera_id_ >= static_cast<int32_t>(current_data.images.size()) ||
    !current_data.images[front_camera_id_]) {
    return std::nullopt;  // Cannot fill dropped data without front camera reference
  }

  VadInputTopicData filled_data = current_data;

  // Get reference front camera image for dimensions and timestamp
  const auto & front_image = filled_data.images[front_camera_id_];

  // Fill dropped images with black images
  for (std::size_t i = 0; i < filled_data.images.size(); ++i) {
    if (!filled_data.images[i]) {
      // Copy front camera image and overwrite with black data
      auto black_image = std::make_shared<sensor_msgs::msg::Image>(*front_image);
      black_image->header.frame_id = "camera_" + std::to_string(i) + "/camera_optical_link";

      // Fill with black pixels (overwrite data)
      std::fill(black_image->data.begin(), black_image->data.end(), 0);

      filled_data.images[i] = black_image;
    }
  }

  return filled_data;
}

bool FrontCriticalSynchronizationStrategy::is_synchronized(
  const VadInputTopicData & vad_input_topic_data) const
{
  // Check if essential data exists
  if (!vad_input_topic_data.kinematic_state || !vad_input_topic_data.acceleration) {
    return false;
  }

  // Check if front camera exists
  if (
    front_camera_id_ >= static_cast<int32_t>(vad_input_topic_data.images.size()) ||
    !vad_input_topic_data.images[front_camera_id_]) {
    return false;
  }

  // Get timestamps
  const auto front_camera_stamp = vad_input_topic_data.images[front_camera_id_]->header.stamp;
  const auto kinematic_state_stamp = vad_input_topic_data.kinematic_state->header.stamp;
  const auto acceleration_stamp = vad_input_topic_data.acceleration->header.stamp;

  // Calculate time differences in milliseconds
  const double front_to_kinematic_diff_ms = std::abs(
    (rclcpp::Time(front_camera_stamp) - rclcpp::Time(kinematic_state_stamp)).seconds() * 1000.0);

  const double front_to_acceleration_diff_ms = std::abs(
    (rclcpp::Time(front_camera_stamp) - rclcpp::Time(acceleration_stamp)).seconds() * 1000.0);

  // Check if all are within tolerance
  return (front_to_kinematic_diff_ms <= sync_tolerance_ms_) &&
         (front_to_acceleration_diff_ms <= sync_tolerance_ms_);
}

}  // namespace autoware::tensorrt_vad
