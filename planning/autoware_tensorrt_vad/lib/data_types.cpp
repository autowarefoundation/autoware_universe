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

#include "../src/data_types.hpp"

namespace autoware::tensorrt_vad
{

VadInputTopicData::VadInputTopicData(const int32_t num_cameras)
: images(num_cameras), camera_infos(num_cameras), num_cameras_(num_cameras)
{
}

bool VadInputTopicData::is_complete() const
{
  // Check if vector sizes match expected camera count
  if (
    static_cast<int32_t>(images.size()) != num_cameras_ ||
    static_cast<int32_t>(camera_infos.size()) != num_cameras_) {
    return false;
  }

  // Check if all required data is available
  for (int32_t i = 0; i < num_cameras_; ++i) {
    if (!images[i] || !camera_infos[i]) {
      return false;
    }
  }

  return kinematic_state != nullptr && acceleration != nullptr;
}

void VadInputTopicData::reset()
{
  frame_started_ = false;

  // Reset all images and camera_infos using clear + resize
  images.clear();
  images.resize(num_cameras_);
  camera_infos.clear();
  camera_infos.resize(num_cameras_);

  // Reset other data
  kinematic_state = nullptr;
  acceleration = nullptr;
}

void VadInputTopicData::ensure_frame_started(const rclcpp::Time & msg_stamp)
{
  if (!frame_started_) {
    stamp = msg_stamp;
    frame_started_ = true;
  }
}

void VadInputTopicData::set_image(
  const std::size_t camera_id, const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (camera_id >= images.size()) {
    return;  // Invalid camera ID
  }
  ensure_frame_started(msg->header.stamp);
  images[camera_id] = msg;
}

void VadInputTopicData::set_camera_info(
  const std::size_t camera_id, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  if (camera_id >= camera_infos.size()) {
    return;  // Invalid camera ID
  }
  ensure_frame_started(msg->header.stamp);
  camera_infos[camera_id] = msg;
}

void VadInputTopicData::set_kinematic_state(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  ensure_frame_started(msg->header.stamp);
  kinematic_state = msg;
}

void VadInputTopicData::set_acceleration(
  const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr & msg)
{
  ensure_frame_started(msg->header.stamp);
  acceleration = msg;
}

}  // namespace autoware::tensorrt_vad
