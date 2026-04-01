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

#include "../src/core/vad_input_converter.hpp"

#include <cmath>

namespace autoware::tensorrt_vad
{

VadInputConverter::VadInputConverter(
  const TransformProvider & transform_provider, const VadInterfaceConfig & config)
: transform_provider_(transform_provider), config_(config)
{
}

VadInputData VadInputConverter::convert(const CoreInputFrame & frame)
{
  VadInputData input;

  if (!cached_vad_base2img_.has_value()) {
    auto computed = compute_vad_base2img(frame.camera_infos);
    bool has_non_zero = false;
    for (const auto & value : computed) {
      if (std::abs(value) > 1e-6f) {
        has_non_zero = true;
        break;
      }
    }
    if (has_non_zero) {
      cached_vad_base2img_ = computed;
    }
  }

  input.vad_base2img = cached_vad_base2img_.has_value() ? cached_vad_base2img_.value()
                                                        : compute_vad_base2img(frame.camera_infos);

  if (frame.kinematic_state.has_value() && frame.acceleration.has_value()) {
    input.can_bus = compute_can_bus(frame.kinematic_state.value(), frame.acceleration.value());
  }

  if (!input.can_bus.empty()) {
    input.shift = compute_shift(input.can_bus, prev_can_bus_);
  }

  input.camera_images = frame.camera_images;
  input.command = config_.default_command;

  prev_can_bus_ = input.can_bus;

  return input;
}

std::vector<float> VadInputConverter::compute_vad_base2img(
  const std::vector<CoreCameraInfo> & camera_infos) const
{
  const int32_t num_cameras = static_cast<int32_t>(camera_infos.size());
  std::vector<float> frame_vad_base2img(16 * num_cameras, 0.0f);

  for (int32_t camera_id = 0; camera_id < num_cameras; ++camera_id) {
    const auto & camera_info = camera_infos[camera_id];
    if (camera_info.width <= 0 || camera_info.height <= 0 || camera_info.frame_id.empty()) {
      continue;
    }

    auto base2cam_opt = transform_provider_.lookup_base2cam(camera_info.frame_id);
    if (!base2cam_opt.has_value()) {
      continue;
    }

    Eigen::Matrix4f base2cam = base2cam_opt.value();
    Eigen::Matrix4f cam2img = create_cam2img(camera_info);
    Eigen::Matrix4f vad_base2img = cam2img * base2cam;

    auto [scale_width, scale_height] = calculate_scale(camera_info);
    Eigen::Matrix4f vad_base2img_scaled = apply_scaling(vad_base2img, scale_width, scale_height);

    const auto vad_base2img_flat = matrix_to_flat(vad_base2img_scaled);
    std::copy(
      vad_base2img_flat.begin(), vad_base2img_flat.end(),
      frame_vad_base2img.begin() + camera_id * 16);
  }

  return frame_vad_base2img;
}

std::vector<float> VadInputConverter::compute_can_bus(
  const CoreKinematicState & kinematic_state, const CoreAcceleration & acceleration) const
{
  std::vector<float> can_bus(18, 0.0f);

  can_bus[0] = static_cast<float>(kinematic_state.position.x());
  can_bus[1] = static_cast<float>(kinematic_state.position.y());
  can_bus[2] = static_cast<float>(kinematic_state.position.z());

  can_bus[3] = static_cast<float>(kinematic_state.orientation.x());
  can_bus[4] = static_cast<float>(kinematic_state.orientation.y());
  can_bus[5] = static_cast<float>(kinematic_state.orientation.z());
  can_bus[6] = static_cast<float>(kinematic_state.orientation.w());

  can_bus[7] = static_cast<float>(acceleration.linear.x());
  can_bus[8] = static_cast<float>(acceleration.linear.y());
  can_bus[9] = static_cast<float>(acceleration.linear.z());

  can_bus[10] = static_cast<float>(kinematic_state.angular_velocity.x());
  can_bus[11] = static_cast<float>(kinematic_state.angular_velocity.y());
  can_bus[12] = static_cast<float>(kinematic_state.angular_velocity.z());

  can_bus[13] = static_cast<float>(kinematic_state.linear_velocity.x());
  can_bus[14] = static_cast<float>(kinematic_state.linear_velocity.y());
  can_bus[15] = 0.0f;

  const double yaw = std::atan2(
    2.0 * (can_bus[6] * can_bus[5] + can_bus[3] * can_bus[4]),
    1.0 - 2.0 * (can_bus[4] * can_bus[4] + can_bus[5] * can_bus[5]));
  const double wrapped_yaw = yaw < 0.0 ? yaw + 2.0 * M_PI : yaw;
  can_bus[16] = static_cast<float>(wrapped_yaw);

  float delta_yaw = 0.0f;
  if (!prev_can_bus_.empty()) {
    float prev_angle = prev_can_bus_[16];
    delta_yaw = static_cast<float>(wrapped_yaw) - prev_angle;
    while (delta_yaw > M_PI) {
      delta_yaw -= 2.0f * static_cast<float>(M_PI);
    }
    while (delta_yaw < -M_PI) {
      delta_yaw += 2.0f * static_cast<float>(M_PI);
    }
  }
  can_bus[17] = delta_yaw * 180.0f / static_cast<float>(M_PI);

  return can_bus;
}

std::vector<float> VadInputConverter::compute_shift(
  const std::vector<float> & can_bus, const std::vector<float> & prev_can_bus) const
{
  const float real_w = config_.detection_range[3] - config_.detection_range[0];
  const float real_h = config_.detection_range[4] - config_.detection_range[1];

  float delta_x = 0.0f;
  float delta_y = 0.0f;

  if (!prev_can_bus.empty()) {
    delta_x = can_bus[0] - prev_can_bus[0];
    delta_y = can_bus[1] - prev_can_bus[1];
  }

  float patch_angle_rad = can_bus[16];
  float ego_angle = patch_angle_rad / static_cast<float>(M_PI) * 180.0f;

  float translation_length = std::sqrt(delta_x * delta_x + delta_y * delta_y);
  float translation_angle = std::atan2(delta_y, delta_x) / static_cast<float>(M_PI) * 180.0f;
  float bev_angle = ego_angle - translation_angle;

  float shift_y =
    translation_length * std::cos(bev_angle / 180.0f * static_cast<float>(M_PI)) / real_h;
  float shift_x =
    translation_length * std::sin(bev_angle / 180.0f * static_cast<float>(M_PI)) / real_w;

  return {shift_x, shift_y};
}

Eigen::Matrix4f VadInputConverter::create_cam2img(const CoreCameraInfo & camera_info) const
{
  Eigen::Matrix4f cam2img;
  cam2img << static_cast<float>(camera_info.k[0]), static_cast<float>(camera_info.k[1]),
    static_cast<float>(camera_info.k[2]), 0.0f, static_cast<float>(camera_info.k[3]),
    static_cast<float>(camera_info.k[4]), static_cast<float>(camera_info.k[5]), 0.0f,
    static_cast<float>(camera_info.k[6]), static_cast<float>(camera_info.k[7]),
    static_cast<float>(camera_info.k[8]), 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;
  return cam2img;
}

std::pair<float, float> VadInputConverter::calculate_scale(const CoreCameraInfo & camera_info) const
{
  const float scale_width = config_.target_image_width / static_cast<float>(camera_info.width);
  const float scale_height = config_.target_image_height / static_cast<float>(camera_info.height);
  return std::make_pair(scale_width, scale_height);
}

Eigen::Matrix4f VadInputConverter::apply_scaling(
  const Eigen::Matrix4f & vad_base2img, float scale_width, float scale_height) const
{
  Eigen::Matrix4f scale_matrix = Eigen::Matrix4f::Identity();
  scale_matrix(0, 0) = scale_width;
  scale_matrix(1, 1) = scale_height;
  return scale_matrix * vad_base2img;
}

std::vector<float> VadInputConverter::matrix_to_flat(const Eigen::Matrix4f & matrix) const
{
  std::vector<float> flat(16);
  int32_t k = 0;
  for (int32_t i = 0; i < 4; ++i) {
    for (int32_t j = 0; j < 4; ++j) {
      flat[k++] = matrix(i, j);
    }
  }
  return flat;
}

}  // namespace autoware::tensorrt_vad
