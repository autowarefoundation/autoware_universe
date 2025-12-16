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

#include "../src/input_converter/bev_shift_converter.hpp"

namespace autoware::tensorrt_vad::vad_interface
{

InputBEVShiftConverter::InputBEVShiftConverter(
  const CoordinateTransformer & coordinate_transformer, const VadInterfaceConfig & config)
: Converter(coordinate_transformer, config),
  real_w_(config.detection_range[3] - config.detection_range[0]),
  real_h_(config.detection_range[4] - config.detection_range[1]),
  default_delta_x_(0.0f),
  default_delta_y_(0.0f)
{
}

ShiftData InputBEVShiftConverter::process_shift(
  const CanBusData & can_bus, const CanBusData & prev_can_bus) const
{
  float delta_x = default_delta_x_;
  float delta_y = default_delta_y_;

  if (!prev_can_bus.empty()) {
    delta_x = can_bus[0] - prev_can_bus[0];  // translation difference
    delta_y = can_bus[1] - prev_can_bus[1];  // translation difference
  }

  float patch_angle_rad = can_bus[16];  // current patch_angle[rad]

  float ego_angle = patch_angle_rad / M_PI * 180.0;

  float translation_length = std::sqrt(delta_x * delta_x + delta_y * delta_y);
  float translation_angle = std::atan2(delta_y, delta_x) / M_PI * 180.0;
  float bev_angle = ego_angle - translation_angle;

  float shift_y = translation_length * std::cos(bev_angle / 180.0 * M_PI) / real_h_;
  float shift_x = translation_length * std::sin(bev_angle / 180.0 * M_PI) / real_w_;

  return {shift_x, shift_y};
}

}  // namespace autoware::tensorrt_vad::vad_interface
