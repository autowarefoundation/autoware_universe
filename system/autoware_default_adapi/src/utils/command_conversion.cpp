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

#include "command_conversion.hpp"

namespace autoware::default_adapi::command_conversion
{

InternalGear convert(const ExternalGear & external)
{
  const auto convert_command = [](const uint8_t status) {
    using ExternalGearType = autoware_adapi_v1_msgs::msg::Gear;
    // clang-format off
    switch (status) {
      case ExternalGearType::NEUTRAL: return InternalGear::NEUTRAL;
      case ExternalGearType::DRIVE:   return InternalGear::DRIVE;
      case ExternalGearType::REVERSE: return InternalGear::REVERSE;
      case ExternalGearType::PARK:    return InternalGear::PARK;
      case ExternalGearType::LOW:     return InternalGear::LOW;
      default:                        return InternalGear::NONE;
    }
    // clang-format on
  };
  InternalGear internal;
  internal.stamp = external.stamp;
  internal.command = convert_command(external.command.status);
  return internal;
}

InternalTurnIndicators convert(const ExternalTurnIndicators & external)
{
  const auto convert_command = [](const uint8_t status) {
    using ExternalTurnIndicatorsType = autoware_adapi_v1_msgs::msg::TurnIndicators;
    // clang-format off
    switch (status) {
      case ExternalTurnIndicatorsType::DISABLE: return InternalTurnIndicators::DISABLE;
      case ExternalTurnIndicatorsType::LEFT:    return InternalTurnIndicators::ENABLE_LEFT;
      case ExternalTurnIndicatorsType::RIGHT:   return InternalTurnIndicators::ENABLE_RIGHT;
      default:                                  return InternalTurnIndicators::NO_COMMAND;
    }
    // clang-format on
  };
  InternalTurnIndicators internal;
  internal.stamp = external.stamp;
  internal.command = convert_command(external.command.status);
  return internal;
}

InternalHazardLights convert(const ExternalHazardLights & external)
{
  const auto convert_command = [](const uint8_t status) {
    using ExternalHazardLightsType = autoware_adapi_v1_msgs::msg::HazardLights;
    // clang-format off
    switch (status) {
      case ExternalHazardLightsType::DISABLE: return InternalHazardLights::DISABLE;
      case ExternalHazardLightsType::ENABLE:  return InternalHazardLights::ENABLE;
      default:                                return InternalHazardLights::NO_COMMAND;
    }
    // clang-format on
  };
  InternalHazardLights internal;
  internal.stamp = external.stamp;
  internal.command = convert_command(external.command.status);
  return internal;
}

}  // namespace autoware::default_adapi::command_conversion
