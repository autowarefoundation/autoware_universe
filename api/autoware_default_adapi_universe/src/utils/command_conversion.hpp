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

#ifndef UTILS__COMMAND_CONVERSION_HPP_
#define UTILS__COMMAND_CONVERSION_HPP_

#include <autoware_adapi_v1_msgs/msg/gear_command.hpp>
#include <autoware_adapi_v1_msgs/msg/hazard_lights_command.hpp>
#include <autoware_adapi_v1_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

namespace autoware::default_adapi::command_conversion
{

using ExternalGear = autoware_adapi_v1_msgs::msg::GearCommand;
using InternalGear = autoware_vehicle_msgs::msg::GearCommand;
InternalGear convert(const ExternalGear & external);

using ExternalTurnIndicators = autoware_adapi_v1_msgs::msg::TurnIndicatorsCommand;
using InternalTurnIndicators = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
InternalTurnIndicators convert(const ExternalTurnIndicators & external);

using ExternalHazardLights = autoware_adapi_v1_msgs::msg::HazardLightsCommand;
using InternalHazardLights = autoware_vehicle_msgs::msg::HazardLightsCommand;
InternalHazardLights convert(const ExternalHazardLights & external);

}  // namespace autoware::default_adapi::command_conversion

#endif  // UTILS__COMMAND_CONVERSION_HPP_
