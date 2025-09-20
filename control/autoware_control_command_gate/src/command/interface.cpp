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

#include "interface.hpp"

#include <utility>

namespace autoware::control_command_gate
{

CommandBridge::CommandBridge(std::unique_ptr<CommandOutput> && output)
{
  output_ = std::move(output);
}

void CommandBridge::on_control(const Control & msg)
{
  output_->on_control(msg);
}

void CommandBridge::on_gear(const GearCommand & msg)
{
  output_->on_gear(msg);
}

void CommandBridge::on_turn_indicators(const TurnIndicatorsCommand & msg)
{
  output_->on_turn_indicators(msg);
}

void CommandBridge::on_hazard_lights(const HazardLightsCommand & msg)
{
  output_->on_hazard_lights(msg);
}

}  // namespace autoware::control_command_gate
