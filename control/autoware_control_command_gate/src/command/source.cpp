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

#include "source.hpp"

#include <memory>
#include <string>

namespace autoware::control_command_gate
{

CommandSource::CommandSource(const std::string & name) : name_(name)
{
  output_ = nullptr;
}

TimeoutDiag * CommandSource::create_diag_task(
  const TimeoutDiag::Params & params, const rclcpp::Clock & clock)
{
  if (timeout_) {
    throw std::logic_error("timeout diag has already been created");
  }
  timeout_ = std::make_unique<TimeoutDiag>(params, clock, "timeout_" + name());
  return timeout_.get();
}

void CommandSource::set_output(CommandOutput * output)
{
  output_ = output;
  if (output_) resend_last_command();
}

void CommandSource::send_control(const Control & msg)
{
  timeout_->update();
  if (output_) output_->on_control(msg);
}

void CommandSource::send_gear(const GearCommand & msg)
{
  if (output_) output_->on_gear(msg);
}

void CommandSource::send_turn_indicators(const TurnIndicatorsCommand & msg)
{
  if (output_) output_->on_turn_indicators(msg);
}

void CommandSource::send_hazard_lights(const HazardLightsCommand & msg)
{
  if (output_) output_->on_hazard_lights(msg);
}

}  // namespace autoware::control_command_gate
