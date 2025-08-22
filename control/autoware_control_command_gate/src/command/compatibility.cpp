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

#include "compatibility.hpp"

#include <algorithm>
#include <memory>
#include <utility>

namespace autoware::control_command_gate
{

Compatibility::Compatibility(std::unique_ptr<CommandOutput> && output, rclcpp::Node & node)
: CommandBridge(std::move(output)), node_(node)
{
  stop_hold_acceleration_ = node.declare_parameter<float>("stop_hold_acceleration");
  emergency_acceleration_ = node.declare_parameter<float>("emergency_acceleration");
  moderate_stop_acceleration_ = node.declare_parameter<float>("moderate_stop_acceleration");

  adapi_pause_ = std::make_unique<AdapiPauseInterface>(&node_);
  emergency_ = std::make_unique<EmergencyInterface>(&node_);
  moderate_stop_ = std::make_unique<ModerateStopInterface>(&node_);
}

void Compatibility::publish()
{
  adapi_pause_->publish();
  emergency_->publish();
  moderate_stop_->publish();
}

void Compatibility::on_control(const Control & msg)
{
  const auto set_stop_command = [](auto & longitudinal, const auto & acceleration) {
    longitudinal.velocity = std::min(0.0f, longitudinal.velocity);
    longitudinal.acceleration = std::min(acceleration, longitudinal.acceleration);
  };

  Control out = msg;

  if (moderate_stop_->is_stop_requested()) {  // if stop requested, stop the vehicle
    set_stop_command(out.longitudinal, moderate_stop_acceleration_);
  }

  if (emergency_->is_emergency()) {
    set_stop_command(out.longitudinal, emergency_acceleration_);
  }

  adapi_pause_->update(out);

  if (adapi_pause_->is_paused()) {
    set_stop_command(out.longitudinal, stop_hold_acceleration_);
  }

  CommandBridge::on_control(out);
}

}  // namespace autoware::control_command_gate
