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
  adapi_pause_ = std::make_unique<AdapiPauseInterface>(&node_);
}

void Compatibility::publish()
{
  adapi_pause_->publish();
}

void Compatibility::on_control(const Control & msg)
{
  Control out = msg;
  adapi_pause_->update(out);

  if (adapi_pause_->is_paused()) {
    out.longitudinal.velocity = std::min(0.0f, out.longitudinal.velocity);
    out.longitudinal.acceleration =
      std::min(stop_hold_acceleration_, out.longitudinal.acceleration);
  }

  CommandBridge::on_control(out);
}

}  // namespace autoware::control_command_gate
