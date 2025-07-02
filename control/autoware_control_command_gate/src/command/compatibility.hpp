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

#ifndef COMMAND__COMPATIBILITY_HPP_
#define COMMAND__COMPATIBILITY_HPP_

#include "compatibility/adapi_pause_interface.hpp"
#include "interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::control_command_gate
{

// Provide vehicle_cmd_gate compatible interface.
class Compatibility : public CommandBridge
{
public:
  Compatibility(std::unique_ptr<CommandOutput> && output, rclcpp::Node & node);
  void publish();
  void set_prev_control(std::shared_ptr<Control> control) { prev_control_ = control; }
  void on_control(const Control & msg) override;

private:
  rclcpp::Node & node_;
  std::shared_ptr<Control> prev_control_;
  std::unique_ptr<AdapiPauseInterface> adapi_pause_;
  float stop_hold_acceleration_;
};

}  // namespace autoware::control_command_gate

#endif  // COMMAND__COMPATIBILITY_HPP_
