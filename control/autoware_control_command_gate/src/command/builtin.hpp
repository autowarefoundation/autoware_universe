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

#ifndef COMMAND__BUILTIN_HPP_
#define COMMAND__BUILTIN_HPP_

#include "source.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autoware::control_command_gate
{

class BuiltinEmergency : public CommandSource
{
public:
  explicit BuiltinEmergency(uint16_t id, const std::string & name, rclcpp::Node & node);
  void resend_last_command() override;
  void set_prev_control(std::shared_ptr<Control> control) { prev_control_ = control; }

private:
  void on_timer();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr clock_;

  double acceleration_;
  std::shared_ptr<Control> prev_control_;
};

}  // namespace autoware::control_command_gate

#endif  // COMMAND__BUILTIN_HPP_
