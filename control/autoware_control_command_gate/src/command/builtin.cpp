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

#include "builtin.hpp"

#include <memory>
#include <string>

namespace autoware::control_command_gate
{

BuiltinEmergency::BuiltinEmergency(uint16_t id, const std::string & name, rclcpp::Node & node)
: CommandSource(id, name)
{
  acceleration_ = node.declare_parameter<double>("builtin_emergency_acceleration");

  const auto period = rclcpp::Rate(10.0).period();
  clock_ = node.get_clock();
  timer_ = rclcpp::create_timer(&node, clock_, period, [this]() { on_timer(); });
}

void BuiltinEmergency::on_timer()
{
  Control control;
  control.stamp = control.longitudinal.stamp = control.lateral.stamp = clock_->now();
  control.longitudinal.velocity = 0.0;
  control.longitudinal.acceleration = acceleration_;
  control.lateral.steering_tire_angle = prev_control_->lateral.steering_tire_angle;
  control.lateral.steering_tire_rotation_rate = prev_control_->lateral.steering_tire_rotation_rate;
  send_control(control);
}

void BuiltinEmergency::resend_last_command()
{
  const auto stamp = clock_->now();

  GearCommand gear;
  gear.stamp = stamp;
  gear.command = GearCommand::NONE;  // Keep current gear.

  TurnIndicatorsCommand turn_indicators;
  turn_indicators.stamp = stamp;
  turn_indicators.command = TurnIndicatorsCommand::DISABLE;

  HazardLightsCommand hazard_lights;
  hazard_lights.stamp = stamp;
  hazard_lights.command = HazardLightsCommand::ENABLE;

  // NOTE: Control does not need to be resent because it is sent periodically.
  send_gear(gear);
  send_turn_indicators(turn_indicators);
  send_hazard_lights(hazard_lights);
}

}  // namespace autoware::control_command_gate
