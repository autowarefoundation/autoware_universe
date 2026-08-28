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

#include "subscription.hpp"

#include <string>

namespace autoware::control_command_gate
{

bool validate_command(const Control & msg)
{
  if (!std::isfinite(msg.longitudinal.velocity)) return false;
  if (!std::isfinite(msg.longitudinal.acceleration)) return false;
  if (!std::isfinite(msg.longitudinal.jerk)) return false;
  if (!std::isfinite(msg.lateral.steering_tire_angle)) return false;
  if (!std::isfinite(msg.lateral.steering_tire_rotation_rate)) return false;
  return true;
}

CommandSubscription::CommandSubscription(uint16_t id, const std::string & name, rclcpp::Node & node)
: CommandSource(id, name), clock_(node.get_clock()), logger_(node.get_logger())
{
  using std::placeholders::_1;
  const auto control_qos = rclcpp::QoS(5);
  const auto volatile_qos = rclcpp::QoS(1);
  const auto prefix = "~/inputs/" + name + "/";

  sub_control_ = node.create_subscription<Control>(
    prefix + "control", control_qos, std::bind(&CommandSubscription::on_control, this, _1));
  sub_gear_ = node.create_subscription<GearCommand>(
    prefix + "gear", volatile_qos, std::bind(&CommandSubscription::on_gear, this, _1));
  sub_turn_indicators_ = node.create_subscription<TurnIndicatorsCommand>(
    prefix + "turn_indicators", volatile_qos,
    std::bind(&CommandSubscription::on_turn_indicators, this, _1));
  sub_hazard_lights_ = node.create_subscription<HazardLightsCommand>(
    prefix + "hazard_lights", volatile_qos,
    std::bind(&CommandSubscription::on_hazard_lights, this, _1));
}

void CommandSubscription::on_control(const Control & msg)
{
  // NOTE: The control command filter does not handle NaN.
  if (!validate_command(msg)) {
    RCLCPP_ERROR_STREAM_THROTTLE(logger_, *clock_, 1000, "invalid control from " + source_name_);
    return;
  }

  // NOTE: Control does not need to be saved because it is sent periodically.
  send_control(msg);
}

void CommandSubscription::on_gear(const GearCommand & msg)
{
  last_gear_ = msg;
  send_gear(msg);
}

void CommandSubscription::on_turn_indicators(const TurnIndicatorsCommand & msg)
{
  last_turn_indicators_ = msg;
  send_turn_indicators(msg);
}

void CommandSubscription::on_hazard_lights(const HazardLightsCommand & msg)
{
  last_hazard_lights_ = msg;
  send_hazard_lights(msg);
}

void CommandSubscription::resend_last_command()
{
  // NOTE: Control does not need to be resent because it is sent periodically.
  if (last_gear_) send_gear(*last_gear_);
  if (last_turn_indicators_) send_turn_indicators(*last_turn_indicators_);
  if (last_hazard_lights_) send_hazard_lights(*last_hazard_lights_);
}

}  // namespace autoware::control_command_gate
