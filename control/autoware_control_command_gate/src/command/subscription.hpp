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

#ifndef COMMAND__SUBSCRIPTION_HPP_
#define COMMAND__SUBSCRIPTION_HPP_

#include "source.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::control_command_gate
{

class CommandSubscription : public CommandSource
{
public:
  CommandSubscription(const std::string & name, rclcpp::Node & node);
  void resend_last_command() override;

private:
  void on_control(const Control & msg);
  void on_gear(const GearCommand & msg);
  void on_turn_indicators(const TurnIndicatorsCommand & msg);
  void on_hazard_lights(const HazardLightsCommand & msg);

  rclcpp::Subscription<Control>::SharedPtr sub_control_;
  rclcpp::Subscription<GearCommand>::SharedPtr sub_gear_;
  rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr sub_turn_indicators_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr sub_hazard_lights_;

  std::optional<GearCommand> last_gear_;
  std::optional<TurnIndicatorsCommand> last_turn_indicators_;
  std::optional<HazardLightsCommand> last_hazard_lights_;
};

}  // namespace autoware::control_command_gate

#endif  // COMMAND__SUBSCRIPTION_HPP_
