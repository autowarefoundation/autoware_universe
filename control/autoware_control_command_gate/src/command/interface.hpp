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

#ifndef COMMAND__INTERFACE_HPP_
#define COMMAND__INTERFACE_HPP_

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <memory>
#include <vector>

namespace autoware::control_command_gate
{

using autoware_control_msgs::msg::Control;
using autoware_vehicle_msgs::msg::GearCommand;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

class CommandOutput
{
public:
  virtual ~CommandOutput() = default;
  virtual void on_control(const Control & msg) = 0;
  virtual void on_gear(const GearCommand & msg) = 0;
  virtual void on_turn_indicators(const TurnIndicatorsCommand & msg) = 0;
  virtual void on_hazard_lights(const HazardLightsCommand & msg) = 0;
};

class CommandBridge : public CommandOutput
{
public:
  explicit CommandBridge(std::unique_ptr<CommandOutput> && output);
  void on_control(const Control & msg) override;
  void on_gear(const GearCommand & msg) override;
  void on_turn_indicators(const TurnIndicatorsCommand & msg) override;
  void on_hazard_lights(const HazardLightsCommand & msg) override;

private:
  std::unique_ptr<CommandOutput> output_;
};

}  // namespace autoware::control_command_gate

#endif  // COMMAND__INTERFACE_HPP_
