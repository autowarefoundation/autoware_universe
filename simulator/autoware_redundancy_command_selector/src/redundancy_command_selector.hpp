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

#ifndef AUTOWARE__REDUNDANCY_COMMAND_SELECTOR__REDUNDANCY_COMMAND_SELECTOR_HPP_
#define AUTOWARE__REDUNDANCY_COMMAND_SELECTOR__REDUNDANCY_COMMAND_SELECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_system_msgs/msg/active_control_unit.hpp>

#include <atomic>
#include <cstdint>

namespace autoware::simulator::redundancy_command_selector
{

class RedundancyCommandSelector : public rclcpp::Node
{
public:
  explicit RedundancyCommandSelector(const rclcpp::NodeOptions & options);

private:
  using Control = autoware_control_msgs::msg::Control;
  using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
  using HazardLightsCommand = autoware_vehicle_msgs::msg::HazardLightsCommand;
  using TurnIndicatorsCommand = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
  using ActiveControlUnit = tier4_system_msgs::msg::ActiveControlUnit;

  rclcpp::Publisher<Control>::SharedPtr pub_control_;
  rclcpp::Publisher<GearCommand>::SharedPtr pub_gear_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr pub_hazard_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr pub_turn_;

  rclcpp::Subscription<Control>::SharedPtr sub_main_control_;
  rclcpp::Subscription<GearCommand>::SharedPtr sub_main_gear_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr sub_main_hazard_;
  rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr sub_main_turn_;

  rclcpp::Subscription<Control>::SharedPtr sub_sub_control_;
  rclcpp::Subscription<GearCommand>::SharedPtr sub_sub_gear_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr sub_sub_hazard_;
  rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr sub_sub_turn_;

  rclcpp::Subscription<ActiveControlUnit>::SharedPtr sub_active_control_unit_;

  std::atomic<bool> use_main_{true};
  uint8_t main_ecu_id_;
  uint8_t sub_ecu_id_;

  void on_active_control_unit(const ActiveControlUnit::ConstSharedPtr & msg);
};

}  // namespace autoware::simulator::redundancy_command_selector

#endif  // AUTOWARE__REDUNDANCY_COMMAND_SELECTOR__REDUNDANCY_COMMAND_SELECTOR_HPP_
