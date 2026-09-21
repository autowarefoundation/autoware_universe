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

#ifndef REDUNDANCY_COMMAND_SELECTOR_HPP_
#define REDUNDANCY_COMMAND_SELECTOR_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_system_msgs/msg/active_control_unit.hpp>

#include <atomic>
#include <cstdint>
#include <string>

namespace autoware::simulator::redundancy_command_selector
{

class RedundancyCommandSelector : public autoware::agnocast_wrapper::Node
{
public:
  explicit RedundancyCommandSelector(const rclcpp::NodeOptions & options);

private:
  using Control = autoware_control_msgs::msg::Control;
  using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
  using HazardLightsCommand = autoware_vehicle_msgs::msg::HazardLightsCommand;
  using TurnIndicatorsCommand = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
  using ActiveControlUnit = tier4_system_msgs::msg::ActiveControlUnit;

  AUTOWARE_PUBLISHER_PTR(Control) pub_control_;
  AUTOWARE_PUBLISHER_PTR(GearCommand) pub_gear_;
  AUTOWARE_PUBLISHER_PTR(HazardLightsCommand) pub_hazard_;
  AUTOWARE_PUBLISHER_PTR(TurnIndicatorsCommand) pub_turn_;

  AUTOWARE_SUBSCRIPTION_PTR(Control) sub_main_control_;
  AUTOWARE_SUBSCRIPTION_PTR(GearCommand) sub_main_gear_;
  AUTOWARE_SUBSCRIPTION_PTR(HazardLightsCommand) sub_main_hazard_;
  AUTOWARE_SUBSCRIPTION_PTR(TurnIndicatorsCommand) sub_main_turn_;

  AUTOWARE_SUBSCRIPTION_PTR(Control) sub_sub_control_;
  AUTOWARE_SUBSCRIPTION_PTR(GearCommand) sub_sub_gear_;
  AUTOWARE_SUBSCRIPTION_PTR(HazardLightsCommand) sub_sub_hazard_;
  AUTOWARE_SUBSCRIPTION_PTR(TurnIndicatorsCommand) sub_sub_turn_;

  AUTOWARE_SUBSCRIPTION_PTR(ActiveControlUnit) sub_active_control_unit_;

  // Creates a subscription that relays the received message to the given publisher
  // only while the active ECU matches `active_when_use_main`.
  template <class MsgT>
  AUTOWARE_SUBSCRIPTION_PTR(MsgT)
  create_relay(
    const std::string & topic, const rclcpp::QoS & qos, const AUTOWARE_PUBLISHER_PTR(MsgT) & pub,
    const bool active_when_use_main)
  {
    return create_subscription<MsgT>(
      topic, qos, [this, pub, active_when_use_main](const typename MsgT::ConstSharedPtr & msg) {
        if (use_main_ == active_when_use_main) {
          pub->publish(*msg);
        }
      });
  }

  std::atomic<bool> use_main_{true};
  uint8_t main_ecu_id_;
  uint8_t sub_ecu_id_;

  void on_active_control_unit(const ActiveControlUnit::ConstSharedPtr & msg);
};

}  // namespace autoware::simulator::redundancy_command_selector

#endif  // REDUNDANCY_COMMAND_SELECTOR_HPP_
