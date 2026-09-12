// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_
#define AUTOWARE__EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_

#include <autoware/agnocast_wrapper/diagnostic_updater.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/update_functions.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/manual_operator_heartbeat.hpp>
#include <autoware_adapi_v1_msgs/msg/pedals_command.hpp>
#include <autoware_adapi_v1_msgs/msg/steering_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_control_msgs/msg/external_command_selector_mode.hpp>
#include <tier4_control_msgs/srv/external_command_select.hpp>

#include <atomic>
#include <memory>

namespace autoware::external_cmd_selector
{
class ExternalCmdSelector : public autoware::agnocast_wrapper::Node
{
public:
  explicit ExternalCmdSelector(const rclcpp::NodeOptions & node_options);

private:
  using CommandSourceSelect = tier4_control_msgs::srv::ExternalCommandSelect;
  using CommandSourceMode = tier4_control_msgs::msg::ExternalCommandSelectorMode;

  using PedalsCommand = autoware_adapi_v1_msgs::msg::PedalsCommand;
  using SteeringCommand = autoware_adapi_v1_msgs::msg::SteeringCommand;
  using OperatorHeartbeat = autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat;

  using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
  using TurnIndicatorsCommand = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
  using HazardLightsCommand = autoware_vehicle_msgs::msg::HazardLightsCommand;

  // CallbackGroups
  rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
  rclcpp::CallbackGroup::SharedPtr callback_group_services_;

  // Publisher
  AUTOWARE_PUBLISHER_PTR(CommandSourceMode) pub_current_selector_mode_;
  AUTOWARE_PUBLISHER_PTR(PedalsCommand) pub_pedals_cmd_;
  AUTOWARE_PUBLISHER_PTR(SteeringCommand) pub_steering_cmd_;
  AUTOWARE_PUBLISHER_PTR(OperatorHeartbeat) pub_heartbeat_;
  AUTOWARE_PUBLISHER_PTR(GearCommand) pub_gear_cmd_;
  AUTOWARE_PUBLISHER_PTR(TurnIndicatorsCommand) pub_turn_indicators_cmd_;
  AUTOWARE_PUBLISHER_PTR(HazardLightsCommand) pub_hazard_lights_cmd_;

  // Subscriber
  AUTOWARE_SUBSCRIPTION_PTR(PedalsCommand) sub_local_pedals_cmd_;
  AUTOWARE_SUBSCRIPTION_PTR(SteeringCommand) sub_local_steering_cmd_;
  AUTOWARE_SUBSCRIPTION_PTR(OperatorHeartbeat) sub_local_heartbeat_;
  AUTOWARE_SUBSCRIPTION_PTR(GearCommand) sub_local_gear_cmd_;
  AUTOWARE_SUBSCRIPTION_PTR(TurnIndicatorsCommand) sub_local_turn_indicators_cmd_;
  AUTOWARE_SUBSCRIPTION_PTR(HazardLightsCommand) sub_local_hazard_lights_cmd_;

  AUTOWARE_SUBSCRIPTION_PTR(PedalsCommand) sub_remote_pedals_cmd_;
  AUTOWARE_SUBSCRIPTION_PTR(SteeringCommand) sub_remote_steering_cmd_;
  AUTOWARE_SUBSCRIPTION_PTR(OperatorHeartbeat) sub_remote_heartbeat_;
  AUTOWARE_SUBSCRIPTION_PTR(GearCommand) sub_remote_gear_cmd_;
  AUTOWARE_SUBSCRIPTION_PTR(TurnIndicatorsCommand) sub_remote_turn_indicators_cmd_;
  AUTOWARE_SUBSCRIPTION_PTR(HazardLightsCommand) sub_remote_hazard_lights_cmd_;

  template <class T, class F>
  std::function<void(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(T) &)> bind_on_cmd(
    F && func, uint8_t mode)
  {
    return std::bind(func, this, std::placeholders::_1, mode);
  }
  void on_pedals_cmd(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(PedalsCommand) & msg, uint8_t mode);
  void on_steering_cmd(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(SteeringCommand) & msg, uint8_t mode);
  void on_heartbeat(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(OperatorHeartbeat) & msg, uint8_t mode);
  void on_gear_cmd(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(GearCommand) & msg, uint8_t mode);
  void on_turn_indicators_cmd(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(TurnIndicatorsCommand) & msg, uint8_t mode);
  void on_hazard_lights_cmd(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(HazardLightsCommand) & msg, uint8_t mode);

  // Service
  AUTOWARE_SERVICE_PTR(CommandSourceSelect) srv_select_external_command_;
  std::atomic<uint8_t> current_selector_mode_;
  bool on_select_external_command(
    const AUTOWARE_SERVER_REQUEST_PTR(CommandSourceSelect) & req,
    const AUTOWARE_SERVER_RESPONSE_PTR(CommandSourceSelect) & res);

  // Timer
  AUTOWARE_TIMER_PTR timer_;
  void on_timer();

  // Diagnostics Updater
  autoware::agnocast_wrapper::diagnostic_updater::Updater updater_{this};
};
}  // namespace autoware::external_cmd_selector
#endif  // AUTOWARE__EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_
