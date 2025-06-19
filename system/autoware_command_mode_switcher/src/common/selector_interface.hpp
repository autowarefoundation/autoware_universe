//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef COMMON__SELECTOR_INTERFACE_HPP_
#define COMMON__SELECTOR_INTERFACE_HPP_

#include "autoware_command_mode_switcher/command_plugin.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <tier4_external_api_msgs/msg/election_status.hpp>
#include <tier4_system_msgs/msg/command_source_status.hpp>
#include <tier4_system_msgs/srv/select_command_source.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::command_mode_switcher
{

class ControlGateInterface
{
public:
  using Callback = std::function<void()>;
  ControlGateInterface(rclcpp::Node & node, Callback callback);
  bool is_selected(const CommandPlugin & plugin) const;
  bool is_requesting() const { return requesting_; }
  bool is_in_transition() const;
  bool request(const CommandPlugin & plugin, bool transition);

private:
  using SelectCommandSource = tier4_system_msgs::srv::SelectCommandSource;
  using CommandSourceStatus = tier4_system_msgs::msg::CommandSourceStatus;
  void on_source_status(const CommandSourceStatus & msg);

  rclcpp::Node & node_;
  rclcpp::Client<SelectCommandSource>::SharedPtr cli_source_select_;
  rclcpp::Subscription<CommandSourceStatus>::SharedPtr sub_source_status_;

  bool requesting_ = false;
  Callback notification_callback_;
  CommandSourceStatus status_;
};

class VehicleGateInterface
{
public:
  using Callback = std::function<void()>;
  VehicleGateInterface(rclcpp::Node & node, Callback callback);
  bool is_selected(const CommandPlugin & plugin) const;
  bool is_requesting() const { return requesting_; }
  bool is_autoware_control() const;
  bool is_manual_control() const;
  bool request_autoware_control();
  bool request_manual_control();
  bool request(uint8_t control_mode_command);

private:
  using ControlModeCommand = autoware_vehicle_msgs::srv::ControlModeCommand;
  using ControlModeReport = autoware_vehicle_msgs::msg::ControlModeReport;
  void on_control_mode(const ControlModeReport & msg);

  rclcpp::Node & node_;
  rclcpp::Client<ControlModeCommand>::SharedPtr cli_control_mode_;
  rclcpp::Subscription<ControlModeReport>::SharedPtr sub_control_mode_;

  bool requesting_ = false;
  Callback notification_callback_;
  ControlModeReport status_;
};

class NetworkGateInterface
{
public:
  using Callback = std::function<void()>;
  NetworkGateInterface(rclcpp::Node & node, Callback callback);
  bool is_selected(const std::optional<uint8_t> ecu) const;

private:
  using ElectionStatus = tier4_external_api_msgs::msg::ElectionStatus;
  void on_election_status(const ElectionStatus & msg);

  rclcpp::Subscription<ElectionStatus>::SharedPtr sub_election_status_;

  Callback notification_callback_;
  std::optional<uint8_t> ecu_;
};

}  // namespace autoware::command_mode_switcher

#endif  // COMMON__SELECTOR_INTERFACE_HPP_
