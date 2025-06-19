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

#ifndef COMMAND_MODE_SWITCHER_HPP_
#define COMMAND_MODE_SWITCHER_HPP_

#include "common/command_container.hpp"
#include "common/selector_interface.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/command_mode_request.hpp>
#include <tier4_system_msgs/msg/command_mode_status.hpp>

#include <memory>
#include <unordered_map>
#include <vector>

namespace autoware::command_mode_switcher
{

using tier4_system_msgs::msg::CommandModeRequest;
using tier4_system_msgs::msg::CommandModeStatus;

enum class VehicleModeRequest {
  None,
  Autoware,
  Manual,
};

class CommandModeSwitcher : public rclcpp::Node
{
public:
  explicit CommandModeSwitcher(const rclcpp::NodeOptions & options);

private:
  void on_request(const CommandModeRequest & msg);
  void request_command_mode(std::shared_ptr<Command> command_mode);
  void request_vehicle_mode(VehicleModeRequest vehicle_mode);

  void update();
  void detect_override();
  void update_status();
  void change_modes();
  void publish_command_mode_status();

  void change_vehicle_mode_to_manual();
  void change_vehicle_mode_to_autoware();
  void change_command_mode();

  // ROS interfaces.
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<CommandModeRequest>::SharedPtr sub_request_;
  rclcpp::Publisher<CommandModeStatus>::SharedPtr pub_status_;

  // Mode switching.
  pluginlib::ClassLoader<CommandPlugin> loader_;
  std::vector<std::shared_ptr<Command>> commands_;
  std::unordered_map<uint16_t, std::shared_ptr<Command>> autoware_commands_;
  ControlGateInterface control_gate_interface_;
  NetworkGateInterface network_gate_interface_;
  VehicleGateInterface vehicle_gate_interface_;

  bool prev_manual_control_ = false;
  VehicleModeRequest vehicle_mode_request_ = VehicleModeRequest::None;
  std::shared_ptr<Command> command_mode_request_;
  std::optional<uint8_t> ecu_;
};

}  // namespace autoware::command_mode_switcher

#endif  // COMMAND_MODE_SWITCHER_HPP_
