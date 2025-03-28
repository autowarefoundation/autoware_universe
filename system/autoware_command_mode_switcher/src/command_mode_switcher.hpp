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

#include "common/plugin.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/command_mode_request.hpp>
#include <tier4_system_msgs/msg/command_mode_status.hpp>
#include <tier4_system_msgs/msg/command_source_status.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::command_mode_switcher
{

using tier4_system_msgs::msg::CommandModeRequest;
using tier4_system_msgs::msg::CommandModeStatus;
using tier4_system_msgs::msg::CommandModeStatusItem;
using tier4_system_msgs::msg::CommandSourceStatus;

class CommandModeSwitcher : public rclcpp::Node
{
public:
  explicit CommandModeSwitcher(const rclcpp::NodeOptions & options);

private:
  void on_timer();
  void on_request(const CommandModeRequest & msg);
  void publish_command_mode_status();

  void on_source_status(const CommandSourceStatus & msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<CommandModeStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<CommandModeRequest>::SharedPtr sub_request_;

  rclcpp::Subscription<CommandSourceStatus>::SharedPtr sub_source_status_;

  pluginlib::ClassLoader<SwitcherPlugin> loader_;
  std::unordered_map<std::string, std::shared_ptr<SwitcherPlugin>> switchers_;
};

}  // namespace autoware::command_mode_switcher

#endif  // COMMAND_MODE_SWITCHER_HPP_
