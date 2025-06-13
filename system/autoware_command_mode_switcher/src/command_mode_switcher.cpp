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

#include "command_mode_switcher.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::command_mode_switcher
{

CommandModeSwitcher::CommandModeSwitcher(const rclcpp::NodeOptions & options)
: Node("command_mode_switcher", options),
  loader_("autoware_command_mode_switcher", "autoware::command_mode_switcher::SwitcherPlugin")
{
  pub_status_ =
    create_publisher<CommandModeStatus>("~/command_mode/status", rclcpp::QoS(1).transient_local());
  sub_request_ = create_subscription<CommandModeRequest>(
    "~/command_mode/request", rclcpp::QoS(1),
    std::bind(&CommandModeSwitcher::on_request, this, std::placeholders::_1));

  sub_source_status_ = create_subscription<CommandSourceStatus>(
    "~/source/status", rclcpp::QoS(1).transient_local(),
    std::bind(&CommandModeSwitcher::on_source_status, this, std::placeholders::_1));

  // Init switchers
  {
    const auto context = std::make_shared<SwitcherContext>(*this);
    const auto plugins = declare_parameter<std::vector<std::string>>("plugins");
    for (const auto & plugin : plugins) {
      if (!loader_.isClassAvailable(plugin)) {
        RCLCPP_WARN_STREAM(get_logger(), "ignore unknown plugin: " << plugin);
        continue;
      }
      const auto switcher = loader_.createSharedInstance(plugin);
      if (switchers_.count(switcher->name())) {
        RCLCPP_WARN_STREAM(get_logger(), "ignore duplicate plugin: " << plugin);
        continue;
      }
      switcher->construct(context);
      switchers_[switcher->name()] = switcher;
    }
  }

  const auto period = rclcpp::Rate(declare_parameter<double>("update_rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void CommandModeSwitcher::on_timer()
{
}

void CommandModeSwitcher::on_source_status(const CommandSourceStatus & msg)
{
  for (const auto & [mode, switcher] : switchers_) {
    switcher->on_source_status(msg);
  }
  publish_command_mode_status();
}

void CommandModeSwitcher::on_request(const CommandModeRequest & msg)
{
  const auto iter = switchers_.find(msg.mode);
  if (iter == switchers_.end()) {
    RCLCPP_ERROR_STREAM(get_logger(), "CommandModeSwitcher on_request: invalid mode: " << msg.mode);
    return;
  }

  for (const auto & [mode, switcher] : switchers_) {
    switcher->request(mode == msg.mode);
  }
}

void CommandModeSwitcher::publish_command_mode_status()
{
  CommandModeStatus msg;
  msg.stamp = now();
  for (const auto & [mode, switcher] : switchers_) {
    msg.items.push_back(switcher->status());
  }
  pub_status_->publish(msg);
}

}  // namespace autoware::command_mode_switcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::command_mode_switcher::CommandModeSwitcher)
