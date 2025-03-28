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

#ifndef COMMON__PLUGIN_HPP_
#define COMMON__PLUGIN_HPP_

#include "common/context.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/command_mode_status_item.hpp>
#include <tier4_system_msgs/msg/command_source_status.hpp>

#include <memory>
#include <string>

namespace autoware::command_mode_switcher
{

using tier4_system_msgs::msg::CommandModeStatusItem;
using tier4_system_msgs::msg::CommandSourceStatus;

class SwitcherPlugin
{
public:
  void construct(std::shared_ptr<SwitcherContext> context);
  auto status() const { return status_; }

  virtual ~SwitcherPlugin() = default;
  virtual std::string name() const = 0;
  virtual std::string source() const = 0;
  virtual void request(bool activate);
  virtual void on_source_status(const CommandSourceStatus & msg);
  // TODO(Isamu, Takagi): private
protected:
  CommandModeStatusItem status_;
  std::shared_ptr<SwitcherContext> context_;
};

}  // namespace autoware::command_mode_switcher

#endif  // COMMON__PLUGIN_HPP_
