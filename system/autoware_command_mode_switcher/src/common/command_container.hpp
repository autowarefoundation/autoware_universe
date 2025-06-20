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

#ifndef COMMON__COMMAND_CONTAINER_HPP_
#define COMMON__COMMAND_CONTAINER_HPP_

#include "autoware_command_mode_switcher/command_plugin.hpp"

#include <tier4_system_msgs/msg/command_mode_status_item.hpp>

#include <memory>

namespace autoware::command_mode_switcher
{

struct Command
{
  explicit Command(std::shared_ptr<CommandPlugin> plugin);

  std::shared_ptr<CommandPlugin> plugin;
  tier4_system_msgs::msg::CommandModeStatusItem status;
};

}  // namespace autoware::command_mode_switcher

#endif  // COMMON__COMMAND_CONTAINER_HPP_
