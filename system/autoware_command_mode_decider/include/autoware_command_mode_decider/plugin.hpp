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

#ifndef AUTOWARE_COMMAND_MODE_DECIDER__PLUGIN_HPP_
#define AUTOWARE_COMMAND_MODE_DECIDER__PLUGIN_HPP_

#include "autoware_command_mode_decider/status.hpp"

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace autoware::command_mode_decider
{

struct RequestModeStatus
{
  bool autoware_control;
  uint16_t operation_mode;
};

class DeciderPlugin
{
public:
  virtual ~DeciderPlugin() = default;
  void construct(rclcpp::Node * node);

  virtual uint16_t from_operation_mode(uint16_t operation_mode) = 0;
  virtual uint16_t to_operation_mode(uint16_t command_mode) = 0;
  virtual uint16_t to_mrm_behavior(uint16_t command_mode) = 0;

  virtual std::vector<uint16_t> decide(
    const RequestModeStatus & request, const CommandModeStatusTable & table) = 0;

protected:
  rclcpp::Node * node_;
};

}  // namespace autoware::command_mode_decider

#endif  // AUTOWARE_COMMAND_MODE_DECIDER__PLUGIN_HPP_
