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

#ifndef COMMAND_MODE_DECIDER_HPP_
#define COMMAND_MODE_DECIDER_HPP_

#include <autoware_command_mode_decider/plugin.hpp>

#include <vector>

namespace autoware::command_mode_decider
{

class CommandModeDecider : public DeciderPlugin
{
public:
  void initialize() override;
  uint16_t from_operation_mode(uint16_t operation_mode) override;
  uint16_t to_operation_mode(uint16_t command_mode) override;
  uint16_t to_mrm_behavior(uint16_t command_mode) override;

  std::vector<uint16_t> decide(
    const RequestModeStatus & request, const CommandModeStatusTable & table) override;
};

}  // namespace autoware::command_mode_decider

#endif  // COMMAND_MODE_DECIDER_HPP_
