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

#ifndef COMMAND__SELECTOR_HPP_
#define COMMAND__SELECTOR_HPP_

#include "interface.hpp"
#include "source.hpp"

#include <rclcpp/logger.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::control_command_gate
{

class CommandSelector
{
public:
  explicit CommandSelector(const rclcpp::Logger & logger);
  void add_source(std::unique_ptr<CommandSource> && source);
  void set_output(std::unique_ptr<CommandOutput> && output);
  void update();
  void select_builtin_source(const uint16_t target);
  std::string select(const uint16_t target);
  uint16_t get_source() const { return current_source_; }

private:
  void select_source(const uint16_t target);

  rclcpp::Logger logger_;
  uint16_t builtin_source_;
  uint16_t current_source_;
  std::unordered_map<uint16_t, std::unique_ptr<CommandSource>> sources_;
  std::unique_ptr<CommandOutput> output_;
};

}  // namespace autoware::control_command_gate

#endif  // COMMAND__SELECTOR_HPP_
