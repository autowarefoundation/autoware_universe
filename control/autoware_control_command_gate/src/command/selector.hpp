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
  using SourceChangeCallback = std::function<void(const std::string &)>;

  CommandSelector(const rclcpp::Logger & logger, SourceChangeCallback on_change_source);
  void add_source(std::unique_ptr<CommandSource> && source);
  void set_output(std::unique_ptr<CommandOutput> && output);
  void update();
  bool select(const std::string & name);
  void select_builtin_source(const std::string & name) { builtin_source_ = name; }

private:
  rclcpp::Logger logger_;
  SourceChangeCallback on_change_source_;
  std::string builtin_source_;
  std::string current_source_;
  std::unordered_map<std::string, std::unique_ptr<CommandSource>> sources_;
  std::unique_ptr<CommandOutput> output_;
};

}  // namespace autoware::control_command_gate

#endif  // COMMAND__SELECTOR_HPP_
