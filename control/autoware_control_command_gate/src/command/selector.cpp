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

#include "selector.hpp"

#include <autoware_command_mode_types/sources.hpp>
#include <rclcpp/logging.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::control_command_gate
{

CommandSelector::CommandSelector(const rclcpp::Logger & logger) : logger_(logger)
{
  builtin_source_ = autoware::command_mode_types::sources::unknown;
  current_source_ = autoware::command_mode_types::sources::unknown;
}

void CommandSelector::add_source(std::unique_ptr<CommandSource> && source)
{
  source->set_output(nullptr);
  sources_.emplace(source->id(), std::move(source));
}

void CommandSelector::set_output(std::unique_ptr<CommandOutput> && output)
{
  output_ = std::move(output);
}

void CommandSelector::update()
{
  if (current_source_ == builtin_source_) {
    return;
  }

  const auto iter = sources_.find(current_source_);
  if (iter == sources_.end()) {
    RCLCPP_ERROR_STREAM(logger_, "Selected source not found. Switched to builtin source.");
    return select_source(builtin_source_);
  }

  const auto & source = iter->second;
  if (source->is_timeout()) {
    RCLCPP_ERROR_STREAM(logger_, "Selected source is timeout. Switched to builtin source.");
    return select_source(builtin_source_);
  }
}

void CommandSelector::select_builtin_source(const uint16_t target)
{
  builtin_source_ = target;
  select_source(builtin_source_);
}

void CommandSelector::select_source(const uint16_t target)
{
  for (auto & [id, source] : sources_) {
    if (id == target) {
      source->set_output(output_.get());
      source->resend_last_command();
    } else {
      source->set_output(nullptr);
    }
  }
  current_source_ = target;
}

std::string CommandSelector::select(const uint16_t target)
{
  const auto iter = sources_.find(target);
  if (iter == sources_.end()) {
    return "target command source is invalid: " + std::to_string(target);
  }
  if (iter->second->is_timeout()) {
    return "target command source is timeout: " + std::to_string(target);
  }
  select_source(target);
  return std::string();
}

}  // namespace autoware::control_command_gate
