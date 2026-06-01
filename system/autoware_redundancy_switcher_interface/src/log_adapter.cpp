//  Copyright 2025 The Autoware Contributors
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
#include "log_adapter.hpp"

#include <memory>
#include <stdexcept>
#include <redundancy_switcher_interface/detail/overloaded.hpp>

namespace autoware::redundancy_switcher
{

void LogAdapter::initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> /*gateway*/)
{
  if (!node) {
    throw std::invalid_argument("LogAdapter: node is null");
  }
  logger_ = node->get_logger();
}

void LogAdapter::execute(const OutputCommand & command)
{
  std::visit(
    overloaded{
      [this](const LogCommand & c) {
        switch (c.level) {
          case LogLevel::Debug:
            RCLCPP_DEBUG(logger_, "%s", c.message.c_str());
            break;
          case LogLevel::Info:
            RCLCPP_INFO(logger_, "%s", c.message.c_str());
            break;
          case LogLevel::Warn:
            RCLCPP_WARN(logger_, "%s", c.message.c_str());
            break;
          case LogLevel::Error:
            RCLCPP_ERROR(logger_, "%s", c.message.c_str());
            break;
          case LogLevel::Fatal:
            RCLCPP_FATAL(logger_, "%s", c.message.c_str());
            break;
          default:
            RCLCPP_WARN(logger_, "Received LogCommand with unknown LogLevel: %d. Message: %s", static_cast<int>(c.level), c.message.c_str());
            break;
        }
      },
      [](const auto &) { /* Not this adapter's responsibility — ignore. */ }},
    command);
}

}  // namespace autoware::redundancy_switcher
