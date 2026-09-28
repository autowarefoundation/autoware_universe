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
#ifndef LOG_ADAPTER_HPP_
#define LOG_ADAPTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <redundancy_switcher_interface/plugin/event_gateway.hpp>
#include <redundancy_switcher_interface/plugin/i_adapter_plugin.hpp>

#include <memory>

namespace autoware::redundancy_switcher
{

/**
 * @brief Logging-only adapter that processes LogCommand exclusively.
 *
 * Responsibilities:
 *   - Receives LogCommands and outputs them via RCLCPP_* macros.
 *   - Ignores all other OutputCommand types.
 *   - Does not submit any events to the EventGateway (output-only).
 */
class LogAdapter : public IAdapterPlugin
{
public:
  LogAdapter() = default;
  ~LogAdapter() override = default;

  void initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway) override;

  void execute(const OutputCommand & command) override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("redundancy_switcher")};
};

}  // namespace autoware::redundancy_switcher
#endif  // LOG_ADAPTER_HPP_
