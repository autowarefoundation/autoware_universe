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

#ifndef REDUNDANCY_COMMAND_MODE_DECIDER_HPP_
#define REDUNDANCY_COMMAND_MODE_DECIDER_HPP_

#include <autoware_command_mode_decider/plugin.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>

#include <tier4_system_msgs/msg/active_control_unit.hpp>

#include <memory>
#include <vector>

namespace autoware::command_mode_decider
{

using tier4_system_msgs::msg::ActiveControlUnit;

class RedundancyCommandModeDecider : public DeciderPlugin
{
public:
  void initialize() override;
  uint16_t from_operation_mode(uint16_t operation_mode) override;
  uint16_t to_operation_mode(uint16_t command_mode) override;
  uint16_t to_mrm_behavior(uint16_t command_mode) override;

  std::vector<uint16_t> decide(
    const RequestModeStatus & request, const CommandModeStatusTable & status) override;

private:
  std::unique_ptr<autoware_utils_rclcpp::InterProcessPollingSubscriber<ActiveControlUnit>>
    sub_active_control_unit_;
  std::vector<uint16_t> decide(
    const RequestModeStatus & request, const CommandModeStatusTable & table,
    const std::vector<uint16_t> & last_modes);

  std::vector<uint16_t> last_modes_;
  uint8_t main_ecu_id_;
  uint8_t sub_ecu_id_;
  bool is_sub_ecu_control(const std::vector<uint16_t> & last_modes) const;
};

}  // namespace autoware::command_mode_decider

#endif  // REDUNDANCY_COMMAND_MODE_DECIDER_HPP_
