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

#ifndef CONTROL_COMMAND_GATE_HPP_
#define CONTROL_COMMAND_GATE_HPP_

#include "command/compatibility.hpp"
#include "command/filter.hpp"
#include "command/interface.hpp"
#include "command/selector.hpp"

#include <autoware_command_mode_types/sources.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/command_filter_status.hpp>
#include <tier4_system_msgs/msg/command_source_status.hpp>
#include <tier4_system_msgs/srv/change_command_filter.hpp>
#include <tier4_system_msgs/srv/change_command_source.hpp>
#include <tier4_system_msgs/srv/select_command_source.hpp>

#include <memory>
#include <string>

namespace autoware::control_command_gate
{

class ControlCmdGate : public rclcpp::Node
{
public:
  explicit ControlCmdGate(const rclcpp::NodeOptions & options);

private:
  static constexpr uint16_t unknown = autoware::command_mode_types::sources::unknown;
  static constexpr uint16_t builtin = autoware::command_mode_types::sources::builtin;
  using CommandSourceStatus = tier4_system_msgs::msg::CommandSourceStatus;
  using CommandFilterStatus = tier4_system_msgs::msg::CommandFilterStatus;
  using ChangeCommandSource = tier4_system_msgs::srv::ChangeCommandSource;
  using ChangeCommandFilter = tier4_system_msgs::srv::ChangeCommandFilter;

  void on_timer();
  void publish_source_status();
  void publish_filter_status();
  void on_change_source(
    const ChangeCommandSource::Request::SharedPtr req,
    const ChangeCommandSource::Response::SharedPtr res);
  void on_change_filter(
    const ChangeCommandFilter::Request::SharedPtr req,
    const ChangeCommandFilter::Response::SharedPtr res);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<CommandSourceStatus>::SharedPtr pub_source_;
  rclcpp::Publisher<CommandFilterStatus>::SharedPtr pub_filter_;
  rclcpp::Service<ChangeCommandSource>::SharedPtr srv_source_;
  rclcpp::Service<ChangeCommandFilter>::SharedPtr srv_filter_;

  diagnostic_updater::Updater diag_;
  std::unique_ptr<CommandSelector> selector_;
  CommandFilter * output_filter_;
  Compatibility * compatibility_;

  std::optional<uint16_t> current_source_;
  std::optional<bool> transition_flag_;

  // Note: for compatibility.
  using SelectCommandSource = tier4_system_msgs::srv::SelectCommandSource;
  rclcpp::Service<SelectCommandSource>::SharedPtr srv_select_;
  void on_select_source(
    const SelectCommandSource::Request::SharedPtr req,
    const SelectCommandSource::Response::SharedPtr res);
};

}  // namespace autoware::control_command_gate

#endif  // CONTROL_COMMAND_GATE_HPP_
