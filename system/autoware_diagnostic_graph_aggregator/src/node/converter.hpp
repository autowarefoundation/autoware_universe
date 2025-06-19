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

#ifndef NODE__CONVERTER_HPP_
#define NODE__CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/command_mode_availability.hpp>
#include <tier4_system_msgs/msg/operation_mode_availability.hpp>

namespace autoware::diagnostic_graph_aggregator
{

using tier4_system_msgs::msg::CommandModeAvailability;
using tier4_system_msgs::msg::CommandModeAvailabilityItem;
using tier4_system_msgs::msg::OperationModeAvailability;

class ConverterNode : public rclcpp::Node
{
public:
  explicit ConverterNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<CommandModeAvailability>::SharedPtr sub_command_mode_;
  rclcpp::Publisher<OperationModeAvailability>::SharedPtr pub_operation_mode_;
  void on_availability(const CommandModeAvailability & in);

  uint16_t stop_;
  uint16_t autonomous_;
  uint16_t local_;
  uint16_t remote_;
  uint16_t emergency_stop_;
  uint16_t comfortable_stop_;
  uint16_t pull_over_;
  OperationModeAvailability out_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // NODE__CONVERTER_HPP_
