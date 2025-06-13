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

#ifndef AVAILABILITY_CONVERTER_HPP_
#define AVAILABILITY_CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/command_mode_availability.hpp>
#include <tier4_system_msgs/msg/operation_mode_availability.hpp>

namespace autoware::command_mode_decider
{

using tier4_system_msgs::msg::CommandModeAvailability;
using tier4_system_msgs::msg::CommandModeAvailabilityItem;
using tier4_system_msgs::msg::OperationModeAvailability;

class AvailabilityConverter : public rclcpp::Node
{
public:
  explicit AvailabilityConverter(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<OperationModeAvailability>::SharedPtr sub_operation_mode_;
  rclcpp::Publisher<CommandModeAvailability>::SharedPtr pub_command_mode_;
};

}  // namespace autoware::command_mode_decider

#endif  // AVAILABILITY_CONVERTER_HPP_
