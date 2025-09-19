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

#include "converter.hpp"

#include <unordered_map>

namespace autoware::diagnostic_graph_aggregator
{

ConverterNode::ConverterNode(const rclcpp::NodeOptions & options) : Node("converter", options)
{
  // Get command mode id from parameter because they are depends on system configuration.
  stop_ = declare_parameter<uint16_t>("stop");
  autonomous_ = declare_parameter<uint16_t>("autonomous");
  local_ = declare_parameter<uint16_t>("local");
  remote_ = declare_parameter<uint16_t>("remote");
  emergency_stop_ = declare_parameter<uint16_t>("emergency_stop");
  comfortable_stop_ = declare_parameter<uint16_t>("comfortable_stop");
  pull_over_ = declare_parameter<uint16_t>("pull_over");

  sub_command_mode_ = create_subscription<CommandModeAvailability>(
    "~/command_mode/availability", 1,
    std::bind(&ConverterNode::on_availability, this, std::placeholders::_1));
  pub_operation_mode_ =
    create_publisher<OperationModeAvailability>("~/operation_mode/availability", rclcpp::QoS(1));
}

void ConverterNode::on_availability(const CommandModeAvailability & in)
{
  std::unordered_map<uint16_t, bool> availability;
  for (const auto & item : in.items) {
    availability[item.mode] = item.available;
  }

  const auto is_available = [availability](uint16_t mode, bool current) {
    const auto iter = availability.find(mode);
    return iter != availability.end() ? iter->second : current;
  };

  out_.stamp = in.stamp;
  out_.stop = is_available(stop_, out_.stop);
  out_.autonomous = is_available(autonomous_, out_.autonomous);
  out_.local = is_available(local_, out_.local);
  out_.remote = is_available(remote_, out_.remote);
  out_.emergency_stop = is_available(emergency_stop_, out_.emergency_stop);
  out_.comfortable_stop = is_available(comfortable_stop_, out_.comfortable_stop);
  out_.pull_over = is_available(pull_over_, out_.pull_over);
  pub_operation_mode_->publish(out_);
}

}  // namespace autoware::diagnostic_graph_aggregator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::diagnostic_graph_aggregator::ConverterNode)
