// Copyright 2025 TIER IV, Inc.
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

#include "autoware/predicted_path_postprocessor/node.hpp"

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <chrono>
#include <memory>
#include <ratio>
#include <string>
#include <utility>
#include <vector>

namespace autoware::predicted_path_postprocessor
{
PredictedPathPostprocessorNode::PredictedPathPostprocessorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("predicted_path_postprocessor", options)
{
  // NOTE: add additional subscriptions if needed, such as lanelet, traffic light, etc.
  object_subscription_ = create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    [this](const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & msg) {
      this->callback(msg);
    });

  object_publisher_ = create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
    "~/output/objects", rclcpp::QoS{1});

  context_ = std::make_unique<processor::Context>();

  const auto processor_names = declare_parameter<std::vector<std::string>>("processor_names");

  processor_ = std::make_unique<processor::ComposableProcessor>(this, processor_names);

  stopwatch_ = std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
  stopwatch_->tic("cyclic_time");
  stopwatch_->tic("processing_time");
  debug_publisher_ = std::make_unique<autoware_utils_debug::DebugPublisher>(this, get_name());

  debug_ = declare_parameter<bool>("debug");
}

void PredictedPathPostprocessorNode::callback(
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects)
{
  stopwatch_->toc("processing_time", true);
  context_->update(objects);

  if (debug_) {
    auto [output, intermediates] = processor_->process_with_intermediates(objects, *context_);
    object_publisher_->publish(output);

    for (const auto & [processor_name, intermediate] : intermediates) {
      debug_publisher_->publish<autoware_perception_msgs::msg::PredictedObjects>(
        "debug/" + processor_name + "/objects", intermediate);
    }
  } else {
    auto output = processor_->process(objects, *context_);
    object_publisher_->publish(output);
  }

  const auto cyclic_time_ms = stopwatch_->toc("cyclic_time", true);
  const auto processing_time_ms = stopwatch_->toc("processing_time", true);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/cyclic_time_ms", cyclic_time_ms);
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", processing_time_ms);
}
}  // namespace autoware::predicted_path_postprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::predicted_path_postprocessor::PredictedPathPostprocessorNode);
