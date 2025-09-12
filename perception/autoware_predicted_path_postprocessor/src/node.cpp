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

#include <memory>
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

  context_ = std::make_unique<Context>();

  auto processor_names = declare_parameter<std::vector<std::string>>("processor_names");
  processor_ = std::make_unique<ComposableProcessor>(this, std::move(processor_names));
}

void PredictedPathPostprocessorNode::callback(
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects)
{
  if (!processor_) {
    RCLCPP_WARN(get_logger(), "Processor has not been initialized");
    return;
  }

  context_->update(objects);

  auto output = processor_->process(objects, *context_);

  object_publisher_->publish(output);
}
}  // namespace autoware::predicted_path_postprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::predicted_path_postprocessor::PredictedPathPostprocessorNode);
