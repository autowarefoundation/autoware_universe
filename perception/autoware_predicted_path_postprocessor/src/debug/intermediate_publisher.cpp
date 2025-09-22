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

#include "autoware/predicted_path_postprocessor/debug/intermediate_publisher.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::predicted_path_postprocessor::debug
{
IntermediatePublisher::IntermediatePublisher(
  rclcpp::Node * node_ptr, const std::vector<std::string> & processor_names)
{
  auto resolve_topic_name = [&node_ptr](size_t i, const std::string & name) {
    return node_ptr->get_node_topics_interface()->resolve_topic_name(
      "~/debug/processor" + std::to_string(i) + "/" + name + "/output/objects");
  };

  for (size_t i = 0; i < processor_names.size(); ++i) {
    const auto & name = processor_names[i];
    auto topic_name = resolve_topic_name(i, name);
    publishers_.emplace(
      name, node_ptr->create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
              topic_name, rclcpp::QoS{1}));
  }
}

void IntermediatePublisher::publish(
  const std::unordered_map<std::string, autoware_perception_msgs::msg::PredictedObjects> &
    intermediates)
{
  for (const auto & [name, objects] : intermediates) {
    publishers_[name]->publish(objects);
  }
}
}  // namespace autoware::predicted_path_postprocessor::debug
