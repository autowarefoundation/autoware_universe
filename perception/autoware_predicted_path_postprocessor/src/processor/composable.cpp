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

#include "autoware/predicted_path_postprocessor/processor/composable.hpp"

#include "autoware/predicted_path_postprocessor/processor/builder.hpp"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::predicted_path_postprocessor::processor
{
ComposableProcessor::ComposableProcessor(
  rclcpp::Node * node_ptr, const std::vector<std::string> & processor_names)
{
  processors_ = build_processors(node_ptr, processor_names);
}

autoware_perception_msgs::msg::PredictedObjects ComposableProcessor::process(
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects,
  const Context & context) const
{
  auto [results, _] = process_internal(objects, context, false);
  return results;
}

std::pair<
  autoware_perception_msgs::msg::PredictedObjects,
  std::unordered_map<std::string, autoware_perception_msgs::msg::PredictedObjects>>
ComposableProcessor::process_with_intermediates(
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects,
  const Context & context) const
{
  return process_internal(objects, context, true);
}

std::pair<
  autoware_perception_msgs::msg::PredictedObjects,
  std::unordered_map<std::string, autoware_perception_msgs::msg::PredictedObjects>>
ComposableProcessor::process_internal(
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr & objects,
  const Context & context, bool collect_intermediate) const
{
  std::unordered_map<std::string, std::vector<autoware_perception_msgs::msg::PredictedObject>>
    intermediates;

  if (collect_intermediate) {
    // Pre-allocate debug buffer for each processor
    for (const auto & processor : processors_) {
      intermediates[processor->name()].reserve(objects->objects.size());
    }
  }

  std::vector<autoware_perception_msgs::msg::PredictedObject> results;
  results.reserve(objects->objects.size());

  for (const auto & object : objects->objects) {
    auto target = object;

    for (const auto & processor : processors_) {
      processor->process(target, context);

      if (collect_intermediate) {
        intermediates[processor->name()].push_back(target);
      }
    }

    results.push_back(std::move(target));
  }

  // Build final result
  auto final_result =
    autoware_perception_msgs::build<autoware_perception_msgs::msg::PredictedObjects>()
      .header(objects->header)
      .objects(std::move(results));

  // Build debug objects if needed
  std::unordered_map<std::string, autoware_perception_msgs::msg::PredictedObjects> debug_results;
  if (collect_intermediate) {
    debug_results.reserve(processors_.size());

    for (const auto & [processor_name, processed_objects] : intermediates) {
      debug_results[processor_name] =
        autoware_perception_msgs::build<autoware_perception_msgs::msg::PredictedObjects>()
          .header(objects->header)
          .objects(processed_objects);
    }
  }

  return {std::move(final_result), std::move(debug_results)};
}
}  // namespace autoware::predicted_path_postprocessor::processor
