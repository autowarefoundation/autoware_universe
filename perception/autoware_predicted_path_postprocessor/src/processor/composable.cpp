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

#include <chrono>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::predicted_path_postprocessor::processor
{
using PredictedObject = autoware_perception_msgs::msg::PredictedObject;
using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;

ComposableProcessor::ComposableProcessor(
  rclcpp::Node * node_ptr, const std::vector<std::string> & processor_names)
{
  processors_ = build_processors(node_ptr, processor_names);
  stopwatch_ = std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
  stopwatch_->tic("processing_time");
}

PredictedObjects ComposableProcessor::process(
  const PredictedObjects::ConstSharedPtr & objects, const Context & context) const
{
  auto [results, _] = process_internal(objects, context, false);
  return results;
}

std::pair<PredictedObjects, std::unordered_map<std::string, IntermediateReport>>
ComposableProcessor::process_with_intermediates(
  const PredictedObjects::ConstSharedPtr & objects, const Context & context) const
{
  return process_internal(objects, context, true);
}

std::pair<PredictedObjects, std::unordered_map<std::string, IntermediateReport>>
ComposableProcessor::process_internal(
  const PredictedObjects::ConstSharedPtr & objects, const Context & context,
  bool collect_intermediate) const
{
  std::unordered_map<std::string, std::pair<std::vector<double>, std::vector<PredictedObject>>>
    intermediates;
  if (collect_intermediate) {
    // Pre-allocate debug buffer for each processor
    for (const auto & processor : processors_) {
      intermediates[processor->name()].first.reserve(objects->objects.size());
      intermediates[processor->name()].second.reserve(objects->objects.size());
    }
  }

  std::vector<PredictedObject> processed_objects;
  processed_objects.reserve(objects->objects.size());

  for (const auto & object : objects->objects) {
    auto target = object;
    for (const auto & processor : processors_) {
      if (collect_intermediate) {
        stopwatch_->toc("processing_time", true);
      }
      processor->process(target, context);
      if (collect_intermediate) {
        intermediates[processor->name()].first.push_back(stopwatch_->toc("processing_time", true));
        intermediates[processor->name()].second.push_back(target);
      }
    }
    processed_objects.push_back(std::move(target));
  }

  // build final output
  auto output = autoware_perception_msgs::build<PredictedObjects>()
                  .header(objects->header)
                  .objects(std::move(processed_objects));

  // build intermediate reports if needed
  std::unordered_map<std::string, IntermediateReport> reports;
  if (collect_intermediate) {
    reports.reserve(processors_.size());
    for (const auto & [key, value] : intermediates) {
      const auto processing_time_ms = std::reduce(value.first.begin(), value.first.end());
      auto processor_result = autoware_perception_msgs::build<PredictedObjects>()
                                .header(objects->header)
                                .objects(value.second);
      reports.emplace(key, IntermediateReport{processing_time_ms, std::move(processor_result)});
    }
  }

  return {std::move(output), std::move(reports)};
}
}  // namespace autoware::predicted_path_postprocessor::processor
