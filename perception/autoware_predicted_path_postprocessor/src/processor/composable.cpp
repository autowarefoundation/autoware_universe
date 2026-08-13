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

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::predicted_path_postprocessor::processor
{
using PredictedObject = autoware_perception_msgs::msg::PredictedObject;

ComposableProcessor::ComposableProcessor(
  rclcpp::Node * node_ptr, const std::vector<std::string> & processor_names)
{
  processors_ = build_processors(node_ptr, processor_names);
  stopwatch_ = std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
  stopwatch_->tic("processing_time");
}

ComposableProcessor::result_type ComposableProcessor::process(
  const target_type::SharedPtr & objects, const Context & context) const
{
  auto result = process_internal(objects, context, false);
  if (result) {
    auto [output, _] = result.ok();
    return make_ok<target_type, error_type>(std::move(output));
  } else {
    return make_err<target_type, error_type>(result.err());
  }
}

ComposableProcessor::result_with_report_type ComposableProcessor::process_with_reports(
  const target_type::SharedPtr & objects, const Context & context) const
{
  return process_internal(objects, context, true);
}

ComposableProcessor::result_with_report_type ComposableProcessor::process_internal(
  const target_type::SharedPtr & objects, const Context & context, bool collect_intermediate) const
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

  std::vector<PredictedObject> processed_targets;
  processed_targets.reserve(objects->objects.size());

  for (auto & target : objects->objects) {
    for (const auto & processor : processors_) {
      if (collect_intermediate) {
        stopwatch_->toc("processing_time", true);
      }

      if (const auto result = processor->run(target, context); !result) {
        return make_err<output_with_report_type, error_type>(result.err());
      }

      if (collect_intermediate) {
        intermediates[processor->name()].first.push_back(stopwatch_->toc("processing_time", true));
        intermediates[processor->name()].second.push_back(target);
      }
    }
    processed_targets.push_back(std::move(target));
  }

  // build final output
  auto output = autoware_perception_msgs::build<target_type>()
                  .header(objects->header)
                  .objects(std::move(processed_targets));

  // build intermediate reports if needed
  report_type reports;
  if (collect_intermediate) {
    reports.reserve(processors_.size());
    for (const auto & [key, value] : intermediates) {
      const auto processing_time_ms = std::reduce(value.first.begin(), value.first.end());
      auto processed_target = autoware_perception_msgs::build<target_type>()
                                .header(objects->header)
                                .objects(value.second);
      reports.emplace(key, IntermediateReport{processing_time_ms, std::move(processed_target)});
    }
  }

  return make_ok<output_with_report_type, error_type>(std::move(output), std::move(reports));
}
}  // namespace autoware::predicted_path_postprocessor::processor
