// Copyright 2026 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/concatenate_data/concatenation_diagnostics.hpp"

#include "autoware/pointcloud_preprocessor/diagnostics/format_utils.hpp"

#include <diagnostic_msgs/msg/key_value.hpp>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

namespace
{

// Same bool formatting as DiagnosticsInterface.
std::string format_bool(bool value)
{
  return value ? "True" : "False";
}

}  // namespace

diagnostic_msgs::msg::DiagnosticStatus build_diagnostic_status(
  const ConcatenationDiagnosticsSummary & summary, const std::vector<std::string> & input_topics,
  const ConcatenationDiagnosticsOptions & options)
{
  std::vector<diagnostic_msgs::msg::KeyValue> values;
  const auto add = [&values](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    values.push_back(kv);
  };

  add(
    "Concatenated pointcloud timestamp",
    format_timestamp(summary.concatenated_cloud_timestamp_sec));

  if (const auto & window = summary.reference_window) {
    add("Minimum reference timestamp", format_timestamp(window->time - window->noise_window));
    add("Maximum reference timestamp", format_timestamp(window->time + window->noise_window));
  } else if (summary.first_arrival_time) {
    add("First pointcloud arrival timestamp", format_timestamp(*summary.first_arrival_time));
  }

  if (options.processing_time_ms.has_value()) {
    add("Processing time (ms)", std::to_string(*options.processing_time_ms));
  }

  std::unordered_map<std::string, double> topic_to_latency;
  if (options.now_sec.has_value()) {
    double max_latency = 0.0;
    for (const auto & [topic, stamp] : summary.topic_to_original_stamp) {
      const double latency_ms = (*options.now_sec - stamp) * 1000.0;
      topic_to_latency[topic] = latency_ms;
      max_latency = std::max(max_latency, latency_ms);
    }
    add("Pipeline latency (ms)", std::to_string(max_latency));
  }

  bool topic_miss = false;
  for (const auto & topic : input_topics) {
    const auto stamp_it = summary.topic_to_original_stamp.find(topic);
    const bool found = stamp_it != summary.topic_to_original_stamp.end();
    add("Concatenated: " + topic, format_bool(found));
    if (found) {
      add("Timestamp: " + topic, format_timestamp(stamp_it->second));
    } else {
      topic_miss = true;
    }
    const auto latency_it = topic_to_latency.find(topic);
    if (latency_it != topic_to_latency.end()) {
      add("Latency (ms): " + topic, std::to_string(latency_it->second));
    }
  }

  const bool concatenation_success = !topic_miss;
  add("Pointcloud concatenation succeeded", format_bool(concatenation_success));

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "Concatenated pointcloud is published and includes all topics";
  if (options.drop_previous_but_late) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    message = topic_miss ? "Concatenated pointcloud was dropped due to missing topics and because "
                           "its timestamp is earlier than the latest published one"
                         : "Concatenated pointcloud was dropped because its timestamp is earlier "
                           "than the latest published one";
  } else if (summary.is_concatenated_cloud_empty) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    message = "Concatenated pointcloud is empty";
  } else if (topic_miss) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    message = "Concatenated pointcloud is published but misses some topics";
  }

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = level;
  status.name = options.diagnostic_name.empty()
                  ? options.node_name
                  : options.node_name + ": " + options.diagnostic_name;
  status.hardware_id = options.node_name;
  status.message = message;
  status.values = std::move(values);
  return status;
}

}  // namespace autoware::pointcloud_preprocessor
