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
#include <vector>

namespace autoware::pointcloud_preprocessor
{

namespace
{

// std::to_string(double) matches the formatting the node's DiagnosticsInterface uses for doubles
// (fixed, 6 decimal places); bools are published as "True"/"False" there as well.
std::string format_double(double value)
{
  return std::to_string(value);
}

std::string format_bool(bool value)
{
  return value ? "True" : "False";
}

}  // namespace

diagnostic_msgs::msg::DiagnosticStatus build_diagnostic_status(
  const ConcatenationDiagnosticsDigest & digest, const std::vector<std::string> & input_topics,
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
    "Concatenated pointcloud timestamp", format_timestamp(digest.concatenated_cloud_timestamp_sec));

  if (digest.is_advanced.has_value()) {
    if (*digest.is_advanced) {
      add(
        "Minimum reference timestamp",
        format_timestamp(digest.reference_time - digest.noise_window));
      add(
        "Maximum reference timestamp",
        format_timestamp(digest.reference_time + digest.noise_window));
    } else {
      add("First pointcloud arrival timestamp", format_timestamp(digest.first_arrival_time));
    }
  }

  if (options.processing_time_ms.has_value()) {
    add("Processing time (ms)", format_double(*options.processing_time_ms));
  }

  std::unordered_map<std::string, double> topic_to_latency;
  if (options.now_sec.has_value()) {
    double max_latency = 0.0;
    for (const auto & [topic, stamp] : digest.topic_to_original_stamp) {
      const double latency_ms = (*options.now_sec - stamp) * 1000.0;
      topic_to_latency[topic] = latency_ms;
      max_latency = std::max(max_latency, latency_ms);
    }
    add("Pipeline latency (ms)", format_double(max_latency));
  }

  bool topic_miss = false;
  for (const auto & topic : input_topics) {
    const auto stamp_it = digest.topic_to_original_stamp.find(topic);
    const bool found = stamp_it != digest.topic_to_original_stamp.end();
    add("Concatenated: " + topic, format_bool(found));
    if (found) {
      add("Timestamp: " + topic, format_timestamp(stamp_it->second));
    } else {
      topic_miss = true;
    }
    const auto latency_it = topic_to_latency.find(topic);
    if (latency_it != topic_to_latency.end()) {
      add("Latency (ms): " + topic, format_double(latency_it->second));
    }
  }

  const bool concatenation_success = !topic_miss;
  add("Pointcloud concatenation succeeded", format_bool(concatenation_success));

  const bool is_concatenated_cloud_empty = digest.is_concatenated_cloud_empty;
  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string message = "Concatenated pointcloud is published and includes all topics";
  if (options.drop_previous_but_late) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    message = topic_miss ? "Concatenated pointcloud was dropped due to missing topics and its "
                           "timestamp is earlier than the latest published one"
                         : "Concatenated pointcloud was dropped due to its timestamp is earlier "
                           "than the latest published one";
  } else if (is_concatenated_cloud_empty) {
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
  // The node's DiagnosticsInterface publishes "OK" as the message whenever the level is OK (see
  // create_diagnostics_array); mirror that so the output matches /diagnostics verbatim.
  status.message = level == diagnostic_msgs::msg::DiagnosticStatus::OK ? "OK" : message;
  status.values = values;
  return status;
}

}  // namespace autoware::pointcloud_preprocessor
