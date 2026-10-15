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

#pragma once

#include "cloud_concatenator.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

struct ConcatenationDiagnosticsOptions
{
  std::string node_name{"concatenate_data_synchronizer"};
  // If non-empty, the status name is "<node_name>: <diagnostic_name>".
  std::string diagnostic_name{};
  // If set, adds "Processing time (ms)".
  std::optional<double> processing_time_ms{};
  // If set (seconds), adds "Pipeline latency (ms)" and per-topic "Latency (ms): <topic>".
  std::optional<double> now_sec{};
  // True when the cloud was dropped because it is older than the last published one.
  bool drop_previous_but_late{false};
};

struct ReferenceWindow
{
  double time{0.0};
  double noise_window{0.0};
};

// Values from the concatenated frame that the diagnostics are built from.
struct ConcatenationDiagnosticsSummary
{
  double concatenated_cloud_timestamp_sec{0.0};
  bool is_concatenated_cloud_empty{false};
  // Set by the naive strategy.
  std::optional<double> first_arrival_time{};
  // Set by the advanced strategy.
  std::optional<ReferenceWindow> reference_window{};
  std::unordered_map<std::string, double> topic_to_original_stamp{};
};

/// Per-topic pipeline latency in milliseconds: (now_sec - original stamp) for every topic.
std::unordered_map<std::string, double> pipeline_latencies_ms(
  const std::unordered_map<std::string, double> & topic_to_original_stamp, double now_sec);

/// Build the DiagnosticStatus for the concatenation. Per-topic entries follow @p input_topics
/// order. Shared by the node and the offline pipeline.
diagnostic_msgs::msg::DiagnosticStatus build_diagnostic_status(
  const ConcatenationDiagnosticsSummary & summary, const std::vector<std::string> & input_topics,
  const ConcatenationDiagnosticsOptions & options = {});

/// Convenience overload building the summary from an emitted frame (the offline path).
/// @throws std::invalid_argument when the frame carries no concatenated cloud.
diagnostic_msgs::msg::DiagnosticStatus build_diagnostic_status(
  const ConcatenatedFrame<sensor_msgs::msg::PointCloud2> & frame,
  const std::vector<std::string> & input_topics,
  const ConcatenationDiagnosticsOptions & options = {});

}  // namespace autoware::pointcloud_preprocessor
