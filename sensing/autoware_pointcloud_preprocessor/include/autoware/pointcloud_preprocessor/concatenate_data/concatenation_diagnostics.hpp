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
  // The online node uses its fully-qualified name for both fields below.
  std::string node_name{"concatenate_data_synchronizer"};
  // When non-empty, the status name becomes "<node_name>: <diagnostic_name>".
  std::string diagnostic_name{};
  // Runtime-only entries with no offline meaning unless the caller supplies the data:
  // when set, added as "Processing time (ms)".
  std::optional<double> processing_time_ms{};
  // When set (seconds), "Pipeline latency (ms)" and per-topic "Latency (ms): <topic>" are computed
  // as (now - original_stamp) * 1000, exactly as the node does against its wall clock.
  std::optional<double> now_sec{};
  // Reproduces the node's out-of-order-republish guard; offline batches normally leave it false.
  bool drop_previous_but_late{false};
};

// The frame-derived values the diagnostics are built from, decoupled from the frame itself so the
// node can capture them before the concatenated cloud is moved into its publisher (the timing
// options only become known after publishing).
struct ConcatenationDiagnosticsDigest
{
  double concatenated_cloud_timestamp_sec{0.0};
  bool is_concatenated_cloud_empty{false};
  // Which matching context to report: true = advanced (reference window), false = naive (first
  // arrival), nullopt = no context (neither entry is emitted).
  std::optional<bool> is_advanced{};
  double reference_time{0.0};
  double noise_window{0.0};
  double first_arrival_time{0.0};
  std::unordered_map<std::string, double> topic_to_original_stamp{};
};

/// Build the DiagnosticStatus the concatenation publishes on /diagnostics: the key/value entries
/// (fixed order, fixed string formatting), level, and message. @p input_topics fixes the order of
/// the per-topic entries. This is the single implementation behind both the node (which relays the
/// result through its DiagnosticsInterface) and the offline pipeline. Note: "Concatenated:
/// <topic>" reflects whether the topic contributed a cloud, not whether that cloud survived -- a
/// cloud dropped for a missing transform still shows "True"; the finer per-source verdict lives in
/// the concatenation info.
diagnostic_msgs::msg::DiagnosticStatus build_diagnostic_status(
  const ConcatenationDiagnosticsDigest & digest, const std::vector<std::string> & input_topics,
  const ConcatenationDiagnosticsOptions & options = {});

/// Convenience overload building the digest from an emitted frame (the offline path).
/// @throws std::invalid_argument when the frame carries no concatenated cloud.
diagnostic_msgs::msg::DiagnosticStatus build_diagnostic_status(
  const ConcatenatedFrame<sensor_msgs::msg::PointCloud2> & frame,
  const std::vector<std::string> & input_topics,
  const ConcatenationDiagnosticsOptions & options = {});

}  // namespace autoware::pointcloud_preprocessor
