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

// Tests for build_diagnostic_status(): key/value entries, their order and formatting, and the
// level/message rules.

#include "autoware/pointcloud_preprocessor/concatenate_data/concatenation_diagnostics.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <gtest/gtest.h>

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace
{
using autoware::pointcloud_preprocessor::build_diagnostic_status;
using autoware::pointcloud_preprocessor::ConcatenationDiagnosticsOptions;
using autoware::pointcloud_preprocessor::ConcatenationDiagnosticsSummary;
using autoware::pointcloud_preprocessor::ReferenceWindow;

const std::vector<std::string> kInputTopics = {"lidar_top", "lidar_left", "lidar_right"};

std::map<std::string, std::string> key_values_of(
  const diagnostic_msgs::msg::DiagnosticStatus & status)
{
  std::map<std::string, std::string> values;
  for (const auto & kv : status.values) {
    values[kv.key] = kv.value;
  }
  return values;
}

std::vector<std::string> keys_in_order(const diagnostic_msgs::msg::DiagnosticStatus & status)
{
  std::vector<std::string> keys;
  keys.reserve(status.values.size());
  for (const auto & kv : status.values) {
    keys.push_back(kv.key);
  }
  return keys;
}

// Naive summary where every input topic has a cloud stamped at 10.0 s.
ConcatenationDiagnosticsSummary complete_naive_summary()
{
  ConcatenationDiagnosticsSummary summary;
  summary.concatenated_cloud_timestamp_sec = 10.0;
  summary.is_concatenated_cloud_empty = false;
  summary.first_arrival_time = 100.0;
  for (const auto & topic : kInputTopics) {
    summary.topic_to_original_stamp[topic] = 10.0;
  }
  return summary;
}
}  // namespace

TEST(ConcatenationDiagnostics, CompleteNaiveIsOk)
{
  const auto status = build_diagnostic_status(complete_naive_summary(), kInputTopics);
  const auto values = key_values_of(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "Concatenated pointcloud is published and includes all topics");
  EXPECT_EQ(values.at("Concatenated pointcloud timestamp"), "10.000000000");
  EXPECT_EQ(values.at("First pointcloud arrival timestamp"), "100.000000000");
  EXPECT_EQ(values.at("Pointcloud concatenation succeeded"), "True");
  for (const auto & topic : kInputTopics) {
    EXPECT_EQ(values.at("Concatenated: " + topic), "True");
    EXPECT_EQ(values.at("Timestamp: " + topic), "10.000000000");
  }
  EXPECT_EQ(values.count("Minimum reference timestamp"), 0u);
  EXPECT_EQ(values.count("Maximum reference timestamp"), 0u);
  // Not emitted without options.
  EXPECT_EQ(values.count("Processing time (ms)"), 0u);
  EXPECT_EQ(values.count("Pipeline latency (ms)"), 0u);
}

TEST(ConcatenationDiagnostics, EntryOrderFollowsInputTopics)
{
  auto summary = complete_naive_summary();
  ConcatenationDiagnosticsOptions options;
  options.processing_time_ms = 2.5;
  options.now_sec = 10.1;

  // Per-topic entries follow input_topics order.
  EXPECT_EQ(
    keys_in_order(build_diagnostic_status(summary, kInputTopics, options)),
    (std::vector<std::string>{
      "Concatenated pointcloud timestamp", "First pointcloud arrival timestamp",
      "Processing time (ms)", "Pipeline latency (ms)", "Concatenated: lidar_top",
      "Timestamp: lidar_top", "Latency (ms): lidar_top", "Concatenated: lidar_left",
      "Timestamp: lidar_left", "Latency (ms): lidar_left", "Concatenated: lidar_right",
      "Timestamp: lidar_right", "Latency (ms): lidar_right",
      "Pointcloud concatenation succeeded"}));
}

TEST(ConcatenationDiagnostics, MissingTopicIsError)
{
  auto summary = complete_naive_summary();
  summary.topic_to_original_stamp.erase("lidar_left");

  const auto status = build_diagnostic_status(summary, kInputTopics);
  const auto values = key_values_of(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "Concatenated pointcloud is published but misses some topics");
  EXPECT_EQ(values.at("Pointcloud concatenation succeeded"), "False");
  EXPECT_EQ(values.at("Concatenated: lidar_top"), "True");
  EXPECT_EQ(values.at("Concatenated: lidar_left"), "False");
  EXPECT_EQ(values.count("Timestamp: lidar_left"), 0u);
}

TEST(ConcatenationDiagnostics, EmptyConcatenatedCloudIsError)
{
  auto summary = complete_naive_summary();
  summary.is_concatenated_cloud_empty = true;

  const auto status = build_diagnostic_status(summary, kInputTopics);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "Concatenated pointcloud is empty");
  // All topics contributed, so concatenation itself succeeded.
  EXPECT_EQ(key_values_of(status).at("Pointcloud concatenation succeeded"), "True");
}

TEST(ConcatenationDiagnostics, LateDropIsErrorAndOutranksTheOtherCauses)
{
  ConcatenationDiagnosticsOptions options;
  options.drop_previous_but_late = true;

  auto summary = complete_naive_summary();
  EXPECT_EQ(
    build_diagnostic_status(summary, kInputTopics, options).message,
    "Concatenated pointcloud was dropped because its timestamp is earlier than the latest "
    "published one");

  // Missing topic changes the message; empty cloud does not override the late drop.
  summary.topic_to_original_stamp.erase("lidar_left");
  summary.is_concatenated_cloud_empty = true;
  const auto status = build_diagnostic_status(summary, kInputTopics, options);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(
    status.message,
    "Concatenated pointcloud was dropped due to missing topics and because its timestamp is "
    "earlier than the latest published one");
}

TEST(ConcatenationDiagnostics, AdvancedReportsTheReferenceWindow)
{
  auto summary = complete_naive_summary();
  summary.first_arrival_time.reset();
  summary.reference_window = ReferenceWindow{10.0, 0.01};

  const auto values = key_values_of(build_diagnostic_status(summary, kInputTopics));

  EXPECT_EQ(values.at("Minimum reference timestamp"), "9.990000000");
  EXPECT_EQ(values.at("Maximum reference timestamp"), "10.010000000");
  EXPECT_EQ(values.count("First pointcloud arrival timestamp"), 0u);
}

TEST(ConcatenationDiagnostics, NoMatchingContextOmitsBothEntries)
{
  auto summary = complete_naive_summary();
  summary.first_arrival_time.reset();

  const auto values = key_values_of(build_diagnostic_status(summary, kInputTopics));

  EXPECT_EQ(values.count("First pointcloud arrival timestamp"), 0u);
  EXPECT_EQ(values.count("Minimum reference timestamp"), 0u);
}

TEST(ConcatenationDiagnostics, OptionalLatencyAndProcessingTime)
{
  ConcatenationDiagnosticsOptions options;
  options.processing_time_ms = 2.5;
  options.now_sec = 10.1;  // 100 ms after the stamps

  const auto values =
    key_values_of(build_diagnostic_status(complete_naive_summary(), kInputTopics, options));

  EXPECT_EQ(values.at("Processing time (ms)"), "2.500000");
  EXPECT_EQ(values.at("Pipeline latency (ms)"), "100.000000");
  for (const auto & topic : kInputTopics) {
    EXPECT_EQ(values.at("Latency (ms): " + topic), "100.000000");
  }
}

TEST(ConcatenationDiagnostics, DiagnosticNameIsAppendedWhenGiven)
{
  ConcatenationDiagnosticsOptions options;
  options.node_name = "/sensing/lidar/concatenate_data_synchronizer";
  options.diagnostic_name = "concat_status";

  EXPECT_EQ(
    build_diagnostic_status(complete_naive_summary(), kInputTopics, options).name,
    "/sensing/lidar/concatenate_data_synchronizer: concat_status");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
