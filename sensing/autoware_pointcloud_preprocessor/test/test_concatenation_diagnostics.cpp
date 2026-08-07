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

// Unit tests for the ROS-runtime-free diagnostics builder: the key/value entries (which appear,
// their order, their string formatting) and the level/message table the node used to build inline.
// The digest is filled exactly as the node fills it, so these tests pin the published status
// without a running node.

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
using autoware::pointcloud_preprocessor::ConcatenationDiagnosticsDigest;
using autoware::pointcloud_preprocessor::ConcatenationDiagnosticsOptions;

const std::vector<std::string> kInputTopics = {"lidar_top", "lidar_left", "lidar_right"};

std::map<std::string, std::string> values_of(const diagnostic_msgs::msg::DiagnosticStatus & status)
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

// A naive digest in which every input topic contributed a cloud stamped at 10.0 s.
ConcatenationDiagnosticsDigest complete_naive_digest()
{
  ConcatenationDiagnosticsDigest digest;
  digest.concatenated_cloud_timestamp_sec = 10.0;
  digest.is_concatenated_cloud_empty = false;
  digest.is_advanced = false;
  digest.first_arrival_time = 100.0;
  for (const auto & topic : kInputTopics) {
    digest.topic_to_original_stamp[topic] = 10.0;
  }
  return digest;
}
}  // namespace

TEST(ConcatenationDiagnostics, CompleteNaiveIsOk)
{
  const auto status = build_diagnostic_status(complete_naive_digest(), kInputTopics);
  const auto values = values_of(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  // The node's DiagnosticsInterface publishes "OK" as the message whenever the level is OK.
  EXPECT_EQ(status.message, "OK");
  EXPECT_EQ(values.at("Concatenated pointcloud timestamp"), "10.000000000");
  EXPECT_EQ(values.at("First pointcloud arrival timestamp"), "100.000000000");
  EXPECT_EQ(values.at("Pointcloud concatenation succeeded"), "True");
  for (const auto & topic : kInputTopics) {
    EXPECT_EQ(values.at("Concatenated: " + topic), "True");
    EXPECT_EQ(values.at("Timestamp: " + topic), "10.000000000");
  }
  // Naive reports the first arrival, never an advanced reference window.
  EXPECT_EQ(values.count("Minimum reference timestamp"), 0u);
  EXPECT_EQ(values.count("Maximum reference timestamp"), 0u);
  // Runtime-only entries are not fabricated when the caller supplies no data.
  EXPECT_EQ(values.count("Processing time (ms)"), 0u);
  EXPECT_EQ(values.count("Pipeline latency (ms)"), 0u);
}

TEST(ConcatenationDiagnostics, EntryOrderFollowsInputTopics)
{
  auto digest = complete_naive_digest();
  ConcatenationDiagnosticsOptions options;
  options.processing_time_ms = 2.5;
  options.now_sec = 10.1;

  // The per-topic entries follow the configured input_topics order, not the map's ordering, and the
  // fixed entries keep the order the node published them in.
  EXPECT_EQ(
    keys_in_order(build_diagnostic_status(digest, kInputTopics, options)),
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
  auto digest = complete_naive_digest();
  digest.topic_to_original_stamp.erase("lidar_left");

  const auto status = build_diagnostic_status(digest, kInputTopics);
  const auto values = values_of(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "Concatenated pointcloud is published but misses some topics");
  EXPECT_EQ(values.at("Pointcloud concatenation succeeded"), "False");
  EXPECT_EQ(values.at("Concatenated: lidar_top"), "True");
  EXPECT_EQ(values.at("Concatenated: lidar_left"), "False");
  EXPECT_EQ(values.count("Timestamp: lidar_left"), 0u);  // never arrived -> no entry
}

TEST(ConcatenationDiagnostics, EmptyCloudIsError)
{
  auto digest = complete_naive_digest();
  digest.is_concatenated_cloud_empty = true;

  const auto status = build_diagnostic_status(digest, kInputTopics);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "Concatenated pointcloud is empty");
  // The concatenation itself still succeeded: every topic contributed.
  EXPECT_EQ(values_of(status).at("Pointcloud concatenation succeeded"), "True");
}

TEST(ConcatenationDiagnostics, LateDropIsErrorAndOutranksTheOtherCauses)
{
  ConcatenationDiagnosticsOptions options;
  options.drop_previous_but_late = true;

  auto digest = complete_naive_digest();
  EXPECT_EQ(
    build_diagnostic_status(digest, kInputTopics, options).message,
    "Concatenated pointcloud was dropped due to its timestamp is earlier than the latest published "
    "one");

  // A missing topic changes the wording; an empty cloud does not outrank the late drop.
  digest.topic_to_original_stamp.erase("lidar_left");
  digest.is_concatenated_cloud_empty = true;
  const auto status = build_diagnostic_status(digest, kInputTopics, options);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(
    status.message,
    "Concatenated pointcloud was dropped due to missing topics and its timestamp is earlier than "
    "the latest published one");
}

TEST(ConcatenationDiagnostics, AdvancedReportsTheReferenceWindow)
{
  auto digest = complete_naive_digest();
  digest.is_advanced = true;
  digest.reference_time = 10.0;
  digest.noise_window = 0.01;

  const auto values = values_of(build_diagnostic_status(digest, kInputTopics));

  EXPECT_EQ(values.at("Minimum reference timestamp"), "9.990000000");
  EXPECT_EQ(values.at("Maximum reference timestamp"), "10.010000000");
  EXPECT_EQ(values.count("First pointcloud arrival timestamp"), 0u);
}

TEST(ConcatenationDiagnostics, NoMatchingContextOmitsBothEntries)
{
  auto digest = complete_naive_digest();
  digest.is_advanced = std::nullopt;

  const auto values = values_of(build_diagnostic_status(digest, kInputTopics));

  EXPECT_EQ(values.count("First pointcloud arrival timestamp"), 0u);
  EXPECT_EQ(values.count("Minimum reference timestamp"), 0u);
}

TEST(ConcatenationDiagnostics, OptionalLatencyAndProcessingTime)
{
  ConcatenationDiagnosticsOptions options;
  options.processing_time_ms = 2.5;
  options.now_sec = 10.1;  // 0.1 s after the 10.0 s stamps -> 100 ms latency

  const auto values =
    values_of(build_diagnostic_status(complete_naive_digest(), kInputTopics, options));

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
    build_diagnostic_status(complete_naive_digest(), kInputTopics, options).name,
    "/sensing/lidar/concatenate_data_synchronizer: concat_status");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
