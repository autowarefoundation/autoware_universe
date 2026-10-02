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

// Concatenates three point clouds without ever creating or spinning a ROS node.
//
// Everything below links against concatenate_core only: no rclcpp, no tf2_ros, no pcl_ros, no
// executor, no middleware. Clouds, transforms and twists are handed to the core directly, so the
// run is deterministic and every input is guaranteed to be seen - unlike a pub/sub pipeline, where
// a subscriber that comes up late silently misses messages.
//
// Build and run:
//   colcon build --packages-select autoware_pointcloud_preprocessor
//   ./build/autoware_pointcloud_preprocessor/concatenate_without_ros_node
//
// To confirm the core really is ROS-runtime-free:
//   ldd ./build/autoware_pointcloud_preprocessor/concatenate_without_ros_node | grep rclcpp

#include "autoware/pointcloud_preprocessor/concatenate_data/cloud_collector_core.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/collector_info.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/combine_cloud_handler.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/concatenation_diagnostics.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/matching_policy.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/matching_strategy_type.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{

using autoware::point_types::PointXYZIRC;
using autoware::point_types::PointXYZIRCGenerator;
using autoware::pointcloud_preprocessor::AdvancedCollectorInfo;
using autoware::pointcloud_preprocessor::AdvancedMatchingPolicy;
using autoware::pointcloud_preprocessor::build_diagnostic_status;
using autoware::pointcloud_preprocessor::CandidateCollectorState;
using autoware::pointcloud_preprocessor::CloudCollectorCore;
using autoware::pointcloud_preprocessor::CollectorInfoBase;
using autoware::pointcloud_preprocessor::CombineCloudHandler;
using autoware::pointcloud_preprocessor::ConcatenationDiagnosticsOptions;
using autoware::pointcloud_preprocessor::ConcatenationDiagnosticsSummary;
using autoware::pointcloud_preprocessor::IncomingCloudInfo;
using autoware::pointcloud_preprocessor::MatchingStrategyType;
using autoware::pointcloud_preprocessor::ReferenceWindow;
using point_cloud_msg_wrapper::PointCloud2Modifier;
using sensor_msgs::msg::PointCloud2;

const std::vector<std::string> input_topics = {
  "/sensor0/points", "/sensor1/points", "/sensor2/points"};
const std::vector<std::string> sensor_frames = {"sensor0", "sensor1", "sensor2"};
constexpr char output_frame[] = "base_link";

// One cloud per sensor, 10 ms apart, so motion compensation has something to correct.
constexpr double base_stamp_sec = 1000.0;
constexpr double stamp_step_sec = 0.01;

// Defaults sized like a real lidar frame, so the benchmark is dominated by the per-point work
// (transform, motion compensation, append) rather than by fixed overhead. Override on the
// command line - see usage().
constexpr size_t default_points_per_cloud = 40000;
constexpr size_t default_iterations = 200;
constexpr size_t default_warmup_iterations = 20;

builtin_interfaces::msg::Time to_stamp(double seconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(seconds);
  stamp.nanosec = static_cast<uint32_t>((seconds - stamp.sec) * 1e9);
  return stamp;
}

double to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec + stamp.nanosec * 1e-9;
}

// A PointXYZIRC cloud of `num_points` points, offset so each sensor is distinguishable.
PointCloud2::ConstSharedPtr make_cloud(size_t sensor_index, double stamp_sec, size_t num_points)
{
  auto cloud = std::make_shared<PointCloud2>();
  PointCloud2Modifier<PointXYZIRC, PointXYZIRCGenerator> modifier{
    *cloud, sensor_frames[sensor_index]};
  modifier.resize(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    PointXYZIRC point;
    point.x = static_cast<float>(sensor_index * 10 + i);
    point.y = static_cast<float>(sensor_index);
    point.z = 0.0f;
    point.intensity = static_cast<uint8_t>(100 + sensor_index);
    point.return_type = 1;
    point.channel = static_cast<uint16_t>(i % 64);
    modifier[i] = point;
  }

  cloud->header.stamp = to_stamp(stamp_sec);
  return cloud;
}

// Identity rotation, sensor_index metres along y, so each source lands in its own lane.
geometry_msgs::msg::TransformStamped make_transform(size_t sensor_index)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = output_frame;
  transform.child_frame_id = sensor_frames[sensor_index];
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = static_cast<double>(sensor_index);
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  return transform;
}

geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr make_twist(double stamp_sec)
{
  auto twist = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
  twist->header.stamp = to_stamp(stamp_sec);
  twist->header.frame_id = output_frame;
  twist->twist.twist.linear.x = 5.0;  // m/s
  return twist;
}

size_t point_count(const PointCloud2 & cloud)
{
  return cloud.width * cloud.height;
}

// Percentile of an already-sorted sample, nearest-rank.
double percentile_ms(const std::vector<double> & sorted_ms, double fraction)
{
  if (sorted_ms.empty()) return 0.0;
  const auto rank = static_cast<size_t>(fraction * static_cast<double>(sorted_ms.size() - 1));
  return sorted_ms[rank];
}

void print_timing(const std::vector<double> & samples_ms, size_t total_points)
{
  std::vector<double> sorted_ms = samples_ms;
  std::sort(sorted_ms.begin(), sorted_ms.end());

  const double sum = std::accumulate(sorted_ms.begin(), sorted_ms.end(), 0.0);
  const double mean = sum / static_cast<double>(sorted_ms.size());

  double variance = 0.0;
  for (const double sample : sorted_ms) {
    variance += (sample - mean) * (sample - mean);
  }
  variance /= static_cast<double>(sorted_ms.size());
  const double stddev = std::sqrt(variance);

  const auto previous_precision = std::cout.precision();
  std::cout << std::fixed << std::setprecision(3) << "concatenation time over " << sorted_ms.size()
            << " runs\n"
            << "  average : " << mean << " ms\n"
            << "  stddev  : " << stddev << " ms\n"
            << "  min     : " << sorted_ms.front() << " ms\n"
            << "  median  : " << percentile_ms(sorted_ms, 0.50) << " ms\n"
            << "  p95     : " << percentile_ms(sorted_ms, 0.95) << " ms\n"
            << "  max     : " << sorted_ms.back() << " ms\n";
  if (mean > 0.0) {
    std::cout << std::setprecision(2)
              << "  rate    : " << (static_cast<double>(total_points) / mean) / 1000.0
              << " Mpoint/s (" << 1000.0 / mean << " concatenations/s)\n";
  }
  std::cout << std::defaultfloat << std::setprecision(static_cast<int>(previous_precision));
}

void usage(const char * program)
{
  std::cout << "usage: " << program << " [points_per_cloud] [iterations] [warmup]\n"
            << "  points_per_cloud  points in each source cloud (default "
            << default_points_per_cloud << ")\n"
            << "  iterations        timed concatenations   (default " << default_iterations << ")\n"
            << "  warmup            untimed warm-up runs   (default " << default_warmup_iterations
            << ")\n";
}

size_t parse_size(const char * text, size_t fallback)
{
  char * end = nullptr;
  const uint64_t value = std::strtoull(text, &end, 10);
  if (end == text || value == 0) return fallback;
  return static_cast<size_t>(value);
}

bool check(bool condition, const std::string & what)
{
  std::cout << (condition ? "  ok   " : "  FAIL ") << what << "\n";
  return condition;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc > 1 && std::string(argv[1]) == "--help") {
    usage(argv[0]);
    return EXIT_SUCCESS;
  }
  const size_t points_per_cloud =
    argc > 1 ? parse_size(argv[1], default_points_per_cloud) : default_points_per_cloud;
  const size_t iterations = argc > 2 ? parse_size(argv[2], default_iterations) : default_iterations;
  const size_t warmup_iterations =
    argc > 3 ? parse_size(argv[3], default_warmup_iterations) : default_warmup_iterations;

  // ---------------------------------------------------------------------------------------
  // 1. Build the core. No node, no parameters server - the configuration is just arguments.
  // ---------------------------------------------------------------------------------------
  CombineCloudHandler<PointCloud2> handler(
    input_topics, output_frame, /*is_motion_compensated=*/true,
    /*publish_synchronized_pointcloud=*/false,
    /*keep_input_frame_in_synchronized_pointcloud=*/false, MatchingStrategyType::advanced);

  // Transforms the node would look up from tf2. Here they are handed over directly, which is what
  // makes an offline run reproducible: no TF listener, no waiting, no lookup failures.
  for (size_t i = 0; i < sensor_frames.size(); ++i) {
    handler.set_transform(make_transform(i));
  }

  // Ego motion the node would take from /twist. Without this the core reports
  // kNoTwistAvailable and leaves the clouds untransformed.
  handler.process_twist(make_twist(base_stamp_sec));

  // ---------------------------------------------------------------------------------------
  // 2. Group the clouds exactly as the node's collectors would, using the same policy object.
  //    This is the part the node drives from subscription callbacks and a timeout timer; here
  //    it is a plain loop over inputs that are already in hand.
  // ---------------------------------------------------------------------------------------
  const std::vector<double> offsets(input_topics.size(), 0.0);
  const std::vector<double> noise_windows(input_topics.size(), 0.02);
  const AdvancedMatchingPolicy policy(input_topics, offsets, noise_windows);

  // The same collector the node uses, minus the timer and the node back-pointer.
  CloudCollectorCore<PointCloud2> collector(input_topics.size(), /*timeout_sec=*/0.2);
  std::vector<CandidateCollectorState> collectors;
  bool ready = false;

  for (size_t i = 0; i < input_topics.size(); ++i) {
    const double stamp_sec = base_stamp_sec + static_cast<double>(i) * stamp_step_sec;
    auto cloud = make_cloud(i, stamp_sec, points_per_cloud);

    IncomingCloudInfo incoming;
    incoming.topic_name = input_topics[i];
    incoming.cloud_timestamp = stamp_sec;
    incoming.cloud_arrival_time = stamp_sec;  // offline: arrival order follows stamp order

    if (!policy.match(collectors, incoming).has_value()) {
      // No collector wants this cloud, so it opens one - same decision the node makes.
      const auto reference = policy.reference_for(incoming);
      collectors.push_back({reference.reference_time, reference.noise_window, false});
      collector.set_info(
        std::make_shared<AdvancedCollectorInfo>(reference.reference_time, reference.noise_window));
    }
    collectors.front().has_topic = true;

    // Offline the arrival time comes from the bag, not a wall clock, so the collector can answer
    // is_timed_out() itself - no rclcpp timer anywhere.
    const auto added = collector.add(input_topics[i], std::move(cloud), stamp_sec);
    ready = added.ready_to_concatenate;
  }

  auto topic_to_cloud_map = collector.topic_to_cloud_map();
  const auto collector_info = collector.get_info();

  // ---------------------------------------------------------------------------------------
  // 3. Concatenate. One call, fully synchronous, no executor involved.
  // ---------------------------------------------------------------------------------------
  auto result = handler.combine_pointclouds(topic_to_cloud_map, collector_info);
  collector.mark_finished();

  // ---------------------------------------------------------------------------------------
  // 3b. Time that one call, and nothing else.
  //
  // Building the clouds, the transforms, the twist and the grouping all happened above and stay
  // outside the clock. combine_pointclouds() only reads topic_to_cloud_map, so re-running it is
  // side-effect free and every iteration does identical work. The returned result is destroyed
  // after the clock is read, so deallocation is not counted - matching the node, which hands the
  // result straight to the publisher.
  // ---------------------------------------------------------------------------------------
  using clock = std::chrono::steady_clock;
  volatile size_t sink = 0;  // keeps the optimizer from eliding the call

  for (size_t i = 0; i < warmup_iterations; ++i) {
    auto warmup = handler.combine_pointclouds(topic_to_cloud_map, collector_info);
    sink += warmup.concatenate_cloud_ptr->data.size();
  }

  std::vector<double> samples_ms;
  samples_ms.reserve(iterations);
  for (size_t i = 0; i < iterations; ++i) {
    const auto started = clock::now();
    auto timed = handler.combine_pointclouds(topic_to_cloud_map, collector_info);
    const auto finished = clock::now();

    samples_ms.push_back(std::chrono::duration<double, std::milli>(finished - started).count());
    sink += timed.concatenate_cloud_ptr->data.size();
  }
  (void)sink;

  // ---------------------------------------------------------------------------------------
  // 4. Report, and check what the node would have published on its topics.
  // ---------------------------------------------------------------------------------------
  const auto & cloud = *result.concatenate_cloud_ptr;
  const auto & info = *result.concatenation_info_ptr;

  std::cout << "concatenated cloud\n"
            << "  frame_id   : " << cloud.header.frame_id << "\n"
            << "  stamp      : " << to_seconds(cloud.header.stamp) << "\n"
            << "  points     : " << point_count(cloud) << "\n"
            << "  point_step : " << cloud.point_step << "\n"
            << "  sources    : " << info.source_info.size() << "\n"
            << "  collectors : " << collectors.size() << "\n\n";

  print_timing(samples_ms, point_count(cloud));
  std::cout << "\n";

  bool all_ok = true;
  all_ok &= check(cloud.header.frame_id == output_frame, "cloud is in the output frame");
  all_ok &= check(
    point_count(cloud) == points_per_cloud * input_topics.size(),
    "every source contributed its points");
  all_ok &=
    check(to_seconds(cloud.header.stamp) == base_stamp_sec, "stamp is the oldest input stamp");
  all_ok &= check(collectors.size() == 1, "all three clouds landed in one collector");
  all_ok &= check(ready, "the collector reported the group complete");
  all_ok &=
    check(!collector.is_timed_out(base_stamp_sec + 0.1), "the group completed before its timeout");
  all_ok &= check(info.source_info.size() == input_topics.size(), "info describes every source");
  all_ok &= check(
    result.motion_compensation_status ==
      autoware::pointcloud_preprocessor::MotionCompensationStatus::kValid,
    "motion compensation ran against the supplied twist");
  all_ok &= check(
    result.dropped_sources_missing_transform.empty(), "no source was dropped for a missing TF");

  // The diagnostics the node publishes are built by the core too, so they can be inspected
  // offline without a diagnostic_updater.
  ConcatenationDiagnosticsSummary summary;
  summary.concatenated_cloud_timestamp_sec = to_seconds(cloud.header.stamp);
  summary.is_concatenated_cloud_empty = point_count(cloud) == 0;
  summary.reference_window = ReferenceWindow{base_stamp_sec, noise_windows.front()};
  summary.topic_to_original_stamp = result.topic_to_original_stamp_map;

  ConcatenationDiagnosticsOptions options;
  options.node_name = "concatenate_without_ros_node";
  const auto status = build_diagnostic_status(summary, input_topics, options);

  std::cout << "\ndiagnostics\n  level   : " << static_cast<int>(status.level)
            << "\n  message : " << status.message << "\n";
  for (const auto & key_value : status.values) {
    std::cout << "  " << key_value.key << " = " << key_value.value << "\n";
  }
  std::cout << "\n";
  all_ok &= check(
    status.message.find("includes all topics") != std::string::npos,
    "diagnostics report a complete concatenation");

  std::cout << "\n" << (all_ok ? "PASS" : "FAIL") << ": concatenation ran without a ROS node\n";
  return all_ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
