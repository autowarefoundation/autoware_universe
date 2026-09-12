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

#include "collector_info.hpp"
#include "combine_cloud_handler.hpp"
#include "matching_policy.hpp"
#include "matching_strategy_type.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

enum class ConcatenatedFrameStatus {
  kComplete,  // every input topic contributed a cloud
  kTimeout,   // the collector was closed before all topics arrived
};

// One concatenated frame emitted by the concatenator, tagged with why it was emitted and with the
// matching context of the collector that produced it (consumed by build_diagnostic_status()).
template <typename PointCloudMsgT>
struct ConcatenatedFrame
{
  ConcatenatedFrameStatus status;
  ConcatenatedCloudResult<PointCloudMsgT> result;
  // The matching-window reference of the collector: for advanced it is the offset-corrected
  // timestamp of the cloud that opened the collector; for naive it is that cloud's arrival time.
  double reference_time;
  double noise_window;        // 0 for naive matching
  double first_arrival_time;  // arrival time of the cloud that opened the collector
};

// Counters for conditions the caller may want to surface (the node logs throttled warnings from
// them); kept as plain data so the core stays logging-free.
struct CloudConcatenatorStats
{
  // Collectors discarded unpublished because max_open_collectors was reached (input outpacing
  // timeouts).
  std::size_t force_dropped_collectors{0};
  // Clouds that replaced an existing entry for the same topic in their matched collector (usually a
  // symptom of a mis-tuned noise window). last_duplicate_topic names the most recent offender.
  std::size_t duplicate_topic_inserts{0};
  std::string last_duplicate_topic{};
};

/// Stateful, ROS-runtime-free point cloud concatenation: takes a stream of input clouds and
/// emits concatenated frames.
///
/// Owns the full collector lifecycle: matching incoming clouds to open collectors (via the shared
/// MatchingPolicy), opening new collectors, closing them on completion or timeout, and running the
/// combine (via CombineCloudHandler<PointCloudMsgT>). Time is driven by the caller, which makes
/// the same implementation usable both offline and online:
///   - offline: call process_cloud() with each cloud's arrival time (e.g. the rosbag record
///     stamp) in arrival order, then flush() when the stream ends;
///   - online (the concatenation node): subscription callbacks call process_cloud(..., now()),
///     and a one-shot timer re-armed from next_deadline() calls close_expired_collectors(now())
///     so collectors still time out while no clouds arrive.
template <typename PointCloudMsgT>
class CloudConcatenator
{
public:
  /// @throws std::invalid_argument when @p input_topics contains a duplicate, or when the advanced
  /// strategy is requested without its offset/noise-window parameters (size mismatches are rejected
  /// by AdvancedMatchingPolicy).
  /// @param max_open_collectors when non-zero, opening a collector beyond this limit first discards
  /// the open collector with the oldest reference (unpublished), mirroring the node's fixed
  /// collector pool; 0 keeps the collector count unbounded (offline default).
  CloudConcatenator(
    const std::vector<std::string> & input_topics, const std::string & output_frame,
    double timeout_sec, bool is_motion_compensated, bool publish_synchronized_pointcloud,
    bool keep_input_frame_in_synchronized_pointcloud, MatchingStrategyType matching_strategy,
    const std::optional<std::vector<double>> & lidar_timestamp_offsets = std::nullopt,
    const std::optional<std::vector<double>> & lidar_timestamp_noise_window = std::nullopt,
    std::size_t max_open_collectors = 0);

  /// Inject one sensor->output extrinsic (header.frame_id = output frame, child_frame_id = sensor
  /// frame), as provided by TF.
  void set_transform(const geometry_msgs::msg::TransformStamped & sensor_to_output_frame);

  /// Feed twist/odometry (in timestamp order) for motion compensation.
  void process_twist(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr & twist);
  void process_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr & odometry);

  /// Add a cloud received at @p arrival_time (seconds) and return the frames this resolves: any
  /// collectors whose timeout expired at @p arrival_time, plus the cloud's own collector when it
  /// completes. Feed clouds in arrival order. @throws std::invalid_argument for a topic not in
  /// input_topics.
  [[nodiscard]] std::vector<ConcatenatedFrame<PointCloudMsgT>> process_cloud(
    const std::string & topic, typename PointCloudMsgT::ConstSharedPtr cloud, double arrival_time);

  /// Close (and return) every collector whose timeout has expired at @p now_sec, without adding a
  /// cloud. This is the hook the node's timer drives.
  [[nodiscard]] std::vector<ConcatenatedFrame<PointCloudMsgT>> close_expired_collectors(
    double now_sec);

  /// Close every still-open collector as a timeout (call once the stream is exhausted).
  [[nodiscard]] std::vector<ConcatenatedFrame<PointCloudMsgT>> flush();

  /// The next instant (seconds, in the caller's arrival clock) at which an open collector times
  /// out, or nullopt when no collector is open. The node arms its one-shot timer with this.
  [[nodiscard]] std::optional<double> next_deadline() const;

  /// Replenish combine buffers after emitted frames were consumed (a real allocation only for the
  /// CUDA handler; a no-op for PointCloud2). The node calls this after publishing, keeping the
  /// allocation off the publish critical path.
  void allocate_pointclouds() { handler_.allocate_pointclouds(); }

  [[nodiscard]] const CloudConcatenatorStats & stats() const { return stats_; }
  [[nodiscard]] const std::vector<std::string> & input_topics() const { return input_topics_; }

  /// The underlying combine handler, for integrations that need handler-level access (e.g. the
  /// CUDA node hands the handler's per-topic streams to its subscribers).
  [[nodiscard]] CombineCloudHandler<PointCloudMsgT> & combine_cloud_handler() { return handler_; }

private:
  // One open collector: the clouds gathered so far for a single output frame.
  struct Collector
  {
    std::unordered_map<std::string, typename PointCloudMsgT::ConstSharedPtr> topic_to_cloud;
    double reference_time;
    double noise_window;
    double creation_arrival;  // arrival time of the cloud that opened this collector
  };

  // Pass-through that rejects input topic duplication.
  static const std::vector<std::string> & validate_input_topics(
    const std::vector<std::string> & input_topics);

  static std::unique_ptr<MatchingPolicy> make_matching_policy(
    const std::vector<std::string> & input_topics, MatchingStrategyType matching_strategy,
    const std::optional<std::vector<double>> & lidar_timestamp_offsets,
    const std::optional<std::vector<double>> & lidar_timestamp_noise_window);

  // Discard (unpublished) the open collector with the oldest reference to make room for a new one.
  void drop_oldest_collector();

  // Combine the collector's clouds and package them as an emitted frame.
  [[nodiscard]] ConcatenatedFrame<PointCloudMsgT> emit(
    Collector && collector, ConcatenatedFrameStatus status);

  std::vector<std::string> input_topics_;
  double timeout_sec_;
  MatchingStrategyType matching_strategy_;
  std::size_t max_open_collectors_;
  // Declared before handler_ so the advanced-parameter validation rejects bad input before the
  // handler is constructed.
  std::unique_ptr<MatchingPolicy> matching_policy_;
  CombineCloudHandler<PointCloudMsgT> handler_;
  std::vector<Collector> collectors_;
  CloudConcatenatorStats stats_;
};

}  // namespace autoware::pointcloud_preprocessor

#include "cloud_concatenator.ipp"
