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

#include "autoware/pointcloud_preprocessor/concatenate_data/cloud_concatenator.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/time_utils.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

template <typename PointCloudMsgT>
std::unique_ptr<MatchingPolicy> CloudConcatenator<PointCloudMsgT>::make_matching_policy(
  const std::vector<std::string> & input_topics, MatchingStrategyType matching_strategy,
  const std::optional<std::vector<double>> & lidar_timestamp_offsets,
  const std::optional<std::vector<double>> & lidar_timestamp_noise_window)
{
  if (matching_strategy == MatchingStrategyType::advanced) {
    if (!lidar_timestamp_offsets.has_value() || !lidar_timestamp_noise_window.has_value()) {
      throw std::invalid_argument(
        "advanced matching requires lidar_timestamp_offsets and lidar_timestamp_noise_window "
        "(one per input topic)");
    }
    return std::make_unique<AdvancedMatchingPolicy>(
      input_topics, *lidar_timestamp_offsets, *lidar_timestamp_noise_window);
  }
  return std::make_unique<NaiveMatchingPolicy>();
}

template <typename PointCloudMsgT>
CloudConcatenator<PointCloudMsgT>::CloudConcatenator(
  const std::vector<std::string> & input_topics, const std::string & output_frame,
  double timeout_sec, bool is_motion_compensated, bool publish_synchronized_pointcloud,
  bool keep_input_frame_in_synchronized_pointcloud, MatchingStrategyType matching_strategy,
  const std::optional<std::vector<double>> & lidar_timestamp_offsets,
  const std::optional<std::vector<double>> & lidar_timestamp_noise_window,
  std::size_t max_open_collectors, LateCloudPolicy late_cloud_policy)
: input_topics_(input_topics),
  timeout_sec_(timeout_sec),
  matching_strategy_(matching_strategy),
  max_open_collectors_(max_open_collectors),
  late_cloud_policy_(late_cloud_policy),
  matching_policy_(make_matching_policy(
    input_topics, matching_strategy, lidar_timestamp_offsets, lidar_timestamp_noise_window)),
  handler_(
    input_topics, output_frame, is_motion_compensated, publish_synchronized_pointcloud,
    keep_input_frame_in_synchronized_pointcloud, matching_strategy)
{
}

template <typename PointCloudMsgT>
bool CloudConcatenator<PointCloudMsgT>::set_transform(
  const geometry_msgs::msg::TransformStamped & sensor_to_output_frame)
{
  return handler_.set_transform(sensor_to_output_frame);
}

template <typename PointCloudMsgT>
void CloudConcatenator<PointCloudMsgT>::process_twist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr & twist)
{
  handler_.process_twist(twist);
}

template <typename PointCloudMsgT>
void CloudConcatenator<PointCloudMsgT>::process_odometry(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry)
{
  handler_.process_odometry(odometry);
}

template <typename PointCloudMsgT>
std::vector<ConcatenatedFrame<PointCloudMsgT>> CloudConcatenator<PointCloudMsgT>::process_cloud(
  const std::string & topic, typename PointCloudMsgT::ConstSharedPtr cloud, double arrival_time)
{
  if (std::find(input_topics_.begin(), input_topics_.end(), topic) == input_topics_.end()) {
    throw std::invalid_argument(
      "unknown topic '" + topic + "'; expected one of the configured input_topics");
  }

  // (1) Timeout: close any collector whose timer has expired in arrival time (it can no longer
  // receive more clouds).
  std::vector<ConcatenatedFrame<PointCloudMsgT>> outputs = close_expired_collectors(arrival_time);

  const IncomingCloudInfo incoming_cloud_info{topic, to_seconds(cloud->header.stamp), arrival_time};

  // (2) Match against the open collectors.
  std::vector<CandidateCollectorState> candidates;
  candidates.reserve(collectors_.size());
  for (const auto & collector : collectors_) {
    candidates.push_back(
      {collector.reference_time, collector.noise_window,
       collector.topic_to_cloud.count(topic) > 0});
  }
  const std::optional<std::size_t> matched_index =
    matching_policy_->match(candidates, incoming_cloud_info);

  // (3) No match -> start a new collector (matching window from the policy-provided reference,
  // timeout starting at this cloud's arrival), unless the cloud is late (see LateCloudPolicy). When
  // the collector limit is reached, make room first by discarding the collector with the oldest
  // reference (mirrors the node's fixed collector pool).
  std::size_t index = 0;
  if (matched_index.has_value()) {
    index = *matched_index;
  } else {
    const MatchingReference reference = matching_policy_->reference_for(incoming_cloud_info);
    if (is_late(frame_of(reference, incoming_cloud_info))) {
      ++stats_.late_clouds_rejected;
      stats_.last_late_topic = topic;
      Collector alone{{}, reference.reference_time, reference.noise_window, arrival_time};
      alone.topic_to_cloud[topic] = std::move(cloud);
      outputs.push_back(emit(std::move(alone), ConcatenatedFrameStatus::kLate));
      return outputs;
    }
    while (max_open_collectors_ != 0 && collectors_.size() >= max_open_collectors_) {
      drop_oldest_collector();
    }
    collectors_.push_back(
      Collector{{}, reference.reference_time, reference.noise_window, arrival_time});
    index = collectors_.size() - 1;
  }

  // (4) Add the cloud; emit immediately if the collector is now complete.
  auto & topic_to_cloud = collectors_[index].topic_to_cloud;
  const bool replaced_existing_cloud =
    !topic_to_cloud.insert_or_assign(topic, std::move(cloud)).second;
  if (replaced_existing_cloud) {
    // Usually a symptom of a mis-tuned noise window; the newer cloud wins, as in the node.
    ++stats_.duplicate_topic_inserts;
    stats_.last_duplicate_topic = topic;
  }
  if (topic_to_cloud.size() == input_topics_.size()) {
    Collector completed = std::move(collectors_[index]);
    collectors_.erase(collectors_.begin() + static_cast<std::ptrdiff_t>(index));
    outputs.push_back(emit(std::move(completed), ConcatenatedFrameStatus::kComplete));
  }

  return outputs;
}

template <typename PointCloudMsgT>
std::vector<ConcatenatedFrame<PointCloudMsgT>>
CloudConcatenator<PointCloudMsgT>::close_expired_collectors(double now_sec)
{
  // A collector expires once its deadline is reached, or once the caller's clock has gone back past
  // its creation (a rosbag restarted under use_sim_time), since it would otherwise wait for that
  // time to return. The deadline is the same sum next_deadline() reports, so a timer armed exactly
  // at it closes the collector (now - creation >= timeout can round just short of it).
  const bool clock_went_back = now_sec < last_time_sec_;
  last_time_sec_ = now_sec;
  std::vector<Collector> stale;
  std::vector<Collector> open;
  for (auto & collector : collectors_) {
    if (
      collector.creation_arrival + timeout_sec_ <= now_sec ||
      now_sec < collector.creation_arrival) {
      stale.push_back(std::move(collector));
    } else {
      open.push_back(std::move(collector));
    }
  }
  collectors_ = std::move(open);

  std::sort(stale.begin(), stale.end(), [](const Collector & a, const Collector & b) {
    return a.creation_arrival < b.creation_arrival;
  });

  std::vector<ConcatenatedFrame<PointCloudMsgT>> outputs;
  outputs.reserve(stale.size());
  for (auto & collector : stale) {
    outputs.push_back(emit(std::move(collector), ConcatenatedFrameStatus::kTimeout));
  }
  if (clock_went_back) {
    // The replayed stream starts over, and the collectors just closed belong to the time before
    // the jump, so they must not set the bar for it.
    emitted_frame_watermark_.reset();
  }
  return outputs;
}

template <typename PointCloudMsgT>
double CloudConcatenator<PointCloudMsgT>::frame_of(
  const MatchingReference & reference, const IncomingCloudInfo & incoming_cloud_info) const
{
  return matching_strategy_ == MatchingStrategyType::advanced ? reference.reference_time
                                                              : incoming_cloud_info.cloud_timestamp;
}

template <typename PointCloudMsgT>
bool CloudConcatenator<PointCloudMsgT>::is_late(double frame)
{
  if (!late_cloud_policy_.reject_late_clouds || !emitted_frame_watermark_.has_value()) {
    return false;
  }
  const double behind = *emitted_frame_watermark_ - frame;
  if (behind < 0.0) return false;  // a newer frame
  if (behind > late_cloud_policy_.replay_jump_sec) {
    emitted_frame_watermark_.reset();  // a replay restart, not a straggler
    return false;
  }
  return true;
}

template <typename PointCloudMsgT>
std::vector<ConcatenatedFrame<PointCloudMsgT>> CloudConcatenator<PointCloudMsgT>::flush()
{
  std::sort(collectors_.begin(), collectors_.end(), [](const Collector & a, const Collector & b) {
    return a.creation_arrival < b.creation_arrival;
  });

  std::vector<ConcatenatedFrame<PointCloudMsgT>> outputs;
  outputs.reserve(collectors_.size());
  for (auto & collector : collectors_) {
    outputs.push_back(emit(std::move(collector), ConcatenatedFrameStatus::kTimeout));
  }
  collectors_.clear();
  return outputs;
}

template <typename PointCloudMsgT>
std::optional<double> CloudConcatenator<PointCloudMsgT>::next_deadline() const
{
  std::optional<double> earliest_creation;
  for (const auto & collector : collectors_) {
    if (!earliest_creation.has_value() || collector.creation_arrival < *earliest_creation) {
      earliest_creation = collector.creation_arrival;
    }
  }
  if (!earliest_creation.has_value()) return std::nullopt;
  return *earliest_creation + timeout_sec_;
}

template <typename PointCloudMsgT>
void CloudConcatenator<PointCloudMsgT>::drop_oldest_collector()
{
  if (collectors_.empty()) return;
  const auto oldest = std::min_element(
    collectors_.begin(), collectors_.end(),
    [](const Collector & a, const Collector & b) { return a.reference_time < b.reference_time; });
  collectors_.erase(oldest);
  ++stats_.force_dropped_collectors;
}

template <typename PointCloudMsgT>
ConcatenatedFrame<PointCloudMsgT> CloudConcatenator<PointCloudMsgT>::emit(
  Collector && collector, ConcatenatedFrameStatus status)
{
  // Only the advanced strategy records a reference window in the info message; the naive info
  // carries the first arrival (as in the node).
  std::shared_ptr<CollectorInfoBase> collector_info;
  if (matching_strategy_ == MatchingStrategyType::advanced) {
    collector_info =
      std::make_shared<AdvancedCollectorInfo>(collector.reference_time, collector.noise_window);
  } else {
    collector_info = std::make_shared<NaiveCollectorInfo>(collector.creation_arrival);
  }

  double emitted_frame = collector.reference_time;
  if (matching_strategy_ != MatchingStrategyType::advanced) {
    emitted_frame = -std::numeric_limits<double>::infinity();
    for (const auto & entry : collector.topic_to_cloud) {
      emitted_frame = std::max(emitted_frame, to_seconds(entry.second->header.stamp));
    }
  }
  if (!emitted_frame_watermark_.has_value() || emitted_frame > *emitted_frame_watermark_) {
    emitted_frame_watermark_ = emitted_frame;
  }

  ConcatenatedFrame<PointCloudMsgT> frame{
    status, handler_.combine_pointclouds(collector.topic_to_cloud, collector_info),
    collector.reference_time, collector.noise_window, collector.creation_arrival};
  return frame;
}

}  // namespace autoware::pointcloud_preprocessor
