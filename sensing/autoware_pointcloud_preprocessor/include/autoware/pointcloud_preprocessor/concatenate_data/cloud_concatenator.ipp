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
#include "autoware/pointcloud_preprocessor/concatenate_data/conversion.hpp"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

template <typename PointCloudMsgT>
const std::vector<std::string> & CloudConcatenator<PointCloudMsgT>::validate_input_topics(
  const std::vector<std::string> & input_topics)
{
  std::unordered_set<std::string> seen;
  seen.reserve(input_topics.size());
  for (const auto & topic : input_topics) {
    if (!seen.insert(topic).second) {
      throw std::invalid_argument(
        "duplicate input topic '" + topic +
        "'; input_topics must be unique, otherwise a collector can never gather one cloud per "
        "topic and every frame is emitted on timeout");
    }
  }
  return input_topics;
}

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
  std::size_t max_open_collectors)
: input_topics_(validate_input_topics(input_topics)),
  timeout_sec_(timeout_sec),
  matching_strategy_(matching_strategy),
  max_open_collectors_(max_open_collectors),
  matching_policy_(make_matching_policy(
    input_topics, matching_strategy, lidar_timestamp_offsets, lidar_timestamp_noise_window)),
  handler_(
    input_topics, output_frame, is_motion_compensated, publish_synchronized_pointcloud,
    keep_input_frame_in_synchronized_pointcloud, matching_strategy)
{
}

template <typename PointCloudMsgT>
void CloudConcatenator<PointCloudMsgT>::set_transform(
  const geometry_msgs::msg::TransformStamped & sensor_to_output_frame)
{
  handler_.set_transform(sensor_to_output_frame);
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

  const IncomingCloudInfo incoming_cloud_info{
    topic, utils::to_seconds(cloud->header.stamp), arrival_time};

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
  // timeout starting at this cloud's arrival). When the collector limit is reached, make room first
  // by discarding the collector with the oldest reference (mirrors the node's fixed collector
  // pool).
  std::size_t index = 0;
  if (matched_index.has_value()) {
    index = *matched_index;
  } else {
    if (max_open_collectors_ != 0 && collectors_.size() >= max_open_collectors_) {
      drop_oldest_collector();
    }
    const MatchingReference reference = matching_policy_->reference_for(incoming_cloud_info);
    collectors_.push_back(
      Collector{{}, reference.reference_time, reference.noise_window, arrival_time});
    index = collectors_.size() - 1;
  }

  // (4) Add the cloud; emit immediately if the collector is now complete.
  if (collectors_[index].topic_to_cloud.count(topic) > 0) {
    // Usually a symptom of a mis-tuned noise window; the newer cloud wins, as in the node.
    ++stats_.duplicate_topic_inserts;
    stats_.last_duplicate_topic = topic;
  }
  collectors_[index].topic_to_cloud[topic] = std::move(cloud);
  if (collectors_[index].topic_to_cloud.size() == input_topics_.size()) {
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
  // A collector expires once timeout_sec has elapsed since its creation (>=, so a timer armed
  // exactly at the deadline closes it).
  std::vector<Collector> stale;
  std::vector<Collector> open;
  for (auto & collector : collectors_) {
    if (collector.creation_arrival <= now_sec - timeout_sec_) {
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
  return outputs;
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

  ConcatenatedFrame<PointCloudMsgT> frame{
    status, handler_.combine_pointclouds(collector.topic_to_cloud, collector_info),
    collector.reference_time, collector.noise_window, collector.creation_arrival};
  return frame;
}

}  // namespace autoware::pointcloud_preprocessor
