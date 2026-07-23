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

#include "autoware/pointcloud_preprocessor/concatenate_data/matching_policy.hpp"

#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

std::optional<std::size_t> NaiveMatchingPolicy::match(
  const std::vector<CandidateCollectorState> & collectors,
  const IncomingCloudInfo & incoming_cloud_info) const
{
  std::optional<double> smallest_time_difference;
  std::optional<std::size_t> closest_collector;

  for (std::size_t i = 0; i < collectors.size(); ++i) {
    if (collectors[i].has_topic) continue;
    const double time_difference =
      std::abs(incoming_cloud_info.cloud_arrival_time - collectors[i].reference_time);
    if (!smallest_time_difference || time_difference < *smallest_time_difference) {
      smallest_time_difference = time_difference;
      closest_collector = i;
    }
  }

  return closest_collector;
}

CollectorReference NaiveMatchingPolicy::reference_for(
  const IncomingCloudInfo & incoming_cloud_info) const
{
  return CollectorReference{incoming_cloud_info.cloud_arrival_time, 0.0};
}

AdvancedMatchingPolicy::AdvancedMatchingPolicy(
  const std::vector<std::string> & input_topics,
  const std::vector<double> & lidar_timestamp_offsets,
  const std::vector<double> & lidar_timestamp_noise_window)
{
  if (lidar_timestamp_offsets.size() != input_topics.size()) {
    throw std::runtime_error(
      "The number of topics does not match the number of timestamp offsets.");
  }
  if (lidar_timestamp_noise_window.size() != input_topics.size()) {
    throw std::runtime_error(
      "The number of topics does not match the number of timestamp noise window.");
  }

  for (std::size_t i = 0; i < input_topics.size(); ++i) {
    topic_to_offset_map_[input_topics[i]] = lidar_timestamp_offsets[i];
    topic_to_noise_window_map_[input_topics[i]] = lidar_timestamp_noise_window[i];
  }
}

std::optional<std::size_t> AdvancedMatchingPolicy::match(
  const std::vector<CandidateCollectorState> & collectors,
  const IncomingCloudInfo & incoming_cloud_info) const
{
  const double time =
    incoming_cloud_info.cloud_timestamp - topic_to_offset_map_.at(incoming_cloud_info.topic_name);
  const double topic_noise_window = topic_to_noise_window_map_.at(incoming_cloud_info.topic_name);

  for (std::size_t i = 0; i < collectors.size(); ++i) {
    const double reference_timestamp_min =
      collectors[i].reference_time - collectors[i].noise_window;
    const double reference_timestamp_max =
      collectors[i].reference_time + collectors[i].noise_window;
    if (
      time < reference_timestamp_max + topic_noise_window &&
      time > reference_timestamp_min - topic_noise_window) {
      return i;
    }
  }
  return std::nullopt;
}

CollectorReference AdvancedMatchingPolicy::reference_for(
  const IncomingCloudInfo & incoming_cloud_info) const
{
  return CollectorReference{
    incoming_cloud_info.cloud_timestamp - topic_to_offset_map_.at(incoming_cloud_info.topic_name),
    topic_to_noise_window_map_.at(incoming_cloud_info.topic_name)};
}

}  // namespace autoware::pointcloud_preprocessor
