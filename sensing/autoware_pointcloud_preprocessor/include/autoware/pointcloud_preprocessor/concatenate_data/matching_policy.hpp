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

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

// Information about an incoming cloud that the strategy matches on.
struct IncomingCloudInfo
{
  std::string topic_name;
  double cloud_timestamp;     // message header stamp, in seconds
  double cloud_arrival_time;  // arrival time, in seconds
};

// Candidate collectors needed for matching.
struct CandidateCollectorState
{
  double reference_time;  // the collector's reference time (Naive/Advanced CollectorInfo timestamp)
  double noise_window;    // the collector's noise window (advanced only; 0 for naive)
  bool has_topic;         // whether the collector already holds IncomingCloudInfo.topic_name
};

// The reference a freshly-created collector should be given for an incoming cloud.
struct CollectorReference
{
  double reference_time;
  double noise_window;
};

class MatchingPolicy
{
public:
  virtual ~MatchingPolicy() = default;

  // Index into `collectors` of the collector this cloud should join, or nullopt to start a new one.
  [[nodiscard]] virtual std::optional<std::size_t> match(
    const std::vector<CandidateCollectorState> & collectors,
    const IncomingCloudInfo & incoming_cloud_info) const = 0;

  // Reference (timestamp + noise window) for a new collector created for this cloud.
  [[nodiscard]] virtual CollectorReference reference_for(
    const IncomingCloudInfo & incoming_cloud_info) const = 0;
};

// Match the closest collector (by arrival time) that does not yet hold the topic.
class NaiveMatchingPolicy : public MatchingPolicy
{
public:
  [[nodiscard]] std::optional<std::size_t> match(
    const std::vector<CandidateCollectorState> & collectors,
    const IncomingCloudInfo & incoming_cloud_info) const override;
  [[nodiscard]] CollectorReference reference_for(
    const IncomingCloudInfo & incoming_cloud_info) const override;
};

// Match by an offset-corrected timestamp falling inside a collector's noise window.
class AdvancedMatchingPolicy : public MatchingPolicy
{
public:
  AdvancedMatchingPolicy(
    const std::vector<std::string> & input_topics,
    const std::vector<double> & lidar_timestamp_offsets,
    const std::vector<double> & lidar_timestamp_noise_window);

  [[nodiscard]] std::optional<std::size_t> match(
    const std::vector<CandidateCollectorState> & collectors,
    const IncomingCloudInfo & incoming_cloud_info) const override;
  [[nodiscard]] CollectorReference reference_for(
    const IncomingCloudInfo & incoming_cloud_info) const override;

private:
  std::unordered_map<std::string, double> topic_to_offset_map_;
  std::unordered_map<std::string, double> topic_to_noise_window_map_;
};

}  // namespace autoware::pointcloud_preprocessor
