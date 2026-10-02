// Copyright 2024 TIER IV, Inc.
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

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

namespace autoware::pointcloud_preprocessor
{

enum class CollectorStatus { Idle, Processing, Finished };

/// What add() changed about the collector. Each field reports a fact, not an instruction: how to
/// react is the caller's choice, which is what lets a node and an offline pipeline share this core.
struct CollectorAddResult
{
  /// The collector was empty and has begun a new group, so its timeout period starts now.
  bool started{false};
  /// This topic already had a cloud in the group, and that older cloud was discarded.
  bool duplicate_topic{false};
  /// The group now holds a cloud for every input topic, so nothing is left to wait for.
  bool ready_to_concatenate{false};
};

/// One group of source clouds on its way to becoming a single concatenated cloud: one slot per
/// input topic, filled as clouds arrive, done once every slot is taken or the timeout expires.
template <typename PointCloudMsgT>
class CloudCollectorCore
{
public:
  using CloudConstPtr = typename PointCloudMsgT::ConstSharedPtr;
  using CloudMap = std::unordered_map<std::string, CloudConstPtr>;

  CloudCollectorCore(std::size_t num_of_clouds, double timeout_sec)
  : num_of_clouds_(num_of_clouds), timeout_sec_(timeout_sec)
  {
  }

  /// Add one cloud. Pass @p arrival_time to let the collector answer is_timed_out() itself;
  /// leave it empty when an external timer owns the timeout.
  CollectorAddResult add(
    const std::string & topic, CloudConstPtr cloud,
    std::optional<double> arrival_time = std::nullopt)
  {
    CollectorAddResult result;

    if (status_ == CollectorStatus::Idle) {
      status_ = CollectorStatus::Processing;
      result.started = true;
      started_at_ = arrival_time;
    } else if (status_ == CollectorStatus::Processing) {
      result.duplicate_topic = has_topic(topic);
    }

    topic_to_cloud_map_[topic] = std::move(cloud);
    result.ready_to_concatenate = topic_to_cloud_map_.size() == num_of_clouds_;
    return result;
  }

  [[nodiscard]] bool has_topic(const std::string & topic) const
  {
    return topic_to_cloud_map_.find(topic) != topic_to_cloud_map_.end();
  }

  /// False unless add() was given an arrival_time, in which case the caller owns the timeout.
  [[nodiscard]] bool is_timed_out(double now_sec) const
  {
    if (status_ != CollectorStatus::Processing || !started_at_) return false;
    return now_sec - *started_at_ >= timeout_sec_;
  }

  [[nodiscard]] CollectorStatus status() const { return status_; }

  /// Mark the group as concatenated. Separate from add() because the caller, not the core,
  /// decides when the concatenation actually happened.
  void mark_finished() { status_ = CollectorStatus::Finished; }

  void reset()
  {
    status_ = CollectorStatus::Idle;
    topic_to_cloud_map_.clear();
    collector_info_ = nullptr;
    started_at_.reset();
  }

  void set_info(std::shared_ptr<CollectorInfoBase> collector_info)
  {
    collector_info_ = std::move(collector_info);
  }
  [[nodiscard]] std::shared_ptr<CollectorInfoBase> get_info() const { return collector_info_; }

  [[nodiscard]] const CloudMap & topic_to_cloud_map() const { return topic_to_cloud_map_; }
  [[nodiscard]] std::size_t num_of_clouds() const { return num_of_clouds_; }
  [[nodiscard]] double timeout_sec() const { return timeout_sec_; }

private:
  std::size_t num_of_clouds_;
  double timeout_sec_;
  CloudMap topic_to_cloud_map_;
  std::shared_ptr<CollectorInfoBase> collector_info_;
  CollectorStatus status_{CollectorStatus::Idle};
  std::optional<double> started_at_;
};

}  // namespace autoware::pointcloud_preprocessor
