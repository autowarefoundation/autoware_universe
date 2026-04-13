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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__TRACKER_OVERLAP_MANAGER_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__TRACKER_OVERLAP_MANAGER_HPP_

#include "autoware/multi_object_tracker/association/adaptive_threshold_cache.hpp"
#include "autoware/multi_object_tracker/configurations.hpp"
#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <list>
#include <memory>
#include <optional>

namespace autoware::multi_object_tracker
{

/// Detects and merges spatially overlapping trackers produced by different sensors or detectors.
/// Higher-priority trackers absorb lower-priority ones when sufficient overlap is confirmed.
class TrackerOverlapManager
{
public:
  explicit TrackerOverlapManager(const TrackerOverlapManagerConfig & config);

  /// Scans tracker_list for overlapping pairs and merges the lower-priority one into the higher.
  void merge(
    std::list<std::shared_ptr<Tracker>> & tracker_list, const rclcpp::Time & time,
    const AdaptiveThresholdCache & threshold_cache,
    const std::optional<geometry_msgs::msg::Pose> & ego_pose);

private:
  TrackerOverlapManagerConfig config_;

  /// Returns true if target may be merged (absorbed) into other.
  bool canMergeTarget(
    const Tracker & target, const Tracker & other, const rclcpp::Time & time,
    const AdaptiveThresholdCache & threshold_cache,
    const std::optional<geometry_msgs::msg::Pose> & ego_pose) const;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__TRACKER_OVERLAP_MANAGER_HPP_
