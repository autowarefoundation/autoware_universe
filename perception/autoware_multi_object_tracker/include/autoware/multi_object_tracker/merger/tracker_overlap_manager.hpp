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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__MERGER__TRACKER_OVERLAP_MANAGER_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__MERGER__TRACKER_OVERLAP_MANAGER_HPP_

#include "autoware/multi_object_tracker/association/adaptive_threshold_cache.hpp"
#include "autoware/multi_object_tracker/configurations.hpp"
#include "autoware/multi_object_tracker/tracker/trackers/tracker_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <list>
#include <memory>
#include <optional>

namespace autoware::multi_object_tracker
{

/// Detects and merges spatially overlapping trackers produced by different sensors or detectors
/// (including over-segmented detections that one-to-one association cannot fuse).
///
/// Guarantees:
/// - The merge outcome is a pure function of the tracker states: pair direction comes from a
///   lexicographic dominance comparison ending in the tracker UUID, so results do not depend on
///   tracker_list order.
/// - The winner of a merge must be confident and carry a parametric shape (bounding box or
///   cylinder); polygon-only trackers may only be absorbed — two polygon-only trackers never
///   merge.
/// - Applied merges form a star forest per cycle (no tracker both absorbs and is absorbed);
///   longer chains converge over subsequent cycles with direct geometry checks.
class TrackerOverlapManager
{
public:
  explicit TrackerOverlapManager(const TrackerOverlapManagerConfig & config);

  /// Scans tracker_list for overlapping pairs and merges each redundant tracker into its
  /// dominant counterpart.
  void merge(
    std::list<std::shared_ptr<Tracker>> & tracker_list, const rclcpp::Time & time,
    const AdaptiveThresholdCache & threshold_cache,
    const std::optional<geometry_msgs::msg::Pose> & ego_pose);

private:
  TrackerOverlapManagerConfig config_;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__MERGER__TRACKER_OVERLAP_MANAGER_HPP_
