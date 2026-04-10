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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_MANAGER_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_MANAGER_HPP_

#include "autoware/multi_object_tracker/association/bev_area_association.hpp"
#include "autoware/multi_object_tracker/association/i_association.hpp"
#include "autoware/multi_object_tracker/association/overlap_merger.hpp"
#include "autoware/multi_object_tracker/association/sensor_perspective.hpp"
#include "autoware/multi_object_tracker/configurations.hpp"
#include "autoware/multi_object_tracker/object_model/types.hpp"

#include <autoware_utils_debug/time_keeper.hpp>

#include <list>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::multi_object_tracker
{

/// Orchestrates two layers of association:
///
///   Layer 1 — Detection-to-tracker (D2T):
///     Routes each measurement batch to the association implementation designated per input channel
///     (selected via InputChannel::associator_type).
///     Available algorithms:
///       BEV               → BevAreaAssociation  (bird's-eye-view area scoring + GNN assignment)
///       SENSOR_PERSPECTIVE → SensorPerspectiveAssociation (sensor-perspective area scoring)
///
///   Layer 2 — Tracker-to-tracker (T2T):
///     TrackerMerger removes spatially redundant trackers after D2T association.
class AssociationManager
{
public:
  AssociationManager(
    const AssociatorConfig & bev_area_config, const TrackerMergerConfig & tracker_merger_config,
    const std::vector<types::InputChannel> & channels_config);

  /// Layer 1 (D2T): match measurements to trackers using the channel's designated association.
  types::AssociationResult associate(
    const types::DynamicObjectList & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers);

  /// Layer 2 (T2T): remove spatially overlapping / redundant trackers.
  void mergeTrackers(
    std::list<std::shared_ptr<Tracker>> & trackers, const rclcpp::Time & time,
    const AdaptiveThresholdCache & threshold_cache,
    const std::optional<geometry_msgs::msg::Pose> & ego_pose);

  void setTimeKeeper(std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr);

private:
  /// Select the D2T association implementation for the given channel index.
  IAssociation & getAssociationForChannel(uint channel_index) const;

  std::vector<types::InputChannel> channels_config_;
  std::unique_ptr<BevAreaAssociation> bev_area_association_;
  std::unique_ptr<SensorPerspectiveAssociation> sensor_perspective_association_;
  std::unique_ptr<TrackerMerger> tracker_merger_;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_MANAGER_HPP_
