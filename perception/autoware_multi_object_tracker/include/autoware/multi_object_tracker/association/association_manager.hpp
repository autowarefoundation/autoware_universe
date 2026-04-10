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

#include "autoware/multi_object_tracker/association/association.hpp"
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

/// Orchestrates all association algorithms.
///
/// Responsibilities:
///   1. Route each measurement batch to the associator designated for its input channel
///      (selected via InputChannel::associator_type).
///   2. Prune spatially redundant trackers via OverlapMerger.
///
/// Available associators:
///   - ONLINE            → DataAssociation (score matrix + GNN linear assignment)
///   - SENSOR_PERSPECTIVE → SensorPerspectiveAssociation (sensor-geometry-based matching)
class AssociationManager
{
public:
  AssociationManager(
    const AssociatorConfig & online_config, const OverlapMergerConfig & overlap_config,
    const std::vector<types::InputChannel> & channels_config);

  /// Match measurements to trackers using the associator designated for the channel.
  types::AssociationResult associate(
    const types::DynamicObjectList & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers);

  /// Remove spatially overlapping / redundant trackers.
  void pruneOverlaps(
    std::list<std::shared_ptr<Tracker>> & trackers, const rclcpp::Time & time,
    const AdaptiveThresholdCache & threshold_cache,
    const std::optional<geometry_msgs::msg::Pose> & ego_pose);

  void setTimeKeeper(std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr);

private:
  /// Select the associator for the given channel index.
  IAssociation & getAssociatorForChannel(uint channel_index) const;

  std::vector<types::InputChannel> channels_config_;
  std::unique_ptr<DataAssociation> online_association_;
  std::unique_ptr<SensorPerspectiveAssociation> sensor_association_;
  std::unique_ptr<OverlapMerger> overlap_merger_;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_MANAGER_HPP_
