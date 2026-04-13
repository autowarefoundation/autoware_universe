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

#include "autoware/multi_object_tracker/association/association_manager.hpp"

#include <list>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{

AssociationManager::AssociationManager(
  const AssociatorConfig & bev_area_config, const TrackerMergerConfig & tracker_merger_config,
  const std::vector<types::InputChannel> & channels_config)
: channels_config_(channels_config),
  bev_area_association_(std::make_unique<BevAreaAssociation>(bev_area_config)),
  sensor_perspective_association_(std::make_unique<SensorPerspectiveAssociation>()),
  tracker_merger_(std::make_unique<TrackerMerger>(tracker_merger_config))
{
}

AssociationBase & AssociationManager::getAssociationForChannel(const uint channel_index) const
{
  if (channel_index < channels_config_.size()) {
    if (
      channels_config_[channel_index].associator_type ==
      types::AssociationType::SENSOR_PERSPECTIVE) {
      return *sensor_perspective_association_;
    }
  }
  return *bev_area_association_;
}

types::AssociationResult AssociationManager::associate(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  return getAssociationForChannel(measurements.channel_index).associate(measurements, trackers);
}

void AssociationManager::mergeTrackers(
  std::list<std::shared_ptr<Tracker>> & trackers, const rclcpp::Time & time,
  const AdaptiveThresholdCache & threshold_cache,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose)
{
  tracker_merger_->merge(trackers, time, threshold_cache, ego_pose);
}

void AssociationManager::setTimeKeeper(
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr)
{
  bev_area_association_->setTimeKeeper(std::move(time_keeper_ptr));
}

}  // namespace autoware::multi_object_tracker
