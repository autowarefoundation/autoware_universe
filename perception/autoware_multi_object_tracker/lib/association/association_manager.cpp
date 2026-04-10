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

#include <utility>

namespace autoware::multi_object_tracker
{

AssociationManager::AssociationManager(
  const AssociatorConfig & online_config, const OverlapMergerConfig & overlap_config)
: online_association_(std::make_unique<DataAssociation>(online_config)),
  overlap_merger_(std::make_unique<OverlapMerger>(overlap_config)),
  sensor_association_(std::make_unique<SensorPerspectiveAssociation>())
{
}

types::AssociationResult AssociationManager::associate(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  const types::AssociationData association_data =
    online_association_->calcAssociationData(measurements, trackers);

  types::AssociationResult result;
  online_association_->assign(association_data, result);

  // Future hook: sensor-perspective refinement (currently no-op)
  sensor_association_->refine(result, measurements, trackers);

  return result;
}

void AssociationManager::pruneOverlaps(
  std::list<std::shared_ptr<Tracker>> & trackers, const rclcpp::Time & time,
  const AdaptiveThresholdCache & threshold_cache,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose)
{
  overlap_merger_->merge(trackers, time, threshold_cache, ego_pose);
}

void AssociationManager::setTimeKeeper(
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr)
{
  online_association_->setTimeKeeper(std::move(time_keeper_ptr));
}

}  // namespace autoware::multi_object_tracker
