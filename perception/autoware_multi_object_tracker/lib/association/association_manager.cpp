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

#include "autoware/multi_object_tracker/association/association_manager.hpp"

#include <list>
#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{

AssociationManager::AssociationManager(
  const AssociatorConfig & bev_config, const std::vector<types::InputChannel> & channels_config)
: channels_config_(channels_config),
  bev_association_(std::make_unique<BevAssociation>(bev_config)),
  polar_association_(std::make_unique<PolarAssociation>()),
  uuid_association_(std::make_unique<UUIDAssociation>())
{
}

AssociationBase & AssociationManager::getAssociationForChannel(const uint channel_index) const
{
  if (channel_index < channels_config_.size()) {
    if (channels_config_[channel_index].associator_type == types::AssociationType::POLAR) {
      return *polar_association_;
    }
  }
  return *bev_association_;
}

types::AssociationResult AssociationManager::associate(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  const auto channel_index = measurements.channel_index;

  // For DETECTED_OBJECTS channels: single-stage geometric association
  if (
    channel_index >= channels_config_.size() ||
    channels_config_[channel_index].input_type != types::InputType::TRACKED_OBJECTS) {
    return getAssociationForChannel(channel_index).associate(measurements, trackers);
  }

  // For TRACKED_OBJECTS channels: two-stage association
  // Stage 1 — UUID identity matching
  auto result = uuid_association_->associate(measurements, trackers);

  if (result.unassigned_measurements.empty() || result.unassigned_trackers.empty()) {
    return result;
  }

  // Stage 2 — geometric fallback for unmatched pairs
  // Build unmatched measurement list using the existing UUID index
  types::DynamicObjectList unmatched_measurements;
  unmatched_measurements.header = measurements.header;
  unmatched_measurements.channel_index = measurements.channel_index;
  for (const auto & meas_uuid : result.unassigned_measurements) {
    const auto idx = measurements.getObjectIndexByUuid(meas_uuid);
    if (idx) {
      unmatched_measurements.objects.push_back(measurements.objects[*idx]);
    }
  }

  // Build unmatched tracker list using a hash set for O(n + m) filtering
  std::unordered_set<
    unique_identifier_msgs::msg::UUID, types::UUIDHash, types::UUIDEqual>
    unassigned_tracker_set(
      result.unassigned_trackers.begin(), result.unassigned_trackers.end());
  std::list<std::shared_ptr<Tracker>> unmatched_trackers;
  for (const auto & tracker : trackers) {
    if (unassigned_tracker_set.count(tracker->getUUID())) {
      unmatched_trackers.push_back(tracker);
    }
  }

  // Run geometric association on the unmatched subsets
  const auto fallback_result =
    getAssociationForChannel(channel_index).associate(unmatched_measurements, unmatched_trackers);

  // Merge fallback into the main result
  for (const auto & [tracker_uuid, meas_uuid] : fallback_result.tracker_to_measurement) {
    result.add(tracker_uuid, meas_uuid);
  }
  for (const auto & uuid : fallback_result.trackers_with_shape_change) {
    result.trackers_with_shape_change.insert(uuid);
  }
  result.unassigned_measurements = fallback_result.unassigned_measurements;
  result.unassigned_trackers = fallback_result.unassigned_trackers;

  return result;
}

void AssociationManager::setTimeKeeper(
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr)
{
  bev_association_->setTimeKeeper(std::move(time_keeper_ptr));
}

}  // namespace autoware::multi_object_tracker
