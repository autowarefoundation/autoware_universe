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

#include "autoware/multi_object_tracker/association/uuid_association.hpp"

#include <algorithm>
#include <list>
#include <memory>

namespace autoware::multi_object_tracker
{

types::AssociationResult UUIDAssociation::associate(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  types::AssociationResult result;

  // All trackers start as unassigned
  for (const auto & tracker : trackers) {
    result.unassigned_trackers.push_back(tracker->getUUID());
  }

  // Match each measurement to a tracker with the same UUID
  const types::UUIDEqual uuid_equal;
  for (const auto & measurement : measurements.objects) {
    auto it = std::find_if(
      result.unassigned_trackers.begin(), result.unassigned_trackers.end(),
      [&](const auto & tracker_uuid) { return uuid_equal(tracker_uuid, measurement.uuid); });

    if (it != result.unassigned_trackers.end()) {
      result.add(*it, measurement.uuid);
      result.unassigned_trackers.erase(it);
    } else {
      result.unassigned_measurements.push_back(measurement.uuid);
    }
  }

  return result;
}

}  // namespace autoware::multi_object_tracker
