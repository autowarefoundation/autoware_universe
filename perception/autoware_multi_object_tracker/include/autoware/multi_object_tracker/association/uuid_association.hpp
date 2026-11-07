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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__UUID_ASSOCIATION_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__UUID_ASSOCIATION_HPP_

#include "autoware/multi_object_tracker/association/association_base.hpp"

#include <list>
#include <memory>

namespace autoware::multi_object_tracker
{

/// Association by direct UUID matching.
/// Used when the input channel carries TrackedObjects: each measurement's UUID
/// is compared against all active tracker UUIDs and matched on equality.
/// Unmatched measurements are passed to the spawn path; unmatched trackers
/// are left to age out normally.
class UUIDAssociation : public AssociationBase
{
public:
  UUIDAssociation() = default;

  types::AssociationResult associate(
    const types::DynamicObjectList & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers) override;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__UUID_ASSOCIATION_HPP_
