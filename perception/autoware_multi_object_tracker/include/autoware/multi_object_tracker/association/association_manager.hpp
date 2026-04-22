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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_MANAGER_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_MANAGER_HPP_

#include "autoware/multi_object_tracker/association/association_base.hpp"
#include "autoware/multi_object_tracker/association/bev_association.hpp"
#include "autoware/multi_object_tracker/association/polar_association.hpp"
#include "autoware/multi_object_tracker/configurations.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <autoware_utils_debug/time_keeper.hpp>

#include <list>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::multi_object_tracker
{

/// Routes each measurement batch to the D2T association implementation designated per input
/// channel (selected via InputChannel::associator_type).
/// Available algorithms:
///   BEV   -> BevAssociation    (bird's-eye-view area scoring + GNN assignment)
///   POLAR -> PolarAssociation  (polar-coordinate range-bearing scoring)
class AssociationManager
{
public:
  AssociationManager(
    const AssociatorConfig & bev_config, const std::vector<types::InputChannel> & channels_config);

  /// Match measurements to trackers using the channel's designated association.
  types::AssociationResult associate(
    const types::DynamicObjectList & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers);

  void setTimeKeeper(std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr);

private:
  /// Select the D2T association implementation for the given channel index.
  AssociationBase & getAssociationForChannel(uint channel_index) const;

  std::vector<types::InputChannel> channels_config_;
  std::unique_ptr<BevAssociation> bev_association_;
  std::unique_ptr<PolarAssociation> polar_association_;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_MANAGER_HPP_
