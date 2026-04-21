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
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{

AssociationManager::AssociationManager(
  const AssociatorConfig & bev_config, const std::vector<types::InputChannel> & channels_config)
: channels_config_(channels_config),
  bev_association_(std::make_unique<BevAssociation>(bev_config)),
  polar_association_(std::make_unique<PolarAssociation>())
{
}

AssociationBase & AssociationManager::getAssociationForChannel(const uint channel_index) const
{
  if (channels_config_[channel_index].associator_type == types::AssociationType::POLAR) {
    return *polar_association_;
  }
  return *bev_association_;
}

types::AssociationResult AssociationManager::associate(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  return getAssociationForChannel(measurements.channel_index).associate(measurements, trackers);
}

void AssociationManager::setTimeKeeper(
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr)
{
  bev_association_->setTimeKeeper(std::move(time_keeper_ptr));
}

}  // namespace autoware::multi_object_tracker
