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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_BASE_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_BASE_HPP_

#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <list>
#include <memory>

namespace autoware::multi_object_tracker
{

/// Abstract interface for measurement-to-tracker association algorithms.
/// Each concrete implementation provides an independent association strategy
/// that can be assigned per input channel via InputChannel::associator_type.
class AssociationBase
{
public:
  virtual ~AssociationBase() = default;

  /// Perform association between measurements and the current tracker list.
  /// Returns a full AssociationResult mapping trackers <-> measurements.
  virtual types::AssociationResult associate(
    const types::DynamicObjectList & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers) = 0;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_BASE_HPP_
