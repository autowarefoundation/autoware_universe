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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__MERGER__DETAIL__SURVIVAL_RANKING_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__MERGER__DETAIL__SURVIVAL_RANKING_HPP_

#include "autoware/multi_object_tracker/merger/detail/overlap_types.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <rclcpp/rclcpp.hpp>

namespace autoware::multi_object_tracker::detail
{

// Lazily evaluate and cache the tracker's confidence for this cycle.
bool ensureConfident(TrackerSnapshot & snap, const DecisionContext & ctx);

// Lazily export and cache the tracker's full object; nullptr if it cannot be exported.
const types::DynamicObject * ensureObject(TrackerSnapshot & snap, const rclcpp::Time & time);

// Decides which of a pair survives; a total order, always strict.
// Returns +1 (a survives), -1 (b survives).
int compareForSurvival(TrackerSnapshot & a, TrackerSnapshot & b, const DecisionContext & ctx);

}  // namespace autoware::multi_object_tracker::detail

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__MERGER__DETAIL__SURVIVAL_RANKING_HPP_
