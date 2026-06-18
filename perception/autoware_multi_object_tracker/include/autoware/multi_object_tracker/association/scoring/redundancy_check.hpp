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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__SCORING__REDUNDANCY_CHECK_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__SCORING__REDUNDANCY_CHECK_HPP_

#include "autoware/multi_object_tracker/configurations.hpp"
#include "autoware/multi_object_tracker/types.hpp"

namespace autoware::multi_object_tracker
{

/// Returns true when source and target trackers are spatially redundant and should be merged.
/// Handles four cases: pedestrian/pedestrian, known/known, known/unknown, unknown/unknown.
bool isRedundant(
  const types::DynamicObject & source_object, const types::DynamicObject & target_object,
  const classes::Label source_label, const classes::Label target_label, float source_known_prob,
  float target_known_prob, const TrackerOverlapManagerConfig & config);

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__SCORING__REDUNDANCY_CHECK_HPP_
