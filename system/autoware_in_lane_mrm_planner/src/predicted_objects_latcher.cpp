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

#include "predicted_objects_latcher.hpp"

namespace autoware::in_lane_mrm_planner
{

void PredictedObjectsLatcher::latch(const PredictedObjects & live_objects, const bool use_latch)
{
  if (!use_latch) {
    return;
  }
  latched_objects_ = live_objects;
  latched_ = true;
}

void PredictedObjectsLatcher::unlatch()
{
  latched_ = false;
}

bool PredictedObjectsLatcher::is_latched() const
{
  return latched_;
}

const PredictedObjects & PredictedObjectsLatcher::objects_for_planning(
  const PredictedObjects & live_objects) const
{
  return latched_ ? latched_objects_ : live_objects;
}

}  // namespace autoware::in_lane_mrm_planner
