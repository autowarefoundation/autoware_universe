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

#ifndef PREDICTED_OBJECTS_LATCHER_HPP_
#define PREDICTED_OBJECTS_LATCHER_HPP_

#include "type_alias.hpp"

namespace autoware::in_lane_mrm_planner
{

class PredictedObjectsLatcher
{
public:
  void latch(const PredictedObjects & live_objects, const bool use_latch);

  void unlatch();

  bool is_latched() const;

  const PredictedObjects & objects_for_planning(const PredictedObjects & live_objects) const;

private:
  bool latched_{false};
  PredictedObjects latched_objects_;
};

}  // namespace autoware::in_lane_mrm_planner

#endif  // PREDICTED_OBJECTS_LATCHER_HPP_
