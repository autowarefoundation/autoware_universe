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

#ifndef TRIGGER_EDGE_DETECTOR_HPP_
#define TRIGGER_EDGE_DETECTOR_HPP_

namespace autoware::in_lane_mrm_planner
{

class TriggerEdgeDetector
{
public:
  struct Edges
  {
    bool rising{false};
    bool falling{false};
  };

  Edges update(const bool trigger_active)
  {
    Edges edges;
    if (trigger_active && !prev_active_) {
      edges.rising = true;
    }
    if (!trigger_active && prev_active_) {
      edges.falling = true;
    }
    prev_active_ = trigger_active;
    return edges;
  }

private:
  bool prev_active_{false};
};

}  // namespace autoware::in_lane_mrm_planner

#endif  // TRIGGER_EDGE_DETECTOR_HPP_
