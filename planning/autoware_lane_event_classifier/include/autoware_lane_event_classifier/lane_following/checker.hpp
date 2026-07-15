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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_FOLLOWING__CHECKER_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_FOLLOWING__CHECKER_HPP_

#include <string_view>

namespace autoware::lane_event_classifier
{

/** @brief Which rule decided the lane-following outcome (for tracing / logging). */
enum class LaneFollowingReason {
  no_reference_lane,          // no reference lane yet -> treated as following
  inside_connected_sequence,  // reference point inside a lane of the connected sequence
  within_lateral_tolerance,   // reference point within the lateral margin of the sequence
  road_shoulder_exempt,       // reference point overlaps a road shoulder
  turn_lane_exempt,           // reference_lane / current lane is a turn / intersection lane
  virtual_boundary_exempt,    // the boundary the ego crossed is virtual
  departed                    // none matched -> a real lateral departure (not following)
};

/** @brief Lane-following verdict plus the rule that decided it. */
struct LaneFollowingResult
{
  bool is_following{true};
  LaneFollowingReason reason{LaneFollowingReason::no_reference_lane};
};

/** @brief Returns a short label for the reason (tracing / logging). */
[[nodiscard]] std::string_view to_string(LaneFollowingReason reason);

/**
 * @brief Evaluates the lane-following check and reports which rule decided the outcome.
 *
 * @note Stub: the classification logic (and its parameters) are added in a follow-up PR. Every
 * call currently reports the ego as following (LaneFollowingReason::no_reference_lane), so the
 * node always publishes LANE_FOLLOWING. The interface is kept stable so the node/debug wiring does
 * not change when the real logic lands.
 */
class LaneFollowingChecker
{
public:
  LaneFollowingChecker() = default;

  /**
   * @brief Evaluates the lane-following check for the current cycle.
   *
   * @note Stub: always reports the ego as following. The real logic (added in a follow-up PR)
   * queries the reference lane / connected sequence from the tracker, so this signature will gain
   * those inputs then.
   */
  [[nodiscard]] LaneFollowingResult evaluate() const;
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_FOLLOWING__CHECKER_HPP_
