// Copyright 2024 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_INTERSECTION_MODULE__INTERSECTION_STOPLINES_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_INTERSECTION_MODULE__INTERSECTION_STOPLINES_HPP_

#include <geometry_msgs/msg/pose.hpp>

#include <optional>

namespace autoware::behavior_velocity_planner
{

/**
 * @brief see the document for more details of IntersectionStopLines
 */
struct IntersectionStopLines
{
  size_t closest_idx{0};

  /**
   * stuck_stopline is null if ego path does not intersect with first_conflicting_area
   */
  std::optional<size_t> stuck_stopline{std::nullopt};

  /**
   * default_stopline is
   * - at RoadMarking position or at the position which is before `first_attention_stopline` by
   * `stopline_margin`, if it is feasible (A)
   */
  std::optional<size_t> default_stopline{std::nullopt};

  /**
   * collision_stopline is
   * - at default_stopline if it is feasible
   * - if above is not feasible and ego is before `pass_judge_line`
   *   - and if `previous_collision_stopline_pose` is null, at the position which is after
   *     `closest_idx` by the braking_distance. And this position is saved as
   *     `previous_collision_stopline_pose` (B)
   *   - else, at `previous_collision_stopline_pose` (C)
   * - otherwise null (D)
   */
  std::optional<size_t> collision_stopline{std::nullopt};

  /**
   * first_attention_stopline is null if ego footprint along the path does not intersect with
   * attention area. if path[0] satisfies the condition, it is 0
   */
  size_t first_attention_stopline{0};

  /**
   * occlusion_peeking_stopline is
   * - after `first_attention_stopline` by `peeking_offset` if it is feasible (E)
   * - if above is not feasible and ego is before `pass_judge_line`
   *   - and if `previous_occlusion_peeking_stopline_pose` is null, at the position which is after
   *     `closest_idx` by the braking distance. And this position is saved as
   *     `previous_occlusion_peeking_stopline_pose` (F)
   *   - else, at `previous_occlusion_peeking_stopline_pose` (G)
   * - otherwise null (H)
   */
  std::optional<size_t> occlusion_peeking_stopline{std::nullopt};

  /**
   * pass_judge_line is before first_attention_stopline by the braking distance. if its value
   * is calculated negative, it is 0
   */
  size_t pass_judge_line{0};

  /**
   * maximum_footprint_overshoot_line is the place after first_attention_stopline where ego's
   * footprint most gets close to the left/right boundary of assigned_lane when ego is turning
   * right/left respectively
   */
  std::optional<size_t> maximum_footprint_overshoot_line{std::nullopt};

  struct PreviousStopPose
  {
    /**
     * collision_stopline_pose is non-null if `collision_stopline` was not statically
     * feasible (B or C)
     */
    std::optional<geometry_msgs::msg::Pose> collision_stopline_pose{std::nullopt};

    /**
     * occlusion_peeking_stopline_pose is non-null if `occlusion_peeking_stopline` was not
     * statically feasible (F or G)
     */
    std::optional<geometry_msgs::msg::Pose> occlusion_peeking_stopline_pose{std::nullopt};
  };
};
}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_INTERSECTION_MODULE__INTERSECTION_STOPLINES_HPP_
