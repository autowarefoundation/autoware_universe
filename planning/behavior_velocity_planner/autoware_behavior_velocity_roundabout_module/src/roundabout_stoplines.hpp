// Copyright 2025 TIER IV, Inc.
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

#ifndef ROUNDABOUT_STOPLINES_HPP_
#define ROUNDABOUT_STOPLINES_HPP_

#include <optional>

namespace autoware::behavior_velocity_planner
{

/**
 * @brief see the document for more details of RoundaboutStopLines
 */
struct RoundaboutStopLines
{
  size_t closest_idx{0};

  /**
   * default_stopline is null if it is calculated negative from first_attention_stopline
   */
  std::optional<size_t> default_stopline{std::nullopt};

  /**
   * collision_stopline is ahead of closest_idx by the braking distance
   */
  size_t collision_stopline{0};

  /**
   * first_attention_stopline is null if ego footprint along the path does not intersect with
   * attention area. if path[0] satisfies the condition, it is 0
   */
  std::optional<size_t> first_attention_stopline{std::nullopt};

  /**
   * first_pass_judge_line is before first_attention_stopline by the braking distance. if its value
   * is calculated negative, it is 0
   */
  size_t first_pass_judge_line{0};
};
}  // namespace autoware::behavior_velocity_planner

#endif  // ROUNDABOUT_STOPLINES_HPP_
