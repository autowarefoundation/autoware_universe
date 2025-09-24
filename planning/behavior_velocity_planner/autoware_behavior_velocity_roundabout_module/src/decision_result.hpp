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

#ifndef DECISION_RESULT_HPP_
#define DECISION_RESULT_HPP_

#include <optional>
#include <string>
#include <variant>

namespace autoware::behavior_velocity_planner
{

/**
 * @brief internal error
 */
struct InternalError
{
  std::string error;
};

/**
 * @brief over the pass judge lines
 */
struct OverPassJudge
{
  std::string safety_report;
  std::string evasive_report;
};

/**
 * @brief only collision is detected
 */
struct CollisionStop
{
  size_t closest_idx{0};
  size_t collision_stopline_idx{0};
};

/**
 * @brief both collision is not detected
 */
struct Safe
{
  size_t closest_idx{0};
  size_t collision_stopline_idx{0};
};

using DecisionResult = std::variant<
  InternalError,  //! internal process error
  OverPassJudge,  //! over the pass judge lines
  CollisionStop,  //! detected collision while FOV is clear
  Safe            //! judge as safe
  >;

std::string formatDecisionResult(const DecisionResult & decision_result, const bool int_activated);

}  // namespace autoware::behavior_velocity_planner

#endif  // DECISION_RESULT_HPP_
