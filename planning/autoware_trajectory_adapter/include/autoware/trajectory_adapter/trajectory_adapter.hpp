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

#ifndef AUTOWARE__TRAJECTORY_ADAPTER__TRAJECTORY_ADAPTER_HPP_
#define AUTOWARE__TRAJECTORY_ADAPTER__TRAJECTORY_ADAPTER_HPP_

#include <tl_expected/expected.hpp>

#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectories.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <string>

namespace autoware::trajectory_adapter
{

using autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

/**
 * @brief Output of TrajectoryAdapter: the highest-scored trajectory and its turn indicators.
 */
struct TrajectoryAdapterResult
{
  Trajectory trajectory;
  TurnIndicatorsCommand turn_indicators;
  std::string best_generator_name;
  float best_score{0.0F};
};

/**
 * @brief Selects the highest-scored candidate trajectory and converts it to planning outputs.
 */
class TrajectoryAdapter
{
public:
  /**
   * @brief Returns the best-scored trajectory as a planning trajectory and turn indicators.
   * @param scored_trajectories Ranked candidate trajectories.
   * @return The adapted outputs, or std::nullopt if the input is empty.
   */
  [[nodiscard]] tl::expected<TrajectoryAdapterResult, std::string> process(
    const ScoredCandidateTrajectories & scored_trajectories) const;
};

}  // namespace autoware::trajectory_adapter

#endif  // AUTOWARE__TRAJECTORY_ADAPTER__TRAJECTORY_ADAPTER_HPP_
