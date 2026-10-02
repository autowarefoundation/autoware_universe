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

#include "autoware/trajectory_adapter/trajectory_adapter.hpp"

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <algorithm>
#include <string>

namespace autoware::trajectory_adapter
{

namespace
{

std::string get_generator_name(
  const ScoredCandidateTrajectories & scored_trajectories,
  const unique_identifier_msgs::msg::UUID & uuid)
{
  const auto generator_itr = std::find_if(
    scored_trajectories.generator_info.begin(), scored_trajectories.generator_info.end(),
    [&uuid](const auto & info) { return info.generator_id == uuid; });
  return generator_itr == scored_trajectories.generator_info.end()
           ? "NOT FOUND"
           : generator_itr->generator_name.data;
}

}  // namespace

tl::expected<TrajectoryAdapterResult, std::string> TrajectoryAdapter::process(
  const ScoredCandidateTrajectories & scored_trajectories) const
{
  if (scored_trajectories.scored_candidate_trajectories.empty()) {
    return tl::make_unexpected("Scored candidate trajectories are empty");
  }

  const auto trajectory_itr = std::max_element(
    scored_trajectories.scored_candidate_trajectories.begin(),
    scored_trajectories.scored_candidate_trajectories.end(),
    [](const auto & a, const auto & b) { return a.score < b.score; });

  TrajectoryAdapterResult result;
  result.best_generator_name =
    get_generator_name(scored_trajectories, trajectory_itr->candidate_trajectory.generator_id);
  result.best_score = trajectory_itr->score;
  result.trajectory = autoware_planning_msgs::build<Trajectory>()
                        .header(trajectory_itr->candidate_trajectory.header)
                        .points(trajectory_itr->candidate_trajectory.points);
  result.turn_indicators = trajectory_itr->candidate_trajectory.turn_indicators_command;

  return result;
}

}  // namespace autoware::trajectory_adapter
