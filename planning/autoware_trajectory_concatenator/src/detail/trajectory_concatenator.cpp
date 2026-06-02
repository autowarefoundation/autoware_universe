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

#include "autoware/trajectory_concatenator/detail/trajectory_concatenator.hpp"

#include <autoware_utils_uuid/uuid_helper.hpp>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::trajectory_concatenator
{

namespace
{
double to_seconds(const builtin_interfaces::msg::Time & time)
{
  return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1e-9;
}
}  // namespace
using autoware_internal_planning_msgs::msg::CandidateTrajectory;

void TrajectoryConcatenator::add_candidate(const CandidateTrajectories & msg)
{
  std::unordered_map<std::string, std::vector<CandidateTrajectory>> by_generator;
  by_generator.reserve(msg.generator_info.size());
  for (const auto & traj : msg.candidate_trajectories) {
    by_generator[autoware_utils_uuid::to_hex_string(traj.generator_id)].push_back(traj);
  }
  for (const auto & generator_info : msg.generator_info) {
    const auto uuid = autoware_utils_uuid::to_hex_string(generator_info.generator_id);
    auto it = by_generator.find(uuid);
    buffer_[uuid] =
      autoware_internal_planning_msgs::build<CandidateTrajectories>()
        .candidate_trajectories(
          it != by_generator.end() ? std::move(it->second) : std::vector<CandidateTrajectory>{})
        .generator_info({generator_info});
  }
}

CandidateTrajectories TrajectoryConcatenator::get_concatenated(
  const builtin_interfaces::msg::Time & current_time)
{
  std::vector<autoware_internal_planning_msgs::msg::CandidateTrajectory> trajectories;
  std::vector<autoware_internal_planning_msgs::msg::GeneratorInfo> generator_info;

  const double current_time_sec = to_seconds(current_time);

  // Prune expired entries individually
  for (auto it = buffer_.begin(); it != buffer_.end();) {
    auto & pre_combine = it->second;

    pre_combine.candidate_trajectories.erase(
      std::remove_if(
        pre_combine.candidate_trajectories.begin(), pre_combine.candidate_trajectories.end(),
        [&](const auto & traj) {
          const double msg_time_sec = to_seconds(traj.header.stamp);
          return (current_time_sec - msg_time_sec) > params_.duration_time;
        }),
      pre_combine.candidate_trajectories.end());

    if (pre_combine.candidate_trajectories.empty()) {
      it = buffer_.erase(it);
    } else {
      it++;
    }
  }

  // Combine surviving entries
  for (const auto & [uuid, pre_combine] : buffer_) {
    trajectories.insert(
      trajectories.end(), pre_combine.candidate_trajectories.begin(),
      pre_combine.candidate_trajectories.end());
    generator_info.insert(
      generator_info.end(), pre_combine.generator_info.begin(), pre_combine.generator_info.end());
  }

  return autoware_internal_planning_msgs::build<CandidateTrajectories>()
    .candidate_trajectories(trajectories)
    .generator_info(generator_info);
}

}  // namespace autoware::trajectory_concatenator
