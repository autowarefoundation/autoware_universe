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

#ifndef AUTOWARE__TRAJECTORY_CONCATENATOR__DETAIL__TRAJECTORY_CONCATENATOR_HPP_
#define AUTOWARE__TRAJECTORY_CONCATENATOR__DETAIL__TRAJECTORY_CONCATENATOR_HPP_

#include <autoware_trajectory_concatenator/autoware_trajectory_concatenator_param.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>

#include <string>
#include <unordered_map>
#include <utility>

namespace autoware::trajectory_concatenator
{

using autoware_internal_planning_msgs::msg::CandidateTrajectories;

/**
 * @brief Aggregates candidate trajectories from multiple generators into a single set.
 */
class TrajectoryConcatenator
{
public:
  /**
   * @brief Constructs the concatenator with the given parameters.
   * @param params Initial concatenator parameters.
   */
  explicit TrajectoryConcatenator(concatenator::Params params) : params_(std::move(params)) {}

  /**
   * @brief Replaces the current parameters.
   * @param params New parameter values.
   */
  void update_parameters(const concatenator::Params & params) { params_ = params; }

  /**
   * @brief Stores the latest trajectories from the message's generator.
   * @param msg Candidate trajectories message from a single generator.
   */
  void add_candidate(const CandidateTrajectories & msg);

  /**
   * @brief Returns the merged set of all non-stale generator trajectories.
   * @param current_time Current ROS time used to filter out stale entries.
   */
  [[nodiscard]] CandidateTrajectories get_concatenated(
    const builtin_interfaces::msg::Time & current_time);

private:
  concatenator::Params params_;
  std::unordered_map<std::string, CandidateTrajectories> buffer_;
};

}  // namespace autoware::trajectory_concatenator

#endif  // AUTOWARE__TRAJECTORY_CONCATENATOR__DETAIL__TRAJECTORY_CONCATENATOR_HPP_
