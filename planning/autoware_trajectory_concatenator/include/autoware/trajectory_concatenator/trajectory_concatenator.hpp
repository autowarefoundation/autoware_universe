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

#ifndef AUTOWARE__TRAJECTORY_CONCATENATOR__TRAJECTORY_CONCATENATOR_HPP_
#define AUTOWARE__TRAJECTORY_CONCATENATOR__TRAJECTORY_CONCATENATOR_HPP_

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
 * @brief Stateful aggregator of trajectories from multiple generators.
 * Uses a most-recent-per-generator buffer with time-based stale pruning.
 * * * Pure C++ Library: This class is completely independent of rclcpp.
 * * Thread-safety: Not thread-safe. Must be protected by a mutex by the caller.
 * * ARCHITECTURAL NOTE ON LATENCY:
 * Because this stage is designed to be flushed via a fixed-rate timer (e.g., 30ms),
 * it introduces an average ~15ms (max 30ms) latency floor to the pipeline. Consequently,
 * all downstream outputs (including debug markers) are quantized to this timer's rate.
 */
class TrajectoryConcatenator
{
public:
  explicit TrajectoryConcatenator(concatenator::Params params) : params_(std::move(params)) {}

  void update_parameters(const concatenator::Params & params) { params_ = params; }

  void add_candidate(const CandidateTrajectories & msg);

  [[nodiscard]] CandidateTrajectories get_concatenated(
    const builtin_interfaces::msg::Time & current_time);

private:
  concatenator::Params params_;
  std::unordered_map<std::string, CandidateTrajectories> buffer_;
};

}  // namespace autoware::trajectory_concatenator

#endif  // AUTOWARE__TRAJECTORY_CONCATENATOR__TRAJECTORY_CONCATENATOR_HPP_
