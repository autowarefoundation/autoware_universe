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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__TRAJECTORY_VALIDATOR_REPORT_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__TRAJECTORY_VALIDATOR_REPORT_HPP_

#include <autoware_trajectory_validator/msg/validation_report.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_validator
{

/** @brief Feasibility result of a single plugin for a single trajectory. */
struct PluginEvaluation
{
  std::string plugin_name;
  bool is_feasible{true};
  bool is_shadow_mode{false};
  std::string reason;
};

/** @brief Aggregated plugin evaluations for a single generator's trajectory. */
struct EvaluationTable
{
  std::string generator_id;
  std::vector<PluginEvaluation> plugin_evaluations;

  /** @brief Returns true if every plugin passed or is in shadow mode. */
  [[nodiscard]] bool all_acceptable() const
  {
    return std::all_of(plugin_evaluations.begin(), plugin_evaluations.end(), [](const auto & e) {
      return e.is_feasible || e.is_shadow_mode;
    });
  }

  /** @brief Returns true if every plugin passed, ignoring shadow mode. */
  [[nodiscard]] bool all_feasible() const
  {
    return std::all_of(plugin_evaluations.begin(), plugin_evaluations.end(), [](const auto & e) {
      return e.is_feasible;
    });
  }
};

/** @brief Result returned by TrajectoryValidator::process. */
struct TrajectoryValidatorReport
{
  autoware_internal_planning_msgs::msg::CandidateTrajectories valid_trajectories;
  std::vector<EvaluationTable> evaluation_tables;
  std::vector<autoware_trajectory_validator::msg::ValidationReport> validation_reports;
  size_t num_feasible_trajectories{0};

  std::unordered_map<std::string, double> processing_time_ms;
};

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__TRAJECTORY_VALIDATOR_REPORT_HPP_
