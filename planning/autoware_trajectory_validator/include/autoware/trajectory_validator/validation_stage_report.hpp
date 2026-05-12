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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__VALIDATION_STAGE_REPORT_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__VALIDATION_STAGE_REPORT_HPP_

#include <autoware_trajectory_validator/msg/validation_report.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_validator
{

struct PluginEvaluation
{
  std::string plugin_name;
  bool is_feasible{true};
  bool is_shadow_mode{false};
  std::string reason;
};

struct EvaluationTable
{
  std::string generator_id;
  std::vector<PluginEvaluation> plugin_evaluations;

  [[nodiscard]] bool all_acceptable() const
  {
    return std::all_of(plugin_evaluations.begin(), plugin_evaluations.end(), [](const auto & e) {
      return e.is_feasible || e.is_shadow_mode;
    });
  }

  [[nodiscard]] bool all_feasible() const
  {
    return std::all_of(plugin_evaluations.begin(), plugin_evaluations.end(), [](const auto & e) {
      return e.is_feasible;
    });
  }
};

/**
 * @brief Final opaque result structure returned by the validation stage.
 * CONTRACT FOR ROS NODE ADAPTER:
 * The Node MUST iterate over `evaluation_tables` post-process to:
 * 1. Emit `RCLCPP_WARN_THROTTLE` for any `!evaluation.is_feasible`.
 * 2. Update the `diagnostics_interface_` per plugin using the final evaluation states.
 */
struct ValidationStageReport
{
  autoware_internal_planning_msgs::msg::CandidateTrajectories valid_trajectories;
  std::vector<EvaluationTable> evaluation_tables;
  std::vector<autoware_trajectory_validator::msg::ValidationReport> validation_reports;
  size_t num_feasible_trajectories{0};

  std::unordered_map<std::string, double> processing_time_ms;
};

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__VALIDATION_STAGE_REPORT_HPP_
