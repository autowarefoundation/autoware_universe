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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__RISK_ACTION_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__RISK_ACTION_HPP_

#include <autoware_trajectory_validator/msg/risk_level.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <string>

namespace autoware::trajectory_validator
{

/** @brief Severity-ordered action the diagnostic system reacts to. */
enum class Action : uint8_t { NONE = 0, COMFORTABLE = 1, MODERATE = 2, EMERGENCY = 3 };

/**
 * @brief Translates a MetricReport.level into an Action.
 * @note This is the ONLY place MetricReport level values (OK/WARN/ERROR) are named.
 * @param risk_level Level value from MetricReport (OK/WARN/ERROR).
 */
inline Action convert_risk_level_to_action(uint8_t risk_level)
{
  using autoware_trajectory_validator::msg::RiskLevel;

  switch (risk_level) {
    case RiskLevel::HIGH_CAUTION:
      return Action::COMFORTABLE;
    case RiskLevel::DANGER:
      return Action::MODERATE;
    case RiskLevel::FATAL:
      return Action::EMERGENCY;
    default:
      return Action::NONE;
  }
}

/**
 * @brief Parses an action string from a parameter into an Action value.
 * @param action_str One of "none", "comfortable", "moderate", "emergency".
 */
inline Action parse_action_string_to_enum(const std::string & action_str)
{
  if (action_str == "comfortable") return Action::COMFORTABLE;
  if (action_str == "moderate") return Action::MODERATE;
  if (action_str == "emergency") return Action::EMERGENCY;
  return Action::NONE;
}

/**
 * @brief Maps an Action to its published DiagnosticStatus level.
 * @param action Action to map.
 */
inline int8_t map_action_to_diagnostic_level(Action action)
{
  switch (action) {
    case Action::NONE:
      return diagnostic_msgs::msg::DiagnosticStatus::OK;
    case Action::COMFORTABLE:
    case Action::MODERATE:
    case Action::EMERGENCY:
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  return diagnostic_msgs::msg::DiagnosticStatus::OK;
}

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__RISK_ACTION_HPP_
