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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__DIAGNOSTIC_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__DIAGNOSTIC_HPP_

#include "autoware/trajectory_validator/detail/risk_action.hpp"

#include <autoware_trajectory_validator/autoware_trajectory_validator_diagnostic_param.hpp>
#include <autoware_trajectory_validator/msg/validation_report.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator
{

// Maps a configured action threshold to the diagnostic status name it publishes.
//   { MODERATE  → "trajectory_validator_uncrossable_boundary_departure_danger",
//     EMERGENCY → "trajectory_validator_uncrossable_boundary_departure_fatal" }
using ConfiguredActionMap = std::unordered_map<Action, std::string>;

// Maps each filter's name to its ConfiguredActionMap.
//   { "uncrossable_boundary_departure_filter" → { MODERATE  → "...danger",
//                                                 EMERGENCY → "...fatal" } }
using FilterConfiguredActionsMap = std::unordered_map<std::string, ConfiguredActionMap>;

// Maps each diagnostic status name to its DiagnosticsInterface publisher.
//   { "trajectory_validator_uncrossable_boundary_departure_danger" → DiagnosticsInterface,
//     "trajectory_validator_uncrossable_boundary_departure_fatal"  → DiagnosticsInterface }
using DiagnosticInterfaceMap = std::unordered_map<
  std::string, std::unique_ptr<autoware_utils_diagnostics::DiagnosticsInterface>>;

/**
 * @brief Builds a FilterConfiguredActionsMap from a configured_actions string array loaded from
 *        trajectory_validator_diagnostic parameters.
 * @param configured_actions Each entry encodes one configured_action as
 *        filter_name:action:diagnostic_name. Entries that are empty or
 *        lack the two required colons are skipped silently.
 */
inline FilterConfiguredActionsMap make_filter_configured_actions_map(
  const std::vector<std::string> & configured_actions)
{
  FilterConfiguredActionsMap m;
  for (const auto & configured_action : configured_actions) {
    if (configured_action.empty()) {
      continue;
    }
    const auto first_colon = configured_action.find(':');
    if (first_colon == std::string::npos) {
      continue;
    }
    const auto second_colon = configured_action.find(':', first_colon + 1);
    if (second_colon == std::string::npos) {
      continue;
    }
    const auto filter_name = configured_action.substr(0, first_colon);
    const auto action_str =
      configured_action.substr(first_colon + 1, second_colon - first_colon - 1);
    const auto diagnostic_name = configured_action.substr(second_colon + 1);
    if (filter_name.empty() || diagnostic_name.empty()) {
      continue;
    }

    auto action = parse_action_string_to_enum(action_str);

    m[filter_name].emplace(std::make_pair(action, diagnostic_name));
  }
  return m;
}

/**
 * @brief Creates one DiagnosticsInterface per distinct non-empty status name found in
 *        filter_configured_actions_map, plus one for no_candidates_diag_status_name if non-empty.
 * @param node ROS 2 node used for publisher creation.
 * @param filter_configured_actions_map Mapping from filter_name to its configured_actions.
 * @param no_candidates_diag_status_name Status name published when no candidate trajectory is
 *        available.
 */
DiagnosticInterfaceMap build_diagnostic_interface_map(
  rclcpp::Node & node, const FilterConfiguredActionsMap & filter_configured_actions_map,
  const std::string & no_candidates_diag_status_name);

/**
 * @brief Aggregates ValidationReports into a per-trajectory action and republishes every tracked
 *        DiagnosticStatus every cycle so none go stale.
 *
 * Pure logic class: takes a pre-built DiagnosticsInterface map; does not hold a ROS node.
 */
class TrajectoryValidatorDiagnostic
{
public:
  /**
   * @brief Constructs the diagnostic handler with pre-built DiagnosticsInterface objects.
   * @param filter_configured_actions_map Mapping from filter_name to its configured_actions.
   * @param diagnostic_params Diagnostic parameters; no_candidates_diag_status_name is read from
   *        here.
   * @param active_filter_names Filter names (from plugin->get_name()) of active (non-shadow)
   *        filters. Only metrics from these filters contribute to action aggregation.
   *        An empty set means all filters are considered active.
   * @param diag_by_name Pre-built DiagnosticsInterface map (use build_diagnostic_interface_map).
   */
  TrajectoryValidatorDiagnostic(
    FilterConfiguredActionsMap filter_configured_actions_map,
    const trajectory_validator_diagnostic::Params & diagnostic_params,
    const std::unordered_set<std::string> & active_filter_names,
    DiagnosticInterfaceMap diag_by_name);

  /**
   * @brief Aggregates reports into a best-available action and publishes all tracked statuses.
   * @param reports One ValidationReport per candidate trajectory.
   * @param stamp Timestamp forwarded to each DiagnosticsInterface::publish call.
   */
  void update_and_publish(
    const std::vector<autoware_trajectory_validator::msg::ValidationReport> & reports,
    const rclcpp::Time & stamp);

private:
  /**
   * @brief For each filter, fires the configured_action whose threshold exactly matches the
   *        filter's current action.
   *
   * A filter with both a MODERATE and EMERGENCY configured_action fires only the MODERATE one
   * when its current action is MODERATE, and only the EMERGENCY one when its current action is
   * EMERGENCY — each level triggers an independent diagnostic response.
   * @param filter_current_action Current action level reached by each filter (minimum across all
   *        candidates: NONE if the filter passed on at least one candidate, otherwise the worst
   *        level seen).
   * @param diag_level_by_status_name Output map of status name to DiagnosticStatus level.
   */
  void get_active_statuses(
    const std::unordered_map<std::string, Action> & filter_current_action,
    std::unordered_map<std::string, int8_t> & diag_level_by_status_name) const;

  /**
   * @brief Resets every tracked DiagnosticsInterface, applies levels, and publishes.
   * @param diag_level_by_status_name Complete map of every tracked status name to its
   *        DiagnosticStatus level for this cycle. Entries at OK are published without a
   *        message; non-OK entries publish with a "<name> triggered" message.
   * @param stamp Timestamp forwarded to each publish call.
   */
  void publish_all(
    const std::unordered_map<std::string, int8_t> & diag_level_by_status_name,
    const rclcpp::Time & stamp);

  FilterConfiguredActionsMap filter_configured_actions_map_;
  std::unordered_set<std::string> active_filter_names_;
  std::string no_candidates_diag_status_name_;
  DiagnosticInterfaceMap diag_by_name_;
};

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__DIAGNOSTIC_HPP_
