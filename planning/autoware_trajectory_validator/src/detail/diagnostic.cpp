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

#include "autoware/trajectory_validator/detail/diagnostic.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator
{

DiagnosticInterfaceMap build_diagnostic_interface_map(
  rclcpp::Node & node, const FilterConfiguredActionsMap & filter_configured_actions_map,
  const std::string & no_candidates_diag_status_name)
{
  DiagnosticInterfaceMap diag_by_name;

  const auto register_status = [&](const std::string & status_name) {
    if (!status_name.empty() && !diag_by_name.count(status_name)) {
      diag_by_name.emplace(
        status_name,
        std::make_unique<autoware_utils_diagnostics::DiagnosticsInterface>(&node, status_name));
    }
  };

  for (const auto & [filter_name, configured_actions] : filter_configured_actions_map) {
    for (const auto & [configured_action, status_name] : configured_actions) {
      register_status(status_name);
    }
  }

  register_status(no_candidates_diag_status_name);
  return diag_by_name;
}

TrajectoryValidatorDiagnostic::TrajectoryValidatorDiagnostic(
  FilterConfiguredActionsMap filter_configured_actions_map,
  const trajectory_validator_diagnostic::Params & diagnostic_params,
  const std::unordered_set<std::string> & active_filter_names, DiagnosticInterfaceMap diag_by_name)
: filter_configured_actions_map_(std::move(filter_configured_actions_map)),
  active_filter_names_(active_filter_names),
  no_candidates_diag_status_name_(diagnostic_params.no_candidates_diag_status_name),
  diag_by_name_(std::move(diag_by_name))
{
}

void TrajectoryValidatorDiagnostic::get_active_statuses(
  const std::unordered_map<std::string, Action> & filter_current_action,
  std::unordered_map<std::string, int8_t> & diag_level_by_status_name) const
{
  const auto trigger_matching_status = [&](const std::string & name, Action action) {
    const auto map_it = filter_configured_actions_map_.find(name);
    if (map_it == filter_configured_actions_map_.end()) return;
    for (const auto & [configured_action, status_name] : map_it->second) {
      // Each configured_action fires only at its exact action level so MODERATE and EMERGENCY
      // statuses trigger independent diagnostic responses rather than both firing at EMERGENCY.
      if (
        !status_name.empty() && configured_action != Action::NONE && action == configured_action) {
        diag_level_by_status_name[status_name] = map_action_to_diagnostic_level(configured_action);
      }
    }
  };

  for (const auto & [filter_name, current_action] : filter_current_action) {
    if (current_action != Action::NONE) {
      trigger_matching_status(filter_name, current_action);
    }
  }
}

void TrajectoryValidatorDiagnostic::update_and_publish(
  const std::vector<autoware_trajectory_validator::msg::ValidationReport> & reports,
  const rclcpp::Time & stamp)
{
  // Within one candidate: returns most severe action for each filter.
  // A filter may produce multiple metrics for the same candidate; the most severe one is taken.
  const auto compute_candidate_traj_action = [&](const auto & report) {
    std::unordered_map<std::string, Action> candidate_traj_action;
    for (const auto & metric : report.metrics) {
      // Skip metrics from shadow-mode filters (those not in
      // active_filter_names_).
      if (!active_filter_names_.empty() && !active_filter_names_.count(metric.validator_name)) {
        continue;
      }
      const Action metric_action = convert_risk_level_to_action(metric.risk.level);
      auto [filter_entry, is_new_entry] =
        candidate_traj_action.emplace(metric.validator_name, metric_action);
      if (!is_new_entry) {
        filter_entry->second = std::max(filter_entry->second, metric_action);
      }
    }
    return candidate_traj_action;
  };

  // Across candidates: keep the best (min) action per filter.
  // If a filter passes on any candidate, its action stays NONE and no configured_action fires —
  // the car can always choose that safer trajectory.
  const auto accumulate_best_action_across_all_candidates =
    [](auto & filter_current_action, const auto & candidate_traj_action) {
      for (const auto & [name, action] : candidate_traj_action) {
        auto [filter_entry, is_new_entry] = filter_current_action.emplace(name, action);
        if (!is_new_entry) {
          filter_entry->second = std::min(filter_entry->second, action);
        }
      }
    };

  // Pre-fill every tracked status at OK. Active filters that trigger will override their
  // status_name to ERROR below. Shadow filters are skipped during aggregation so their status_name
  // stays at OK.
  std::unordered_map<std::string, int8_t> diag_level_by_status_name;
  for (const auto & status_name : diag_by_name_) {
    diag_level_by_status_name[status_name.first] = diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  if (reports.empty()) {
    if (!no_candidates_diag_status_name_.empty()) {
      diag_level_by_status_name[no_candidates_diag_status_name_] =
        diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
  } else {
    std::unordered_map<std::string, Action> filter_current_action;
    for (const auto & report : reports) {
      accumulate_best_action_across_all_candidates(
        filter_current_action, compute_candidate_traj_action(report));
    }
    get_active_statuses(filter_current_action, diag_level_by_status_name);
  }

  publish_all(diag_level_by_status_name, stamp);
}

void TrajectoryValidatorDiagnostic::publish_all(
  const std::unordered_map<std::string, int8_t> & diag_level_by_status_name,
  const rclcpp::Time & stamp)
{
  for (auto & [name, diag] : diag_by_name_) {
    diag->clear();
    const auto level = diag_level_by_status_name.at(name);
    if (level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
      diag->update_level_and_message(level, name + " triggered");
    }
    diag->publish(stamp);
  }
}

}  // namespace autoware::trajectory_validator
