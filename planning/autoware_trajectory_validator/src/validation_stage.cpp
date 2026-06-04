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

#include "autoware/trajectory_validator/validation_stage.hpp"

#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator
{

ValidationStageReport ValidationStage::process(
  const autoware_internal_planning_msgs::msg::CandidateTrajectories & input_trajectories,
  const EvaluationContext & context) const
{
  ValidationStageReport report;
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("Total");

  // O(N) Map UUID to generator names upfront to preserve performance
  std::unordered_map<std::string, std::string> uuid_to_name;
  uuid_to_name.reserve(input_trajectories.generator_info.size());
  for (const auto & info : input_trajectories.generator_info) {
    uuid_to_name[autoware_utils_uuid::to_hex_string(info.generator_id)] = info.generator_name.data;
  }

  for (const auto & trajectory : input_trajectories.candidate_trajectories) {
    EvaluationTable table;
    const auto hex_generator_id = autoware_utils_uuid::to_hex_string(trajectory.generator_id);
    table.generator_id = hex_generator_id;

    std::vector<autoware_trajectory_validator::msg::MetricReport> combined_metrics;

    // Evaluate Hard Constraints (Validators)
    for (const auto & plugin : validators_) {
      PluginEvaluation evaluation;
      evaluation.plugin_name = plugin->get_name();
      evaluation.is_shadow_mode = plugin->is_shadow_mode();

      stop_watch.tic(evaluation.plugin_name);
      const auto res = plugin->is_feasible(trajectory.points, context);

      // Preserve raw per-plugin verdict exactly as the original implementation
      if (!res) {
        evaluation.is_feasible = false;
        evaluation.reason = res.error();
      } else {
        const auto & val = res.value();
        evaluation.is_feasible = val.is_feasible;  // Recorded regardless of shadow mode
        if (!val.is_feasible) {
          evaluation.reason = "Found failed metrics";
        }
        combined_metrics.insert(combined_metrics.end(), val.metrics.begin(), val.metrics.end());
      }

      report.processing_time_ms[evaluation.plugin_name] += stop_watch.toc(evaluation.plugin_name);

      // Populate flat list and categorized map
      table.plugin_evaluations.push_back(evaluation);
    }

    report.evaluation_tables.push_back(table);

    // Final filtering depends exclusively on all_acceptable()
    if (table.all_acceptable()) {
      report.valid_trajectories.candidate_trajectories.push_back(trajectory);
    }

    const bool all_feasible = table.all_feasible();
    if (all_feasible) {
      report.num_feasible_trajectories++;
    }

    // Build Validation Report
    report.validation_reports.push_back(
      autoware_trajectory_validator::build<autoware_trajectory_validator::msg::ValidationReport>()
        .trajectory_stamp(trajectory.header.stamp)
        .generator_id(trajectory.generator_id)
        .generator_name(uuid_to_name.at(hex_generator_id))
        .level(
          all_feasible ? autoware_trajectory_validator::msg::ValidationReport::OK
                       : autoware_trajectory_validator::msg::ValidationReport::ERROR)
        .metrics(std::move(combined_metrics)));
  }

  // Filter generator_info to match surviving trajectories
  for (const auto & traj : report.valid_trajectories.candidate_trajectories) {
    auto it = std::find_if(
      input_trajectories.generator_info.begin(), input_trajectories.generator_info.end(),
      [&](const auto & info) { return traj.generator_id.uuid == info.generator_id.uuid; });

    if (it != input_trajectories.generator_info.end()) {
      report.valid_trajectories.generator_info.push_back(*it);
    }
  }

  report.processing_time_ms["Total"] = stop_watch.toc("Total");
  return report;
}

}  // namespace autoware::trajectory_validator
