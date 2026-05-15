// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__TRAJECTORY_VALIDATION_ACCUMULATOR_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__TRAJECTORY_VALIDATION_ACCUMULATOR_HPP_

#include "autoware/planning_evaluator/metrics/metric.hpp"
#include "autoware/planning_evaluator/metrics/output_metric.hpp"

#include <autoware_trajectory_validator/msg/validation_report_array.hpp>
#include <autoware_utils/math/accumulator.hpp>
#include <nlohmann/json.hpp>

#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <optional>
#include <string>
#include <unordered_map>

namespace planning_diagnostics
{
using autoware_trajectory_validator::msg::ValidationReportArray;
using autoware_utils::Accumulator;
using MetricMsg = tier4_metric_msgs::msg::Metric;
using MetricArrayMsg = tier4_metric_msgs::msg::MetricArray;
using json = nlohmann::json;

/**
 * @class TrajectoryValidationAccumulator
 * @brief Accumulates trajectory validation error statistics from ValidationReportArray.
 *
 * Two scopes (published under metric_to_str(Metric::trajectory_validation) + "/" + <scope> + "/…"):
 * 1) Whole trajectory (per candidate / generator), from ValidationReport.level —
 *    <scope> = <generator_name>
 * 2) Each MetricReport row —
 *    <scope> = <generator_name>/<validator_name>/<metric_name>
 *    Per generator, metric scopes already present in stats (keys `gen/validator/metric` in
 *    stats_by_scope_) are updated every report; missing rows are treated as OK for that step.
 *
 * `count_warn_as_error`: if false, only ERROR(2); if true, WARN(1) also counts as error.
 *
 * Metric rows whose `metric_name` matches `check_*_<32-hex UUID>` (optional `_<suffix>`) are
 * skipped (object-id-specific checks); see `shouldCollectMetricRow` in
 * trajectory_validation_accumulator.cpp.
 */
class TrajectoryValidationAccumulator
{
public:
  struct Parameters
  {
    bool count_warn_as_error = false;
  } parameters;

  TrajectoryValidationAccumulator() = default;
  ~TrajectoryValidationAccumulator() = default;

  /**
   * @brief Consume a new validation report array (one message may contain multiple trajectories).
   */
  void update(const ValidationReportArray & msg);

  /**
   * @brief Append live metrics for publishing (names follow the scheme in the class comment).
   */
  bool addMetricMsg(const Metric & metric, MetricArrayMsg & metrics_msg);

  /**
   * @brief Final statistics for output.json (min/max/mean/total error_duration, error_count, etc.).
   */
  json getOutputJson(const OutputMetric & output_metric);

  static std::string makeMetricScopeKey(
    const std::string & generator_name, const std::string & validator_name,
    const std::string & metric_name)
  {
    return generator_name + "/" + validator_name + "/" + metric_name;
  }

private:
  struct ErrorSpanStats
  {
    double current_error_duration_s{0.0};
    uint64_t error_count{0};
    /// Sum of completed error-span durations (seconds); complements min/max/mean samples.
    double completed_error_duration_total_s_{0.0};
    Accumulator<double> error_duration_accumulator;
    bool state_updated{false};
    bool in_error_{false};
    /// Stamp of the last report step applied to this scope (trajectory_stamp seconds).
    std::optional<double> last_update_time_s_;

    /// `current_time_s` is ValidationReport.trajectory_stamp for this step; dt is derived inside.
    void update(bool is_error, double current_time_s);
    /// Move ongoing error duration into the accumulator (e.g. before final JSON export).
    void flushOpenErrorForJson();
  };

  std::unordered_map<std::string, ErrorSpanStats> stats_by_scope_;
};

}  // namespace planning_diagnostics

#endif  // AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__TRAJECTORY_VALIDATION_ACCUMULATOR_HPP_
