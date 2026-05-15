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

#include "autoware/planning_evaluator/metric_accumulators/trajectory_validation_accumulator.hpp"

#include <regex>

namespace planning_diagnostics
{
namespace
{
bool levelIndicatesError(const uint8_t level, const bool count_warn_as_error)
{
  if (level >= 2) {
    return true;
  }
  return count_warn_as_error && level == 1;
}

/// Skip one-off object-id rows: metric_name like check_<prefix>_<32-hex> or
/// check_<...>_<32-hex>_<suffix>.
bool shouldCollectMetricRow(
  [[maybe_unused]] const std::string & validator_name, const std::string & metric_name)
{
  if (std::regex_match(metric_name, std::regex{R"(^check_.*_[0-9a-fA-F]{32}(?:_.*)?$)"})) {
    return false;
  }
  return true;
}

bool isMetricScopeUnderGenerator(const std::string & scope, const std::string & gen)
{
  const std::string prefix = gen + "/";
  return scope.size() > prefix.size() && scope.compare(0, prefix.size(), prefix) == 0;
}
}  // namespace

void TrajectoryValidationAccumulator::ErrorSpanStats::update(
  const bool is_error, const double current_time_s)
{
  double dt = 0.0;
  if (last_update_time_s_.has_value()) {
    dt = current_time_s - last_update_time_s_.value();
    if (dt < 0.0) {
      dt = 0.0;
    }
  }
  last_update_time_s_ = current_time_s;

  if (is_error) {
    if (!in_error_) {
      in_error_ = true;
      error_count++;
      current_error_duration_s = 0.0;
    } else {
      current_error_duration_s += dt;
    }
  } else if (in_error_) {
    error_duration_accumulator.add(current_error_duration_s);
    completed_error_duration_total_s_ += current_error_duration_s;
    in_error_ = false;
    current_error_duration_s = 0.0;
  }
  state_updated = true;
}

void TrajectoryValidationAccumulator::ErrorSpanStats::flushOpenErrorForJson()
{
  if (!in_error_) {
    return;
  }
  error_duration_accumulator.add(current_error_duration_s);
  completed_error_duration_total_s_ += current_error_duration_s;
  current_error_duration_s = 0.0;
  in_error_ = false;
}

void TrajectoryValidationAccumulator::update(const ValidationReportArray & msg)
{
  if (msg.reports.empty()) {
    return;
  }

  for (const auto & report : msg.reports) {
    const std::string & gen = report.generator_name;
    const auto & stamp = report.trajectory_stamp;
    const double report_time_s =
      static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;

    const bool warn_as_err = parameters.count_warn_as_error;
    const bool traj_err = levelIndicatesError(report.level, warn_as_err);
    stats_by_scope_[gen].update(traj_err, report_time_s);

    // Rows in this report (last row wins if the same scope appears more than once).
    std::unordered_map<std::string, bool> present_error_by_scope;
    for (const auto & row : report.metrics) {
      if (!shouldCollectMetricRow(row.validator_name, row.metric_name)) {
        continue;
      }
      const std::string scope = gen + "/" + row.validator_name + "/" + row.metric_name;
      present_error_by_scope[scope] = levelIndicatesError(row.level, warn_as_err);
    }

    for (const auto & [scope, m_err] : present_error_by_scope) {
      stats_by_scope_[scope].update(m_err, report_time_s);
    }

    for (const auto & entry : stats_by_scope_) {
      const std::string & scope = entry.first;
      if (!isMetricScopeUnderGenerator(scope, gen)) {
        continue;
      }
      if (present_error_by_scope.count(scope) != 0) {
        continue;
      }
      stats_by_scope_[scope].update(false, report_time_s);
    }
  }
}

bool TrajectoryValidationAccumulator::addMetricMsg(
  const Metric & metric, MetricArrayMsg & metrics_msg)
{
  if (metric != Metric::trajectory_validation) {
    return false;
  }
  const std::string base = metric_to_str.at(Metric::trajectory_validation) + "/";
  MetricMsg m;
  bool any = false;
  for (auto & [scope, st] : stats_by_scope_) {
    if (!st.state_updated) {
      continue;
    }
    m.name = base + scope + "/error_duration";
    m.value = std::to_string(st.current_error_duration_s);
    metrics_msg.metric_array.push_back(m);
    m.name = base + scope + "/error_count";
    m.value = std::to_string(st.error_count);
    metrics_msg.metric_array.push_back(m);
    st.state_updated = false;
    any = true;
  }
  return any;
}

json TrajectoryValidationAccumulator::getOutputJson(const OutputMetric & output_metric)
{
  json j;
  if (output_metric != OutputMetric::trajectory_validation) {
    return j;
  }
  for (auto & [scope, st] : stats_by_scope_) {
    st.flushOpenErrorForJson();
    if (st.error_duration_accumulator.count() > 0) {
      j[scope + "/error_duration/min"] = st.error_duration_accumulator.min();
      j[scope + "/error_duration/max"] = st.error_duration_accumulator.max();
      j[scope + "/error_duration/mean"] = st.error_duration_accumulator.mean();
      j[scope + "/error_duration/total"] = st.completed_error_duration_total_s_;
    }
    if (st.error_count > 0) {
      j[scope + "/error_count"] = st.error_count;
    }
  }
  return j;
}

}  // namespace planning_diagnostics
