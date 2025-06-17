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

#include "autoware/planning_evaluator/metric_accumulators/planning_factor_accumulator.hpp"

#include <cmath>
#include <cstdlib>
#include <optional>

namespace planning_diagnostics
{

void PlanningFactorAccumulator::update(
  const std::string & module_name, const PlanningFactorArray & planning_factors,
  const Odometry & ego_state)
{
  // initialize new module for stop decision
  stop_decision_modules_.emplace(module_name);
  stop_decision_state_.emplace(module_name, StopDecisionState());
  abnormal_stop_decision_state_.emplace(module_name, StopDecisionState());

  if (planning_factors.factors.empty()) {
    return;
  }

  const auto cur_time = static_cast<double>(planning_factors.header.stamp.sec) +
                        static_cast<double>(planning_factors.header.stamp.nanosec) * 1e-9;

  // find the closest stop point
  std::optional<ControlPoint> closest_stop_point;
  for (const auto & factor : planning_factors.factors) {
    if (factor.behavior == PlanningFactor::STOP) {
      for (const auto & control_point : factor.control_points) {
        if (
          !closest_stop_point.has_value() ||
          std::abs(control_point.distance) < std::abs(closest_stop_point->distance)) {
          closest_stop_point = control_point;
        }
      }
    }
  }

  if (closest_stop_point.has_value()) {
    stop_decision_state_[module_name].update_stop_decision(
      closest_stop_point.value(), cur_time, parameters);

    const double min_dist_to_stop = ego_state.twist.twist.linear.x *
                                    ego_state.twist.twist.linear.x /
                                    (2.0 * parameters.abnormal_deceleration_threshold_mps2);
    const bool vel_near_to_stop = std::abs(ego_state.twist.twist.linear.x) <
                                  1.38888;  // don't count abnormal stop if vel < 5km/h
    if (!vel_near_to_stop && closest_stop_point->distance < min_dist_to_stop) {
      abnormal_stop_decision_state_[module_name].update_stop_decision(
        closest_stop_point.value(), cur_time, parameters);
    }
  }
}

bool PlanningFactorAccumulator::addMetricMsg(
  const Metric & metric, MetricArrayMsg & metrics_msg, const std::string & module_name)
{
  const std::string base_name = metric_to_str.at(metric) + "/";
  MetricMsg metric_msg;
  bool added = false;
  if (metric == Metric::stop_decision || metric == Metric::abnormal_stop_decision) {
    if (stop_decision_modules_.count(module_name) == 0) {
      return false;
    }
    auto & state = metric == Metric::stop_decision ? stop_decision_state_.at(module_name)
                                                   : abnormal_stop_decision_state_.at(module_name);
    if (state.state_updated) {
      metric_msg.name = base_name + module_name + "/keep_duration";
      metric_msg.value = std::to_string(state.stop_decision_keep_time);
      metrics_msg.metric_array.push_back(metric_msg);

      metric_msg.name = base_name + module_name + "/distance_to_stop";
      metric_msg.value = std::to_string(state.distance_to_stop);
      metrics_msg.metric_array.push_back(metric_msg);

      metric_msg.name = base_name + module_name + "/count";
      metric_msg.value = std::to_string(state.stop_decision_keep_time_accumulator.count() + 1);
      metrics_msg.metric_array.push_back(metric_msg);
      added = true;
      state.state_updated = false;
    }
  }
  return added;
}

json PlanningFactorAccumulator::getOutputJson(const OutputMetric & output_metric)
{
  json j;

  if (
    output_metric == OutputMetric::stop_decision ||
    output_metric == OutputMetric::abnormal_stop_decision) {
    for (const auto & module : stop_decision_modules_) {
      auto & state = output_metric == OutputMetric::stop_decision
                       ? stop_decision_state_.at(module)
                       : abnormal_stop_decision_state_.at(module);
      state.clear_current_stop_decision();
      // update the json output
      if (state.stop_decision_keep_time_accumulator.count() > 0) {
        j[module + "/count"] = state.stop_decision_keep_time_accumulator.count();
        j[module + "/keep_duration/min"] = state.stop_decision_keep_time_accumulator.min();
        j[module + "/keep_duration/max"] = state.stop_decision_keep_time_accumulator.max();
        j[module + "/keep_duration/mean"] = state.stop_decision_keep_time_accumulator.mean();
      }
    }
  }

  return j;
}

}  // namespace planning_diagnostics
