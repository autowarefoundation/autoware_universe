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
#include <optional>

namespace planning_diagnostics
{

void PlanningFactorAccumulator::update(
  const std::string & module_name, const PlanningFactorArray & planning_factors,
  const Odometry & ego_state)
{
  if (planning_factors.factors.empty()) {
    return;
  }
  const auto cur_time = static_cast<double>(planning_factors.header.stamp.sec) +
                        static_cast<double>(planning_factors.header.stamp.nanosec) * 1e-9;

  // Update status and count for stop_decision and abnormal_stop_decision
  // initialize new module
  stop_decision_modules_.emplace(module_name);
  stop_decision_state_.emplace(module_name, StopDecisionState());
  abnormal_stop_decision_state_.emplace(module_name, StopDecisionState());

  // find the nearest stop point
  std::optional<ControlPoint> nearest_stop_point;
  for (const auto & factor : planning_factors.factors) {
    if (factor.behavior == PlanningFactor::STOP) {
      for (const auto & control_point : factor.control_points) {
        if (
          !nearest_stop_point.has_value() ||
          std::abs(control_point.distance) < std::abs(nearest_stop_point->distance)) {
          nearest_stop_point = control_point;
        }
      }
    }
  }
  // a function to update stop decision state
  auto update_stop_decision_state = [&](
                                      const bool is_stop, const ControlPoint & cur_stop_point,
                                      StopDecisionState & stop_decision_state) {
    if (!is_stop) {
      if (stop_decision_state.is_stop_decision) {
        stop_decision_state.stop_decision_keep_time_accumulator.add(
          stop_decision_state.stop_decision_keep_time);
      }
      stop_decision_state.is_stop_decision = false;
      stop_decision_state.stop_decision_keep_time = std::numeric_limits<double>::quiet_NaN();
      stop_decision_state.distance_to_stop = std::numeric_limits<double>::quiet_NaN();
      return;
    }

    const double time_to_last_stop_decision = cur_time - stop_decision_state.last_stop_time;
    const double dist_to_last_stop_decision = std::hypot(
      cur_stop_point.pose.position.x - stop_decision_state.last_stop_x,
      cur_stop_point.pose.position.y - stop_decision_state.last_stop_y,
      cur_stop_point.pose.position.z - stop_decision_state.last_stop_z);
    if (
      time_to_last_stop_decision > parameters.time_count_threshold_s ||
      dist_to_last_stop_decision > parameters.dist_count_threshold_m ||
      !stop_decision_state.is_stop_decision) {
      stop_decision_state.last_stop_time = cur_time;
      stop_decision_state.stop_decision_keep_time = 0.0;
      stop_decision_state.last_stop_x = cur_stop_point.pose.position.x;
      stop_decision_state.last_stop_y = cur_stop_point.pose.position.y;
      stop_decision_state.last_stop_z = cur_stop_point.pose.position.z;
    } else {
      stop_decision_state.stop_decision_keep_time = time_to_last_stop_decision;
    }
    stop_decision_state.is_stop_decision = true;
    stop_decision_state.distance_to_stop = std::abs(cur_stop_point.distance);
  };

  if (nearest_stop_point.has_value()) {
    // update stop decision related status and count
    update_stop_decision_state(true, nearest_stop_point.value(), stop_decision_state_[module_name]);

    // update abnormal stop decision related status and count
    const double min_dist_to_stop = ego_state.twist.twist.linear.x *
                                    ego_state.twist.twist.linear.x /
                                    (2.0 * parameters.abnormal_deceleration_threshold_mps2);
    update_stop_decision_state(
      nearest_stop_point->distance < min_dist_to_stop, nearest_stop_point.value(),
      abnormal_stop_decision_state_[module_name]);

  } else {
    stop_decision_state_[module_name].is_stop_decision = false;
    abnormal_stop_decision_state_[module_name].is_stop_decision = false;
  }
}

bool PlanningFactorAccumulator::addMetricMsg(
  const Metric & metric, MetricArrayMsg & metrics_msg) const
{
  const std::string base_name = metric_to_str.at(metric) + "/";
  MetricMsg metric_msg;
  bool added = false;
  if (metric == Metric::stop_decision || metric == Metric::abnormal_stop_decision) {
    for (const auto & module : stop_decision_modules_) {
      auto & state = metric == Metric::stop_decision ? stop_decision_state_.at(module)
                                                     : abnormal_stop_decision_state_.at(module);
      if (state.is_stop_decision) {
        metric_msg.name = base_name + module + "/keep_duration";
        metric_msg.value = std::to_string(state.stop_decision_keep_time);
        metrics_msg.metric_array.push_back(metric_msg);

        metric_msg.name = base_name + module + "/distance_to_stop";
        metric_msg.value = std::to_string(state.distance_to_stop);
        metrics_msg.metric_array.push_back(metric_msg);
        added = true;
      }
    }
  }
  return added;
}

json PlanningFactorAccumulator::getOutputJson(const OutputMetric & output_metric)
{
  json j;

  const std::string base_name = output_metric_to_str.at(output_metric) + "/";
  if (
    output_metric == OutputMetric::stop_decision ||
    output_metric == OutputMetric::abnormal_stop_decision) {
    for (const auto & module : stop_decision_modules_) {
      auto & state = output_metric == OutputMetric::stop_decision
                       ? stop_decision_state_.at(module)
                       : abnormal_stop_decision_state_.at(module);
      // update the status and count for stop_decision
      if (state.is_stop_decision) {
        state.stop_decision_keep_time_accumulator.add(state.stop_decision_keep_time);
      }
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
