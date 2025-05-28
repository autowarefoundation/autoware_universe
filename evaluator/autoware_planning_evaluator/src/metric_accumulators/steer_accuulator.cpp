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

#include "autoware/planning_evaluator/metric_accumulators/steer_accumulator.hpp"

namespace planning_diagnostics
{

void SteerAccumulator::update(const SteeringReport & msg)
{
  const double cur_steer_angle = msg.steering_tire_angle;
  const auto cur_t =
    static_cast<double>(msg.stamp.sec) + static_cast<double>(msg.stamp.nanosec) * 1e-9;

  if (!steer_state_.has_steer_angle()) {
    steer_state_.steer_angle = cur_steer_angle;
    steer_state_.timestamp = cur_t;
    return;
  }

  const double dt = cur_t - steer_state_.timestamp;
  const double cur_steer_rate = (cur_steer_angle - steer_state_.steer_angle) / dt;
  const SteerRateState cur_steer_rate_state =
    std::abs(cur_steer_rate) < parameters.steer_rate_margin
      ? SteerRateState::AROUND_ZERO
      : (cur_steer_rate > 0 ? SteerRateState::POSITIVE : SteerRateState::NEGATIVE);

  if (!steer_state_.has_steer_rate()) {
    steer_state_.steer_rate = cur_steer_rate;
    steer_state_.steer_rate_state = cur_steer_rate_state;
    steer_state_.timestamp = cur_t;
    return;
  }

  if (
    cur_steer_rate_state != steer_state_.steer_rate_state &&
    cur_steer_rate_state != SteerRateState::AROUND_ZERO) {
    steer_rate_change_window_.push_back(cur_t);
    steer_rate_change_count_total_ += 1;
  }

  while (!steer_rate_change_window_.empty() &&
         cur_t - steer_rate_change_window_.front() > parameters.window_duration_s) {
    steer_rate_change_window_.pop_front();
  }
  steer_rate_change_count_in_window_ = steer_rate_change_window_.size();
  steer_rate_change_count_accumulator_.add(static_cast<double>(steer_rate_change_count_in_window_));

  steer_state_.steer_angle = cur_steer_angle;
  steer_state_.steer_rate = cur_steer_rate;
  steer_state_.steer_rate_state = cur_steer_rate_state;
  steer_state_.timestamp = cur_t;
}

bool SteerAccumulator::addMetricMsg(const Metric & metric, MetricArrayMsg & metrics_msg) const
{
  if (metric == Metric::steer_change_count) {
    const std::string base_name = metric_to_str.at(metric) + "/";
    MetricMsg metric_msg;
    metric_msg.name = base_name + "count_in_duration";
    metric_msg.value = std::to_string(steer_rate_change_count_in_window_);
    metrics_msg.metric_array.push_back(metric_msg);
    return true;
  }
  return false;
}

json SteerAccumulator::getOutputJson(const OutputMetric & output_metric) const
{
  json j;
  if (output_metric == OutputMetric::steer_change_count) {
    j["count"] = steer_rate_change_count_total_;
    j["count_in_duration/min"] = steer_rate_change_count_accumulator_.min();
    j["count_in_duration/max"] = steer_rate_change_count_accumulator_.max();
    j["count_in_duration/mean"] = steer_rate_change_count_accumulator_.mean();
    j["parameter/window_duration_s"] = parameters.window_duration_s;
    j["description"] = output_metric_descriptions.at(output_metric);
  }
  return j;
}

}  // namespace planning_diagnostics
