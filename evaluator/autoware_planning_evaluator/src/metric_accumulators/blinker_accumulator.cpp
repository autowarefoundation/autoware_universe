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

#include "autoware/planning_evaluator/metric_accumulators/blinker_accumulator.hpp"

namespace planning_diagnostics
{

void BlinkerAccumulator::update(const TurnIndicatorsReport & msg)
{
  const auto cur_blinker = msg.report;
  const auto cur_t =
    static_cast<double>(msg.stamp.sec) + static_cast<double>(msg.stamp.nanosec) * 1e-9;
  if (cur_blinker != prev_blinker_ && cur_blinker != TurnIndicatorsReport::DISABLE) {
    blinker_change_window_.push_back(cur_t);
    blinker_change_count_total_ += 1;
  }

  while (!blinker_change_window_.empty() &&
         cur_t - blinker_change_window_.front() > parameters.window_duration_s) {
    blinker_change_window_.pop_front();
  }
  blinker_change_count_in_window_ = blinker_change_window_.size();
  blinker_change_count_accumulator_.add(static_cast<double>(blinker_change_count_in_window_));
  prev_blinker_ = cur_blinker;
}

bool BlinkerAccumulator::addMetricMsg(const Metric & metric, MetricArrayMsg & metrics_msg) const
{
  if (metric == Metric::blinker_change_count) {
    const std::string base_name = metric_to_str.at(metric) + "/";
    MetricMsg metric_msg;
    metric_msg.name = base_name + "count_in_duration";
    metric_msg.value = std::to_string(blinker_change_count_in_window_);
    metrics_msg.metric_array.push_back(metric_msg);
    return true;
  }
  return false;
}

json BlinkerAccumulator::getOutputJson(const OutputMetric & output_metric) const
{
  json j;
  if (output_metric == OutputMetric::blinker_change_count) {
    j["count"] = blinker_change_count_total_;
    j["count_in_duration/min"] = blinker_change_count_accumulator_.min();
    j["count_in_duration/max"] = blinker_change_count_accumulator_.max();
    j["count_in_duration/mean"] = blinker_change_count_accumulator_.mean();
    j["parameter/window_duration_s"] = parameters.window_duration_s;
    j["description"] = output_metric_descriptions.at(output_metric);
  }
  return j;
}
}  // namespace planning_diagnostics
