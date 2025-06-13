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

#ifndef AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__BLINKER_ACCUMULATOR_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__BLINKER_ACCUMULATOR_HPP_

#include "autoware/planning_evaluator/metrics/metric.hpp"
#include "autoware/planning_evaluator/metrics/output_metric.hpp"

#include <autoware_utils/math/accumulator.hpp>
#include <nlohmann/json.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include <cstdint>
#include <deque>

namespace planning_diagnostics
{
using autoware_utils::Accumulator;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using MetricMsg = tier4_metric_msgs::msg::Metric;
using MetricArrayMsg = tier4_metric_msgs::msg::MetricArray;
using json = nlohmann::json;

/**
 * @class BlinkerAccumulator
 * @brief Accumulator to generate blinker-related metrics from TurnIndicatorsReport messages.
 */
class BlinkerAccumulator
{
public:
  struct Parameters
  {
    double window_duration_s = 5.0;  // [s] Duration to count blinker changes for publishing
  } parameters;

  BlinkerAccumulator() = default;
  ~BlinkerAccumulator() = default;

  /**
   * @brief update the accumulator with new blinker data
   * @param msg new TurnIndicatorsReport msg to update the accumulator state
   */
  void update(const TurnIndicatorsReport & msg);

  /**
   * @brief get the output json data for the OutputMetric
   * @return json data
   */
  json getOutputJson(const OutputMetric & output_metric) const;

  /**
   * @brief Add metrics to MetricArray for the given Metric
   * @param metric Metric to add to the MetricArrayMsg
   * @param metrics_msg MetricArrayMsg to add the metric to
   */
  bool addMetricMsg(const Metric & metric, MetricArrayMsg & metrics_msg) const;

private:
  // statistics
  uint8_t prev_blinker_ = TurnIndicatorsReport::DISABLE;
  size_t blinker_change_count_total_ = 0;
  size_t blinker_change_count_in_window_ = 0;
  std::deque<double> blinker_change_window_;
  Accumulator<double> blinker_change_count_accumulator_;
};

}  // namespace planning_diagnostics

#endif  // AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__BLINKER_ACCUMULATOR_HPP_
