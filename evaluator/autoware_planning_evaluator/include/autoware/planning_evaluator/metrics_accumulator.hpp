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

#ifndef AUTOWARE__PLANNING_EVALUATOR__METRICS_ACCUMULATOR_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__METRICS_ACCUMULATOR_HPP_

#include "autoware/planning_evaluator/metric_accumulators/blinker_accumulator.hpp"
#include "autoware/planning_evaluator/metric_accumulators/common_accumulator.hpp"
#include "autoware/planning_evaluator/metric_accumulators/planning_factor_accumulator.hpp"
#include "autoware/planning_evaluator/metric_accumulators/steer_accumulator.hpp"
#include "autoware/planning_evaluator/metrics/metric.hpp"
#include "autoware/planning_evaluator/metrics/output_metric.hpp"
#include "metrics/metric.hpp"

#include <autoware_utils/math/accumulator.hpp>
#include <nlohmann/json.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace planning_diagnostics
{
using autoware_utils::Accumulator;
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using json = nlohmann::json;

class MetricsAccumulator
{
public:
  void accumulate(const OutputMetric metric, const double value);

  void accumulate(
    const OutputMetric metric, const Accumulator<double> & accumulator, const unsigned int count);

  void accumulate(const OutputMetric metric, const Accumulator<double> & accumulator);

  void setEgoPose(const nav_msgs::msg::Odometry & ego_odometry);

  void setBlinkerData(const TurnIndicatorsReport & msg);

  void setSteerData(const SteeringReport & msg);

  void setPlanningFactors(
    const std::string & module_name, const PlanningFactorArray & planning_factors);

  void addMetricMsg(const Metric & metric, MetricArrayMsg & metrics_msg) const;

  void addMetricMsg(
    const Metric & metric, MetricArrayMsg & metrics_msg, const std::string & module_name);

  json getOutputJson(const OutputMetric & output_metric);

  PlanningFactorAccumulator planning_factor_accumulator;
  BlinkerAccumulator blinker_accumulator;
  SteerAccumulator steer_accumulator;
  std::unordered_map<OutputMetric, CommonAccumulator> common_accumulators;

private:
  nav_msgs::msg::Odometry ego_odometry_;

};  // class MetricsAccumulator

}  // namespace planning_diagnostics

#endif  // AUTOWARE__PLANNING_EVALUATOR__METRICS_ACCUMULATOR_HPP_
