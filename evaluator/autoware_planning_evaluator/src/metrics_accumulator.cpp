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

#include "autoware/planning_evaluator/metrics_accumulator.hpp"

namespace planning_diagnostics
{
void MetricsAccumulator::accumulate(const OutputMetric output_metric, const double value)
{
  common_accumulators.emplace(output_metric, CommonAccumulator());
  common_accumulators[output_metric].update(value);
}

void MetricsAccumulator::accumulate(
  const OutputMetric output_metric, const Accumulator<double> & accumulator,
  const unsigned int count)
{
  common_accumulators.emplace(output_metric, CommonAccumulator());
  common_accumulators[output_metric].update(accumulator, count);
}

void MetricsAccumulator::accumulate(
  const OutputMetric output_metric, const Accumulator<double> & accumulator)
{
  common_accumulators.emplace(output_metric, CommonAccumulator());
  common_accumulators[output_metric].update(accumulator);
}

void MetricsAccumulator::setEgoPose(const nav_msgs::msg::Odometry & ego_odometry)
{
  ego_odometry_ = ego_odometry;
}

void MetricsAccumulator::setBlinkerData(const TurnIndicatorsReport & msg)
{
  blinker_accumulator.update(msg);
}

void MetricsAccumulator::setSteerData(const SteeringReport & msg)
{
  steer_accumulator.update(msg);
}

void MetricsAccumulator::setPlanningFactors(
  const std::string & module_name, const PlanningFactorArray & planning_factors)
{
  planning_factor_accumulator.update(module_name, planning_factors, ego_odometry_);
}

void MetricsAccumulator::addMetricMsg(const Metric & metric, MetricArrayMsg & metrics_msg) const
{
  switch (metric) {
    case Metric::blinker_change_count:
      blinker_accumulator.addMetricMsg(metric, metrics_msg);
      return;
    case Metric::steer_change_count:
      steer_accumulator.addMetricMsg(metric, metrics_msg);
      return;
    default:
      return;
  }
}

void MetricsAccumulator::addMetricMsg(
  const Metric & metric, MetricArrayMsg & metrics_msg, const std::string & module_name)
{
  switch (metric) {
    case Metric::stop_decision:
    case Metric::abnormal_stop_decision:
      planning_factor_accumulator.addMetricMsg(metric, metrics_msg, module_name);
      return;
    default:
      return;
  }
}

json MetricsAccumulator::getOutputJson(const OutputMetric & output_metric)
{
  switch (output_metric) {
    case OutputMetric::blinker_change_count:
      return blinker_accumulator.getOutputJson(output_metric);
    case OutputMetric::steer_change_count:
      return steer_accumulator.getOutputJson(output_metric);

    case OutputMetric::stop_decision:
    case OutputMetric::abnormal_stop_decision:
      return planning_factor_accumulator.getOutputJson(output_metric);

    default:
      if (common_accumulators.find(output_metric) == common_accumulators.end()) {
        return {};
      } else {
        return common_accumulators[output_metric].getOutputJson(output_metric);
      }
  }
}

}  // namespace planning_diagnostics
