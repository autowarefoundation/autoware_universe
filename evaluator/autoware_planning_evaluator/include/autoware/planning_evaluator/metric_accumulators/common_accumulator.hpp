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

#ifndef AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__COMMON_ACCUMULATOR_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__COMMON_ACCUMULATOR_HPP_

#include "autoware/planning_evaluator/metrics/output_metric.hpp"

#include <autoware_utils/math/accumulator.hpp>
#include <nlohmann/json.hpp>

namespace planning_diagnostics
{
using autoware_utils::Accumulator;
using json = nlohmann::json;

/**
 * @class CommonAccumulator
 * @brief Accumulator to generate OutputMetric json result from normal Metric.
 */
class CommonAccumulator
{
public:
  CommonAccumulator() = default;
  ~CommonAccumulator() = default;

  /**
   * @brief update the accumulator with new statistics-based metrics with count of data points
   * @param accumulator new metric data to add to the accumulator
   * @param count number of data points to count
   */
  void update(const Accumulator<double> & accumulator, const unsigned int count);

  /**
   * @brief update the accumulator with new statistics-based metrics with count of update times
   * @param accumulator new metric data to add to the accumulator
   */
  void update(const Accumulator<double> & accumulator) { update(accumulator, 1); }

  /**
   * @brief update the accumulator with new value-based metrics
   * @param value new metric data to add to the accumulator
   */
  void update(const double value);

  /**
   * @brief get the output json data for the OutputMetric
   * @return json data
   */
  json getOutputJson(const OutputMetric & output_metric) const;

private:
  Accumulator<double> min_accumulator_;
  Accumulator<double> max_accumulator_;
  Accumulator<long double> mean_accumulator_;
  unsigned int count_ = 0;
};

}  // namespace planning_diagnostics

#endif  // AUTOWARE__PLANNING_EVALUATOR__METRIC_ACCUMULATORS__COMMON_ACCUMULATOR_HPP_
