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

#include "autoware/planning_evaluator/metric_accumulators/common_accumulator.hpp"

namespace planning_diagnostics
{

void CommonAccumulator::update(const Accumulator<double> & accumulator, const unsigned int count)
{
  min_accumulator_.add(accumulator.min());
  max_accumulator_.add(accumulator.max());
  mean_accumulator_.add(accumulator.mean());
  count_ += count;
}

void CommonAccumulator::update(const double value)
{
  min_accumulator_.add(value);
  max_accumulator_.add(value);
  mean_accumulator_.add(value);
  count_ += 1;
}

json CommonAccumulator::getOutputJson(const OutputMetric & output_metric) const
{
  json j;
  j["min"] = min_accumulator_.min();
  j["max"] = max_accumulator_.max();
  j["mean"] = mean_accumulator_.mean();
  j["count"] = count_;
  j["description"] = output_metric_descriptions.at(output_metric);
  return j;
}

}  // namespace planning_diagnostics
