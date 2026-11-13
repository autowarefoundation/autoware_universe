// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY_RANKER__METRICS__STEERING_CONSISTENCY_METRIC_HPP_
#define AUTOWARE__TRAJECTORY_RANKER__METRICS__STEERING_CONSISTENCY_METRIC_HPP_

#include "autoware/trajectory_ranker/interface/metrics_interface.hpp"

#include <autoware_trajectory_ranker/autoware_trajectory_ranker_param.hpp>

#include <memory>
#include <vector>

namespace autoware::trajectory_ranker::metrics
{

class SteeringConsistency : public MetricInterface
{
public:
  SteeringConsistency() : MetricInterface("SteeringConsistency") {}

  void evaluate(
    const std::shared_ptr<autoware::trajectory_ranker::DataInterface> & result) const override;

  bool is_deviation() const override { return true; }  // Higher deviation in steering is worse

  void setup_parameters(const trajectory_ranker_params::Params::Evaluation & params) override;

  double weight() const override { return params_.weight; }
  std::vector<double> decay_weights() const override { return params_.decay_weight; }

private:
  trajectory_ranker_params::Params::Evaluation::SteeringConsistency params_;
};

}  // namespace autoware::trajectory_ranker::metrics

#endif  // AUTOWARE__TRAJECTORY_RANKER__METRICS__STEERING_CONSISTENCY_METRIC_HPP_
