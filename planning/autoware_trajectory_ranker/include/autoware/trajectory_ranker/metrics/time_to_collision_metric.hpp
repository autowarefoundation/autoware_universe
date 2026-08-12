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

#ifndef AUTOWARE__TRAJECTORY_RANKER__METRICS__TIME_TO_COLLISION_METRIC_HPP_
#define AUTOWARE__TRAJECTORY_RANKER__METRICS__TIME_TO_COLLISION_METRIC_HPP_

#include "autoware/trajectory_ranker/interface/metrics_interface.hpp"

#include <memory>

namespace autoware::trajectory_ranker::metrics
{

class TimeToCollision : public MetricInterface
{
public:
  TimeToCollision() : MetricInterface("TimeToCollision") {}

  void evaluate(
    const std::shared_ptr<autoware::trajectory_ranker::DataInterface> & result,
    const float max_value) const override;

  bool is_deviation() const override { return false; }
};

}  // namespace autoware::trajectory_ranker::metrics

#endif  // AUTOWARE__TRAJECTORY_RANKER__METRICS__TIME_TO_COLLISION_METRIC_HPP_
