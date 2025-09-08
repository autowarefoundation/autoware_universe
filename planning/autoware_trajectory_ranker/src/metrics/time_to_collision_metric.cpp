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

#include "autoware/trajectory_ranker/metrics/time_to_collision_metric.hpp"

#include "autoware/trajectory_ranker/metrics/metrics_utils.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace autoware::trajectory_ranker::metrics
{

void TimeToCollision::evaluate(
  const std::shared_ptr<autoware::trajectory_ranker::DataInterface> & result,
  const float max_value) const
{
  if (!result->points() || result->points()->empty()) {
    return;
  }

  constexpr float epsilon = 1.0e-3f;
  std::vector<float> ttc(result->points()->size(), 0.0f);
  if (max_value < epsilon) {
    result->set_metric(index(), ttc);
    return;
  }

  ttc.reserve(result->points()->size());
  for (size_t i = 0; i < result->points()->size(); i++) {
    ttc.at(i) = std::min(
      1.0f,
      utils::time_to_collision(result->points(), result->objects(), i, max_value) / max_value);
  }

  result->set_metric(index(), ttc);
}

}  // namespace autoware::trajectory_ranker::metrics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_ranker::metrics::TimeToCollision,
  autoware::trajectory_ranker::metrics::MetricInterface)
