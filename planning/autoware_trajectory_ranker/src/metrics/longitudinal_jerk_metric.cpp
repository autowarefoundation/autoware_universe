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

#include "autoware/trajectory_ranker/metrics/longitudinal_jerk_metric.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace autoware::trajectory_ranker::metrics
{

void LongitudinalJerk::evaluate(
  const std::shared_ptr<autoware::trajectory_ranker::DataInterface> & result,
  const float max_value) const
{
  const auto points = result->points();
  if (!points || points->size() < 2) {
    return;
  }

  std::vector<float> jerk(points->size(), 0.0f);

  constexpr float epsilon = 1.0e-3f;
  const float time_resolution = resolution() > epsilon ? resolution() : epsilon;

  if (max_value < epsilon) {
    result->set_metric(index(), jerk);
    return;
  }

  std::vector<float> acceleration;
  acceleration.reserve(points->size());
  for (size_t i = 0; i < points->size() - 1; i++) {
    acceleration.push_back(
      (points->at(i + 1).longitudinal_velocity_mps - points->at(i).longitudinal_velocity_mps) /
      time_resolution);
  }
  acceleration.push_back(acceleration.back());

  for (size_t i = 0; i < acceleration.size() - 1; i++) {
    const auto calculated_jerk = (acceleration.at(i + 1) - acceleration.at(i)) / time_resolution;
    jerk.at(i) = std::min(1.0f, static_cast<float>(std::abs(calculated_jerk)) / max_value);
  }
  jerk.back() = jerk.at(jerk.size() - 2);

  result->set_metric(index(), jerk);
}

}  // namespace autoware::trajectory_ranker::metrics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_ranker::metrics::LongitudinalJerk,
  autoware::trajectory_ranker::metrics::MetricInterface)
