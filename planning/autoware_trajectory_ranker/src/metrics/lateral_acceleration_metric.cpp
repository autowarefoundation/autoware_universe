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

#include "autoware/trajectory_ranker/metrics/lateral_acceleration_metric.hpp"

#include <tf2/utils.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace autoware::trajectory_ranker::metrics
{

void LateralAcceleration::evaluate(
  const std::shared_ptr<autoware::trajectory_ranker::DataInterface> & result,
  const float max_value) const
{
  if (!result->points() || result->points()->size() < 2) return;

  std::vector<float> lateral_accelerations(result->points()->size(), 0.0f);
  constexpr float epsilon = 1.0e-3f;
  const float time_resolution = resolution() > epsilon ? resolution() : epsilon;

  if (max_value < epsilon) {
    result->set_metric(index(), lateral_accelerations);
    return;
  }

  lateral_accelerations.reserve(result->points()->size());
  for (size_t i = 0; i < result->points()->size() - 1; i++) {
    const auto lateral_acc = (result->points()->at(i + 1).lateral_velocity_mps -
                              result->points()->at(i).lateral_velocity_mps) /
                             time_resolution;
    lateral_accelerations.at(i) =
      std::min(1.0f, static_cast<float>(std::abs(lateral_acc)) / max_value);
  }
  lateral_accelerations.back() = lateral_accelerations.at(lateral_accelerations.size() - 2);

  result->set_metric(index(), lateral_accelerations);
}

}  // namespace autoware::trajectory_ranker::metrics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_ranker::metrics::LateralAcceleration,
  autoware::trajectory_ranker::metrics::MetricInterface)
