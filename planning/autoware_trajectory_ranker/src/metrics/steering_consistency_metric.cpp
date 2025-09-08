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

#include "autoware/trajectory_ranker/metrics/steering_consistency_metric.hpp"

#include "autoware/trajectory_ranker/metrics/metrics_utils.hpp"

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace autoware::trajectory_ranker::metrics
{

void SteeringConsistency::evaluate(
  const std::shared_ptr<autoware::trajectory_ranker::DataInterface> & result,
  const float max_value) const
{
  if (result->previous() == nullptr) return;
  if (!result->points() || result->points()->size() < 2) return;

  constexpr float epsilon = 1.0e-3f;
  std::vector<float> steering_command(result->points()->size(), 0.0f);
  if (max_value < epsilon) {
    result->set_metric(index(), steering_command);
    return;
  }

  const auto wheel_base = vehicle_info()->wheel_base_m;
  for (size_t i = 0; i < result->points()->size(); i++) {
    const auto & point = result->points()->at(i);
    const auto current = utils::steer_command(result->points(), point.pose, wheel_base);
    const auto previous = utils::steer_command(result->previous(), point.pose, wheel_base);

    steering_command.at(i) =
      std::min(1.0f, static_cast<float>(std::abs(current - previous)) / max_value);
  }

  result->set_metric(index(), steering_command);
}

}  // namespace autoware::trajectory_ranker::metrics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_ranker::metrics::SteeringConsistency,
  autoware::trajectory_ranker::metrics::MetricInterface)
