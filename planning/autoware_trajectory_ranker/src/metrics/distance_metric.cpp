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

#include "autoware/trajectory_ranker/metrics/distance_metric.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace autoware::trajectory_ranker::metrics
{

void TravelDistance::evaluate(
  const std::shared_ptr<autoware::trajectory_ranker::DataInterface> & result,
  const float max_value) const
{
  const auto points = result->points();
  if (!points || points->empty()) {
    return;
  }

  constexpr float epsilon = 1.0e-3f;
  std::vector<float> distances(points->size(), 0.0f);
  if (max_value < epsilon) {
    result->set_metric(index(), distances);
    return;
  }

  for (size_t i = 0; i < result->points()->size(); i++) {
    distances.at(i) = std::min(
      1.0f,
      static_cast<float>(autoware::motion_utils::calcSignedArcLength(*result->points(), 0L, i)) /
        max_value);
  }

  result->set_metric(index(), distances);
}

}  // namespace autoware::trajectory_ranker::metrics

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_ranker::metrics::TravelDistance,
  autoware::trajectory_ranker::metrics::MetricInterface)
