// Copyright 2026 TIER IV, Inc.
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

#include "trajectory_sanitizer.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <utility>

namespace autoware::in_lane_mrm_planner
{

size_t remove_overlap_points(TrajectoryPoints & points, const double min_interval)
{
  if (points.size() < 2) {
    return 0;
  }

  TrajectoryPoints kept;
  kept.reserve(points.size());
  kept.push_back(points.front());
  for (size_t i = 1; i < points.size(); ++i) {
    const auto & candidate = points.at(i);
    if (autoware_utils::calc_distance2d(kept.back(), candidate) < min_interval) {
      kept.back().longitudinal_velocity_mps =
        std::min(kept.back().longitudinal_velocity_mps, candidate.longitudinal_velocity_mps);
      continue;
    }
    kept.push_back(candidate);
  }

  const size_t removed = points.size() - kept.size();
  points = std::move(kept);
  return removed;
}

}  // namespace autoware::in_lane_mrm_planner
