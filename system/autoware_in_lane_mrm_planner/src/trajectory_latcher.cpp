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

#include "trajectory_latcher.hpp"

namespace autoware::in_lane_mrm_planner
{

void TrajectoryLatcher::update_candidate(const Trajectory & candidate)
{
  if (candidate.points.empty()) {
    return;
  }
  latest_candidate_ = candidate;
}

void TrajectoryLatcher::latch()
{
  if (!latest_candidate_.has_value()) {
    return;
  }
  latched_traj_ = latest_candidate_.value();
  latched_ = true;
}

void TrajectoryLatcher::unlatch()
{
  latched_ = false;
}

bool TrajectoryLatcher::is_latched() const
{
  return latched_;
}

bool TrajectoryLatcher::has_latest_candidate() const
{
  return latest_candidate_.has_value();
}

std::optional<Trajectory> TrajectoryLatcher::output() const
{
  if (latched_) {
    return latched_traj_;
  }
  return latest_candidate_;
}

}  // namespace autoware::in_lane_mrm_planner
