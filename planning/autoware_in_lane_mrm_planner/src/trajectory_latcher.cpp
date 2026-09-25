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

void TrajectoryLatcher::update_candidate(const StopProfile profile, const Trajectory & candidate)
{
  if (candidate.points.empty()) {
    return;
  }
  latest_candidates_.at(to_index(profile)) = candidate;
}

bool TrajectoryLatcher::latch(const StopProfile profile)
{
  const auto & candidate = latest_candidates_.at(to_index(profile));
  if (!candidate.has_value()) {
    return false;
  }
  latched_traj_ = candidate.value();
  latched_profile_ = profile;
  return true;
}

void TrajectoryLatcher::unlatch()
{
  latched_profile_.reset();
}

bool TrajectoryLatcher::is_latched() const
{
  return latched_profile_.has_value();
}

std::optional<StopProfile> TrajectoryLatcher::latched_profile() const
{
  return latched_profile_;
}

bool TrajectoryLatcher::has_candidate(const StopProfile profile) const
{
  return latest_candidates_.at(to_index(profile)).has_value();
}

bool TrajectoryLatcher::has_latest_candidate() const
{
  return has_candidate(kStandbyStopProfile);
}

std::optional<Trajectory> TrajectoryLatcher::output() const
{
  if (latched_profile_) {
    return latched_traj_;
  }
  return latest_candidates_.at(to_index(kStandbyStopProfile));
}

}  // namespace autoware::in_lane_mrm_planner
