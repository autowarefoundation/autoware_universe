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

#ifndef TRAJECTORY_LATCHER_HPP_
#define TRAJECTORY_LATCHER_HPP_

#include "stop_profile.hpp"
#include "type_alias.hpp"

#include <array>
#include <optional>

namespace autoware::in_lane_mrm_planner
{

// Holds the latest candidate trajectory of every deceleration profile and freezes one of them
// while the in-lane stop trigger is active.
class TrajectoryLatcher
{
public:
  void update_candidate(StopProfile profile, const Trajectory & candidate);

  // Latches the latest candidate of `profile`. Returns false (and keeps the current state) if
  // no candidate of that profile is stored.
  bool latch(StopProfile profile);

  void unlatch();

  bool is_latched() const;
  std::optional<StopProfile> latched_profile() const;
  bool has_candidate(StopProfile profile) const;
  // True if a candidate of the standby profile (published while unlatched) is stored.
  bool has_latest_candidate() const;

  // Latched trajectory while latched, otherwise the latest candidate of the standby profile.
  std::optional<Trajectory> output() const;

private:
  std::optional<StopProfile> latched_profile_;
  Trajectory latched_traj_;
  std::array<std::optional<Trajectory>, kNumStopProfiles> latest_candidates_;
};

}  // namespace autoware::in_lane_mrm_planner

#endif  // TRAJECTORY_LATCHER_HPP_
