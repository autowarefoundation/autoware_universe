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

#include "type_alias.hpp"

#include <optional>

namespace autoware::in_lane_mrm_planner
{

class TrajectoryLatcher
{
public:
  void update_candidate(const Trajectory & candidate);

  void latch();

  void unlatch();

  bool is_latched() const;
  bool has_latest_candidate() const;

  std::optional<Trajectory> output() const;

private:
  bool latched_{false};
  Trajectory latched_traj_;
  std::optional<Trajectory> latest_candidate_;
};

}  // namespace autoware::in_lane_mrm_planner

#endif  // TRAJECTORY_LATCHER_HPP_
