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

#ifndef DATA_STRUCTS_HPP_
#define DATA_STRUCTS_HPP_

#include "type_alias.hpp"

#include <vector>

namespace autoware::motion_velocity_planner::experimental
{
struct DepartureInterval
{
  TrajectoryPoint start;
  TrajectoryPoint end;
  SideKey side_key;
  double start_dist_on_traj;
  double end_dist_on_traj;

  bool start_at_traj_front{false};

  DeparturePoints candidates;
};
using DepartureIntervals = std::vector<DepartureInterval>;

}  // namespace autoware::motion_velocity_planner::experimental

#endif  // DATA_STRUCTS_HPP_
