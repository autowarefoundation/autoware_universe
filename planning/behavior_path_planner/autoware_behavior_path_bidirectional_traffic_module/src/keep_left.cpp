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

#include "autoware/behavior_path_bidirectional_traffic_module/keep_left.hpp"

#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/utils/shift.hpp"

#include <vector>
namespace autoware::behavior_path_planner
{

trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
shift_trajectory_for_keep_left(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<trajectory::Interval> & bidirectional_lane_intervals_in_trajectory,
  const double & keep_left_length_from_center,
  const double & distance_to_shift_for_enter_bidirectional_lane,
  const double & distance_to_shift_for_exit_bidirectional_lane)
{
  std::vector<trajectory::ShiftInterval> intervals_to_shift;
  for (const auto & interval : bidirectional_lane_intervals_in_trajectory) {
    trajectory::ShiftInterval shift_for_enter;
    shift_for_enter.start = interval.start - distance_to_shift_for_enter_bidirectional_lane;
    shift_for_enter.end = interval.start;
    shift_for_enter.lateral_offset = -keep_left_length_from_center;
    trajectory::ShiftInterval shift_for_exit;
    shift_for_exit.start = interval.end;
    shift_for_exit.end = interval.end + distance_to_shift_for_exit_bidirectional_lane;
    shift_for_exit.lateral_offset = keep_left_length_from_center;

    intervals_to_shift.emplace_back(shift_for_enter);
    intervals_to_shift.emplace_back(shift_for_exit);
  }

  auto shifted_trajectory = trajectory::shift(trajectory, intervals_to_shift);

  return shifted_trajectory;
}
}  // namespace autoware::behavior_path_planner
