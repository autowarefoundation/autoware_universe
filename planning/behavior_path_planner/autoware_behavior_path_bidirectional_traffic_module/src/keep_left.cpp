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

#include "autoware/behavior_path_bidirectional_traffic_module/utils.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/utils/shift.hpp"

#include <autoware/trajectory/utils/find_intervals.hpp>

#include <lanelet2_core/Forward.h>

#include <limits>
#include <optional>
#include <vector>
namespace autoware::behavior_path_planner
{

trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
shift_trajectory_for_keep_left(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<ConnectedBidirectionalLanelets> & all_connected_bidirectional_lanelets,
  const double & keep_left_ratio, const double & vehicle_width)
{
  std::vector<trajectory::ShiftInterval> intervals_to_shift;

  for (const auto & bidirectional_lane : all_connected_bidirectional_lanelets) {
    std::optional<trajectory::Interval> interval =
      bidirectional_lane.get_overlap_interval(trajectory);
    if (!interval) {
      continue;
    }

    trajectory::ShiftInterval shift_for_enter_bidirectional_lane;

    lanelet::Ids lane_ids_in_trajectory = get_lane_ids(trajectory);
    std::optional<lanelet::ConstLanelet> lanelet_before_bidirectional_lanes;
    for (const auto & lanelet : bidirectional_lane.get_lanelets_before_bidirectional_lanes()) {
      if (
        std::find(lane_ids_in_trajectory.begin(), lane_ids_in_trajectory.end(), lanelet.id()) !=
        lane_ids_in_trajectory.end()) {
        lanelet_before_bidirectional_lanes = lanelet;
        break;
      }
    }

    if (lanelet_before_bidirectional_lanes) {
      shift_for_enter_bidirectional_lane.start =
        interval->start - compute_length_of_lanelets(*lanelet_before_bidirectional_lanes);
      shift_for_enter_bidirectional_lane.end = interval->start;
      shift_for_enter_bidirectional_lane.lateral_offset =
        -keep_left_ratio * (bidirectional_lane.average_lane_width() / 2.0 - vehicle_width / 2.0);
      intervals_to_shift.emplace_back(shift_for_enter_bidirectional_lane);
    } else {
      shift_for_enter_bidirectional_lane.start = std::numeric_limits<double>::lowest();
      shift_for_enter_bidirectional_lane.end = interval->start;
      shift_for_enter_bidirectional_lane.lateral_offset =
        -keep_left_ratio * (bidirectional_lane.average_lane_width() / 2.0 - vehicle_width / 2.0);
      intervals_to_shift.emplace_back(shift_for_enter_bidirectional_lane);
    }

    trajectory::ShiftInterval shift_for_exit_bidirectional_lane;

    std::optional<lanelet::ConstLanelet> lanelet_after_bidirectional_lanes;
    for (const auto & lanelet : bidirectional_lane.get_lanelets_after_bidirectional_lanes()) {
      if (
        std::find(lane_ids_in_trajectory.begin(), lane_ids_in_trajectory.end(), lanelet.id()) !=
        lane_ids_in_trajectory.end()) {
        lanelet_after_bidirectional_lanes = lanelet;
        break;
      }
    }

    if (lanelet_after_bidirectional_lanes) {
      shift_for_exit_bidirectional_lane.start = interval->end;
      shift_for_exit_bidirectional_lane.end =
        interval->end + compute_length_of_lanelets(*lanelet_after_bidirectional_lanes);
      shift_for_exit_bidirectional_lane.lateral_offset =
        keep_left_ratio * (bidirectional_lane.average_lane_width() / 2.0 - vehicle_width / 2.0);
      intervals_to_shift.emplace_back(shift_for_exit_bidirectional_lane);
    } else {
      shift_for_exit_bidirectional_lane.start = interval->end;
      shift_for_exit_bidirectional_lane.end = std::numeric_limits<double>::max();
      shift_for_exit_bidirectional_lane.lateral_offset =
        keep_left_ratio * (bidirectional_lane.average_lane_width() / 2.0 - vehicle_width / 2.0);
      intervals_to_shift.emplace_back(shift_for_exit_bidirectional_lane);
    }
  }

  auto shifted_trajectory = trajectory::shift(trajectory, intervals_to_shift);
  return shifted_trajectory;
}
}  // namespace autoware::behavior_path_planner
