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

#include "autoware/behavior_path_bidirectional_traffic_module/parameter.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/utils.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/utils/shift.hpp"

#include <autoware/trajectory/utils/find_intervals.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <limits>
#include <optional>
#include <vector>
namespace autoware::behavior_path_planner
{

experimental::trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
shift_trajectory_for_keep_left(
  const experimental::trajectory::Trajectory<
    autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const std::vector<ConnectedBidirectionalLanelets::SharedConstPtr> &
    all_connected_bidirectional_lanelets,
  const BidirectionalTrafficModuleParameters & parameter, const EgoParameters & ego_params,
  const std::function<void(std::string_view)> & logger)
{
  std::vector<experimental::trajectory::ShiftInterval> shifts_for_keep_left;

  for (const ConnectedBidirectionalLanelets::SharedConstPtr & bidirectional_lanelet :
       all_connected_bidirectional_lanelets) {
    // Get the overlap interval between the trajectory and the bidirectional lanelet
    std::optional<experimental::trajectory::Interval> overlap_interval =
      bidirectional_lanelet->get_overlap_interval(trajectory);
    if (!overlap_interval.has_value()) {  // No overlap
      continue;
    }

    const double keep_left_distance = -std::min(
      parameter.keep_left_distance_from_center_line,
      bidirectional_lanelet->average_lane_width() / 2.0 - ego_params.vehicle_width / 2.0 -
        parameter.min_distance_from_roadside);

    // Make a shift for entering the bidirectional lanelet
    lanelet::ConstLanelets lanelets_before_entering_bidirectional_lanelet = filter_lanelets_by_ids(
      bidirectional_lanelet->get_lanelets_before_entering(), trajectory.get_contained_lane_ids());

    double shift_start_s = 0.0;

    if (lanelets_before_entering_bidirectional_lanelet.size() > 1) {
      logger("The trajectory has multiple lanelets before entering the bidirectional lane");
    }
    if (lanelets_before_entering_bidirectional_lanelet.empty()) {
      shift_start_s = std::numeric_limits<double>::lowest();
    } else {
      shift_start_s =
        overlap_interval->start -
        compute_length_of_lanelets(lanelets_before_entering_bidirectional_lanelet.front());
    }
    shifts_for_keep_left.emplace_back(
      experimental::trajectory::ShiftInterval{
        shift_start_s, overlap_interval->start, keep_left_distance});

    lanelet::ConstLanelets lanelets_after_exiting_bidirectional_lanelet = filter_lanelets_by_ids(
      bidirectional_lanelet->get_lanelets_after_exiting(), trajectory.get_contained_lane_ids());

    // experimental::trajectory::ShiftInterval shift_for_exiting_bidirectional_lanelet;
    double shift_end_s = 0.0;

    if (lanelets_after_exiting_bidirectional_lanelet.size() > 1) {
      logger("The trajectory has multiple lanelets after exiting the bidirectional lane");
    }

    if (lanelets_after_exiting_bidirectional_lanelet.empty()) {
      shift_end_s = std::numeric_limits<double>::max();
    } else {
      shift_end_s = overlap_interval->end + compute_length_of_lanelets(
                                              lanelets_after_exiting_bidirectional_lanelet.front());
    }
    shifts_for_keep_left.emplace_back(
      experimental::trajectory::ShiftInterval{
        overlap_interval->end, shift_end_s, -keep_left_distance});
  }

  auto shifted_trajectory =
    experimental::trajectory::shift(trajectory, shifts_for_keep_left, {0.0, 0.0});

  if (!shifted_trajectory) {
    logger(shifted_trajectory.error().what);
    return trajectory;
  }
  return shifted_trajectory.value();
}
}  // namespace autoware::behavior_path_planner
