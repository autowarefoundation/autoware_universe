// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_goal_planner_module/lane_change.hpp"

#include "autoware/behavior_path_goal_planner_module/util.hpp"

#include <vector>

namespace autoware::behavior_path_planner
{

LaneChangeContext::State LaneChangeContext::get_next_state(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapConstPtr lanelet_map,
  lanelet::routing::RoutingGraphConstPtr routing_graph, const rclcpp::Time & now,
  const lanelet::Id goal_lanelet_id) const
{
  if (is_state<Completed>()) {
    return state_;
  }

  const auto lane_change_complete_lane =
    goal_planner_utils::find_last_lane_change_completed_lanelet(path, lanelet_map, routing_graph);
  const auto is_lane_changing_path = lane_change_complete_lane.has_value();
  if (is_state<Started>() || is_state<Executing>()) {
    if (is_lane_changing_path) {
      // NOTE(soblin): is it possible that lane_change_complete_lane changes during execution ?
      return is_state<Started>() ? Executing(std::get<Started>(state_)) : state_;
    }
    std::vector<lanelet::Id> lane_ids;
    for (const auto & point : path.points) {
      for (const auto & lane_id : point.lane_ids) {
        if (std::find(lane_ids.begin(), lane_ids.end(), lane_id) == lane_ids.end()) {
          lane_ids.push_back(lane_id);
        }
      }
    }
    if (lane_ids.empty()) {
      return Aborted{};
    }
    const auto current_target_lane_id = is_state<Started>()
                                          ? std::get<Started>(state_).complete_lane()
                                          : std::get<Executing>(state_).complete_lane();
    if (
      lane_ids.front() == current_target_lane_id &&
      std::find(lane_ids.begin(), lane_ids.end(), goal_lanelet_id) != lane_ids.end()) {
      const auto & exec =
        is_state<Started>() ? Executing{std::get<Started>(state_)} : std::get<Executing>(state_);
      return Completed{exec};
    }
    return Aborted{};
  }

  // Aborted or NotLaneChanging
  return is_lane_changing_path ? State{Started{lane_change_complete_lane.value().id(), now}}
                               : State{NotLaneChanging{}};
}

bool LaneChangeContext::is_not_consistent_transition(const State & from, const State & to)
{
  if (is_state<Started>(to) || is_state<Aborted>(to)) {
    return true;
  }
  if (is_state<Executing>(to)) {
    if (is_state<Aborted>(from) || is_state<NotLaneChanging>(from)) {
      return true;
    }
    if (is_state<Started>(from) || is_state<Executing>(from)) {
      const auto lane_change_target_to = std::get<Executing>(to).complete_lane();
      const auto lane_change_target_from = is_state<Started>(from)
                                             ? std::get<Started>(from).complete_lane()
                                             : std::get<Executing>(from).complete_lane();
      if (lane_change_target_to != lane_change_target_from) {
        return true;
      }
      const auto & from_start_time = is_state<Started>(from)
                                       ? std::get<Started>(from).start_time()
                                       : std::get<Executing>(from).start_time();
      const auto & to_start_time = std::get<Executing>(to).start_time();
      // if timestamp of `to` is larger than `from`, `to` is another lane change after `from`
      return (to_start_time - from_start_time).seconds() > 0.0;
    }
  }
  if (is_state<NotLaneChanging>(to) && !is_state<NotLaneChanging>(from)) {
    return true;
  }
  // {Completed} and {NotLaneChanging -> NotLaneChanging} transitions are consistent
  return false;
}
}  // namespace autoware::behavior_path_planner
