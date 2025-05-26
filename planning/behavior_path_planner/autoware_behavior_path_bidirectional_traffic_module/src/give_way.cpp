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
#include "autoware/behavior_path_bidirectional_traffic_module/give_way.hpp"

#include "autoware/behavior_path_bidirectional_traffic_module/oncoming_car.hpp"
#include "autoware/trajectory/utils/shift.hpp"

#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
NoNeedToGiveWay::modify_trajectory(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose, bool)
{
  if (!oncoming_cars.empty()) {
    give_way_->decide_ego_stop_pose(ego_pose);
    give_way_->transition_to<ShiftingRoadside>(give_way_);
  }
  return trajectory;
}

trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
ShiftingRoadside::modify_trajectory(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<OncomingCar> &, const geometry_msgs::msg::Pose &, bool is_vehicle_stopped)
{
  if (is_vehicle_stopped) {
    give_way_->transition_to<WaitingForOncomingCarsToPass>(give_way_);
  }
  return give_way_->modify_trajectory_for_waiting(trajectory, true);
}

trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
WaitingForOncomingCarsToPass::modify_trajectory(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose &, bool)
{
  if (oncoming_cars.empty()) {
    give_way_->transition_to<BackToNormalLane>(give_way_);
  }
  return give_way_->modify_trajectory_for_waiting(trajectory, true);
}

trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
BackToNormalLane::modify_trajectory(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<OncomingCar> &, const geometry_msgs::msg::Pose &, bool)
{
  return give_way_->modify_trajectory_for_waiting(trajectory, false);
}

GiveWay::GiveWay(
  ConnectedBidirectionalLanelets bidirectional_lanelets,  //
  const double & vehicle_width,                           //
  const double & shift_starting_length,                   //
  const double & distance_to_shift,                       //
  const double & min_distance_to_left)
: bidirectional_lanelets_(std::move(bidirectional_lanelets)),
  state_(std::make_shared<NoNeedToGiveWay>(this)),
  vehicle_width_(vehicle_width),
  shift_starting_length_(shift_starting_length),
  distance_to_shift_(distance_to_shift),
  min_distance_to_left_(min_distance_to_left)
{
}

[[nodiscard]] trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
GiveWay::modify_trajectory(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
  bool is_vehicle_stopped)
{
  return state_->modify_trajectory(trajectory, oncoming_cars, ego_pose, is_vehicle_stopped);
}

void GiveWay::decide_ego_stop_pose(const geometry_msgs::msg::Pose & ego_pose)
{
  auto left = bidirectional_lanelets_.get_left_line();
  auto ego = autoware::trajectory::closest(left, ego_pose.position);
  auto pose_on_the_left_line = left.compute(ego + shift_starting_length_);
  double shift = min_distance_to_left_ + vehicle_width_ / 2.0;
  geometry_msgs::msg::Pose pose;

  tf2::Quaternion q;
  tf2::fromMsg(pose_on_the_left_line.orientation, q);
  tf2::Vector3 offset(0.0, -shift, 0.0);
  tf2::Vector3 offset_world = tf2::quatRotate(q, offset);
  pose.position.x = pose_on_the_left_line.position.x + offset_world.x();
  pose.position.y = pose_on_the_left_line.position.y + offset_world.y();
  pose.position.z = pose_on_the_left_line.position.z + offset_world.z();

  ego_stop_point_for_waiting_ = pose;
}

[[nodiscard]] trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
GiveWay::modify_trajectory_for_waiting(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  bool stop_at_stop_point) const
{
  if (!ego_stop_point_for_waiting_.has_value()) {
    return trajectory;
  }

  std::vector<trajectory::ShiftInterval> shifts;
  trajectory::ShiftInterval shift1;
  double stop_point = trajectory::closest(trajectory, ego_stop_point_for_waiting_.value());

  if (stop_point == 0.0) {
    return trajectory;
  }

  shift1.start = stop_point - distance_to_shift_;
  shift1.end = shift1.start + distance_to_shift_;
  shift1.lateral_offset = -autoware::universe_utils::calcDistance2d(
    trajectory.compute(stop_point), ego_stop_point_for_waiting_.value());

  trajectory::ShiftInterval shift2;
  shift2.start = shift1.end;
  shift2.end = shift2.start + distance_to_shift_;
  shift2.lateral_offset = autoware::universe_utils::calcDistance2d(
    trajectory.compute(stop_point), ego_stop_point_for_waiting_.value());

  auto shifted_trajectory = trajectory::shift(trajectory, {shift1, shift2});

  if (stop_at_stop_point) {
    auto stop_point = trajectory::closest(shifted_trajectory, ego_stop_point_for_waiting_.value());
    shifted_trajectory.longitudinal_velocity_mps()
      .range(stop_point, shifted_trajectory.length())
      .set(0.0);
  }
  return shifted_trajectory;
}

}  // namespace autoware::behavior_path_planner
