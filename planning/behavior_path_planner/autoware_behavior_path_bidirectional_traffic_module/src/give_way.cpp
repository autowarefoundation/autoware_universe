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
#include "autoware/behavior_path_bidirectional_traffic_module/parameter.hpp"
#include "autoware/trajectory/utils/frenet_utils.hpp"
#include "autoware/trajectory/utils/shift.hpp"

#include <iostream>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
NoNeedToGiveWay::modify_trajectory(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
  const double & vehicle_speed)
{
  std::cerr << "No need to give way" << std::endl;
  if (!oncoming_cars.empty()) {
    give_way_->decide_ego_stop_pose(ego_pose, vehicle_speed);
    give_way_->transition_to<ApproachingToShift>(give_way_);
  }
  return trajectory;
}

trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
ApproachingToShift::modify_trajectory(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
  const double &)
{
  std::cerr << "Approaching to shift" << std::endl;
  if (oncoming_cars.empty()) {
    give_way_->transition_to<NoNeedToGiveWay>(give_way_);
  }
  auto ego_stop_pose = give_way_->get_stop_pose();
  if (!ego_stop_pose.has_value()) {
    return trajectory;
  }

  auto ego_stop_pose_frenet_coordinate =
    autoware::trajectory::compute_frenet_coordinate(trajectory, ego_stop_pose.value().position);

  if (!ego_stop_pose_frenet_coordinate.has_value()) {
    return trajectory;
  }

  auto ego_frenet_coordinate =
    autoware::trajectory::compute_frenet_coordinate(trajectory, ego_pose.position);

  if (!ego_frenet_coordinate.has_value()) {
    return trajectory;
  }

  if (
    ego_frenet_coordinate->first >
    ego_stop_pose_frenet_coordinate->first - give_way_->get_shift_distance_to_pull_over()) {
    give_way_->transition_to<ShiftingRoadside>(give_way_);
  }

  return give_way_->modify_trajectory_for_waiting(trajectory, true);
}

trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
ShiftingRoadside::modify_trajectory(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<OncomingCar> &, const geometry_msgs::msg::Pose &, const double & vehicle_speed)
{
  std::cerr << "Shifting roadside" << std::endl;
  if (vehicle_speed < 0.1) {
    give_way_->transition_to<WaitingForOncomingCarsToPass>(give_way_);
  }
  return give_way_->modify_trajectory_for_waiting(trajectory, true);
}

trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
WaitingForOncomingCarsToPass::modify_trajectory(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose &, const double &)
{
  std::cerr << "Waiting for oncoming cars to pass" << std::endl;
  if (oncoming_cars.empty()) {
    give_way_->transition_to<BackToNormalLane>(give_way_);
  }
  return give_way_->modify_trajectory_for_waiting(trajectory, true);
}

trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
BackToNormalLane::modify_trajectory(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<OncomingCar> &, const geometry_msgs::msg::Pose &, const double &)
{
  std::cerr << "Back to normal lane" << std::endl;
  return give_way_->modify_trajectory_for_waiting(trajectory, false);
}

GiveWay::GiveWay(
  ConnectedBidirectionalLanelets bidirectional_lanelets,  //
  const EgoParameters & ego_params,                       //
  const double & time_to_prepare_pull_over_,              //
  const double & default_shift_distance_to_pull_over,     //
  const double & shift_ratio_for_pull_over)
: bidirectional_lanelets_(std::move(bidirectional_lanelets)),
  state_(std::make_shared<NoNeedToGiveWay>(this)),
  ego_params_(ego_params),
  time_to_prepare_pull_over_(time_to_prepare_pull_over_),
  default_shift_distance_to_pull_over_(default_shift_distance_to_pull_over),
  shift_ratio_for_pull_over_(shift_ratio_for_pull_over)
{
}

[[nodiscard]] trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
GiveWay::modify_trajectory(
  const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
    trajectory,
  const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
  const double & vehicle_speed)
{
  return state_->modify_trajectory(trajectory, oncoming_cars, ego_pose, vehicle_speed);
}

void GiveWay::decide_ego_stop_pose(
  const geometry_msgs::msg::Pose & ego_pose, const double & vehicle_speed)
{
  auto center_line = bidirectional_lanelets_.get_center_line();
  const double ego_s = autoware::trajectory::closest(center_line, ego_pose.position);
  auto pose_on_the_centerline = center_line.compute(
    ego_s + time_to_prepare_pull_over_ * vehicle_speed + default_shift_distance_to_pull_over_ +
    0.1);
  double shift = shift_ratio_for_pull_over_ * (bidirectional_lanelets_.average_lane_width() / 2.0 -
                                               ego_params_.vehicle_width / 2.0);
  geometry_msgs::msg::Pose pose;

  tf2::Quaternion q;
  tf2::fromMsg(pose_on_the_centerline.orientation, q);
  tf2::Vector3 offset(0.0, shift, 0.0);
  tf2::Vector3 offset_world = tf2::quatRotate(q, offset);
  pose.position.x = pose_on_the_centerline.position.x + offset_world.x();
  pose.position.y = pose_on_the_centerline.position.y + offset_world.y();
  pose.position.z = pose_on_the_centerline.position.z + offset_world.z();

  auto [is_possible, poses] =
    bidirectional_lanelets_.check_if_is_possible_to_stop_and_suggest_alternative_stop_poses(
      pose, ego_params_);

  if (!is_possible) {
    std::cerr << "Failed to find a stop point" << std::endl;
  }

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

  shift1.start = stop_point - default_shift_distance_to_pull_over_;
  shift1.end = shift1.start + default_shift_distance_to_pull_over_;
  shift1.lateral_offset = -autoware::universe_utils::calcDistance2d(
    trajectory.compute(stop_point), ego_stop_point_for_waiting_.value());

  trajectory::ShiftInterval shift2;
  shift2.start = shift1.end;
  shift2.end = shift2.start + default_shift_distance_to_pull_over_;
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
