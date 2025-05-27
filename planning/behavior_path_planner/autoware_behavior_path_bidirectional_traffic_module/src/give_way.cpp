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
#include "autoware/motion_utils/trajectory/path_shift.hpp"
#include "autoware/trajectory/utils/closest.hpp"
#include "autoware/trajectory/utils/shift.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <algorithm>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
experimental::trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
NoNeedToGiveWay::modify_trajectory(
  GiveWay * give_way,
  const experimental::trajectory::Trajectory<
    autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const OncomingCars & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
  const double & ego_speed)
{
  if (ego_speed < 0.1) {
    return trajectory;
  }
  std::optional<CarObject> front_oncoming_car = oncoming_cars.get_front_oncoming_car();
  if (front_oncoming_car.has_value()) {
    bool go_to_shift_mode =
      give_way->decide_ego_stop_pose(ego_pose, ego_speed, *front_oncoming_car);
    if (go_to_shift_mode) {
      give_way->transition_to<ApproachingToShift>();
    }
    return give_way->modify_trajectory_for_waiting(trajectory, ego_pose, true);
  }
  return trajectory;
}

experimental::trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
ApproachingToShift::modify_trajectory(
  GiveWay * give_way,
  const experimental::trajectory::Trajectory<
    autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const OncomingCars & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose, const double &)
{
  if (oncoming_cars.empty()) {
    give_way->transition_to<NoNeedToGiveWay>();
    return trajectory;
  }
  auto ego_stop_pose = give_way->get_pull_over_pose();
  if (!ego_stop_pose.has_value()) {
    return trajectory;
  }

  double ego_stop_pose_s =
    autoware::experimental::trajectory::closest(trajectory, ego_stop_pose.value().position);

  double ego_s = autoware::experimental::trajectory::closest(trajectory, ego_pose.position);

  std::optional<double> shift_distance_to_pull_over = give_way->get_shift_distance_to_pull_over();

  if (!shift_distance_to_pull_over.has_value()) {
    give_way->transition_to<NoNeedToGiveWay>();
    return trajectory;
  }

  if (ego_stop_pose_s - ego_s < shift_distance_to_pull_over.value()) {
    give_way->transition_to<ShiftingRoadside>();
  }

  return give_way->modify_trajectory_for_waiting(trajectory, ego_pose, true);
}

experimental::trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
ShiftingRoadside::modify_trajectory(
  GiveWay * give_way,
  const experimental::trajectory::Trajectory<
    autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const OncomingCars &, const geometry_msgs::msg::Pose & ego_pose, const double & ego_speed)
{
  if (ego_speed < 0.1) {
    give_way->transition_to<WaitingForOncomingCarsToPass>();
  }
  return give_way->modify_trajectory_for_waiting(trajectory, ego_pose, true);
}

experimental::trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
WaitingForOncomingCarsToPass::modify_trajectory(
  GiveWay * give_way,
  const experimental::trajectory::Trajectory<
    autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const OncomingCars & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose, const double &)
{
  if (oncoming_cars.empty()) {
    give_way->transition_to<BackToNormalLane>();
  }
  return give_way->modify_trajectory_for_waiting(trajectory, ego_pose, true);
}

experimental::trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
BackToNormalLane::modify_trajectory(
  GiveWay * give_way,
  const experimental::trajectory::Trajectory<
    autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const OncomingCars &, const geometry_msgs::msg::Pose & ego_pose, const double &)
{
  auto ego_stop_pose = give_way->get_pull_over_pose();
  if (!ego_stop_pose.has_value()) {
    return trajectory;
  }

  double ego_stop_pose_s =
    autoware::experimental::trajectory::closest(trajectory, ego_stop_pose.value().position);

  double ego_s = autoware::experimental::trajectory::closest(trajectory, ego_pose);

  if (
    ego_s > ego_stop_pose_s + *give_way->get_shift_distance_to_back_to_normal_lane() ||
    ego_stop_pose_s == 0.0) {
    give_way->transition_to<NoNeedToGiveWay>();
  }

  return give_way->modify_trajectory_for_waiting(trajectory, ego_pose, false);
}

GiveWay::GiveWay(
  ConnectedBidirectionalLanelets::SharedConstPtr bidirectional_lanelets,  //
  const EgoParameters & ego_params,                                       //
  const BidirectionalTrafficModuleParameters & parameters,
  const std::function<void(geometry_msgs::msg::Pose)> & insert_stop_wall)
: bidirectional_lanelets_(std::move(bidirectional_lanelets)),
  state_(std::make_shared<NoNeedToGiveWay>()),
  ego_params_(ego_params),
  parameters_(parameters),
  insert_stop_wall_(insert_stop_wall)
{
}

experimental::trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
GiveWay::modify_trajectory(
  const experimental::trajectory::Trajectory<
    autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const OncomingCars & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
  const double & ego_speed)
{
  return state_->modify_trajectory(this, trajectory, oncoming_cars, ego_pose, ego_speed);
}

bool GiveWay::decide_ego_stop_pose(
  const geometry_msgs::msg::Pose & ego_pose, const double & ego_speed,
  const CarObject & front_oncoming_car)
{
  auto center_line = bidirectional_lanelets_->get_center_line();
  const double ego_s = autoware::experimental::trajectory::closest(center_line, ego_pose.position);

  double shift_prepare_distance = ego_speed * parameters_.time_to_prepare_pull_over;

  double distance_to_front_oncoming_car =
    distance_on_lane(ego_pose, front_oncoming_car.get_pose(), bidirectional_lanelets_);

  double desired_stop_point_from_ego =
    (distance_to_front_oncoming_car -
     front_oncoming_car.get_speed() * parameters_.wait_time_for_oncoming_car) *
    ego_speed / (ego_speed + front_oncoming_car.get_speed());

  double shift = std::min(
    parameters_.shift_distance_to_pull_over_from_center_line,  //
    bidirectional_lanelets_->average_lane_width() / 2.0 - ego_params_.vehicle_width / 2.0 -
      parameters_.min_distance_from_roadside);

  double min_longitudinal_distance =
    motion_utils::calc_longitudinal_dist_from_jerk(shift, parameters_.max_lateral_jerk, ego_speed);
  double stop_from_ego = 0.0;

  if (desired_stop_point_from_ego < min_longitudinal_distance + shift_prepare_distance) {
    shift_distance_to_pull_over_ = min_longitudinal_distance;
    stop_from_ego = min_longitudinal_distance + shift_prepare_distance;
  } else {
    shift_distance_to_pull_over_ = motion_utils::calc_longitudinal_dist_from_jerk(
      shift, parameters_.min_lateral_jerk, ego_speed);
    stop_from_ego = desired_stop_point_from_ego;
  }

  bool go_to_shift_mode =
    *shift_distance_to_pull_over_ + shift_prepare_distance + 1e-3 > stop_from_ego;

  shift_distance_to_back_to_normal_lane_ = min_longitudinal_distance;

  auto pose_on_the_centerline = center_line.compute(ego_s + stop_from_ego);

  geometry_msgs::msg::Pose pose;

  tf2::Quaternion q;
  tf2::fromMsg(pose_on_the_centerline.orientation, q);
  tf2::Vector3 offset(0.0, shift, 0.0);
  tf2::Vector3 offset_world = tf2::quatRotate(q, offset);
  pose.position.x = pose_on_the_centerline.position.x + offset_world.x();
  pose.position.y = pose_on_the_centerline.position.y + offset_world.y();
  pose.position.z = pose_on_the_centerline.position.z + offset_world.z();

  ego_stop_point_for_waiting_ = pose;

  return go_to_shift_mode;
}

experimental::trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
GiveWay::modify_trajectory_for_waiting(
  const experimental::trajectory::Trajectory<
    autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
  const geometry_msgs::msg::Pose & ego_pose, bool stop_at_stop_point) const
{
  if (!ego_stop_point_for_waiting_.has_value()) {
    return trajectory;
  }

  double stop_point =
    experimental::trajectory::closest(trajectory, ego_stop_point_for_waiting_.value());

  if (stop_point == 0.0) {
    return trajectory;
  }

  experimental::trajectory::ShiftInterval shift1{
    stop_point - *shift_distance_to_pull_over_,    // start
    shift1.start + *shift_distance_to_pull_over_,  // end
    -autoware_utils_geometry::calc_distance2d(
      trajectory.compute(stop_point), ego_stop_point_for_waiting_.value())  // lateral_offset
  };

  experimental::trajectory::ShiftInterval shift2{
    shift1.end,                                            // start
    shift1.end + *shift_distance_to_back_to_normal_lane_,  // end
    autoware_utils_geometry::calc_distance2d(
      trajectory.compute(stop_point), ego_stop_point_for_waiting_.value())  // lateral_offset
  };

  experimental::trajectory::ShiftParameters shift_params{0.0, 0.0};

  auto shifted_trajectory =
    experimental::trajectory::shift(trajectory, {shift1, shift2}, shift_params);

  if (!shifted_trajectory) {
    return trajectory;
  }

  if (stop_at_stop_point) {
    double stop_point_in_shifted_trajectory =
      experimental::trajectory::closest(*shifted_trajectory, ego_stop_point_for_waiting_.value());
    double ego_point = experimental::trajectory::closest(*shifted_trajectory, ego_pose.position);
    shifted_trajectory->longitudinal_velocity_mps()
      .range(std::max(stop_point_in_shifted_trajectory, ego_point), shifted_trajectory->length())
      .set(0.0);
    insert_stop_wall_(
      shifted_trajectory->compute(std::max(stop_point_in_shifted_trajectory, ego_point))
        .point.pose);
  }
  return *shifted_trajectory;
}

std::optional<geometry_msgs::msg::Pose> GiveWay::get_pull_over_pose() const
{
  return ego_stop_point_for_waiting_;
}

std::string GiveWay::get_state_name() const
{
  return state_->name();
}

std::optional<double> GiveWay::get_shift_distance_to_pull_over() const
{
  return shift_distance_to_pull_over_;
}

std::optional<double> GiveWay::get_shift_distance_to_back_to_normal_lane() const
{
  return shift_distance_to_back_to_normal_lane_;
}

}  // namespace autoware::behavior_path_planner
