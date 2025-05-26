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

#ifndef AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__GIVE_WAY_HPP_
#define AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__GIVE_WAY_HPP_

#include "autoware/behavior_path_bidirectional_traffic_module/bidirectional_lane_utils.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/oncoming_car.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/utils/closest.hpp"
#include "autoware/trajectory/utils/shift.hpp"

#include <autoware/trajectory/forward.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>

#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
namespace autoware::behavior_path_planner
{

class GiveWay;
class GiveWayState;
class NoNeedToGiveWay;
class ShiftingRoadside;
class WaitingForOncomingCarsToPass;
class BackToNormalLane;

class GiveWayState
{
protected:
  GiveWay * give_way_;

public:
  explicit GiveWayState(GiveWay * give_way) : give_way_(give_way) {}
  virtual trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> modify_trajectory(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    bool is_vehicle_stopped) = 0;
};

class NoNeedToGiveWay : public GiveWayState
{
public:
  explicit NoNeedToGiveWay(GiveWay * give_way) : GiveWayState(give_way) {}

  trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> modify_trajectory(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    bool is_vehicle_stopped) override;
};

class ShiftingRoadside : public GiveWayState
{
public:
  explicit ShiftingRoadside(GiveWay * give_way) : GiveWayState(give_way) {}
  trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> modify_trajectory(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    bool is_vehicle_stopped) override;
};

class WaitingForOncomingCarsToPass : public GiveWayState
{
public:
  explicit WaitingForOncomingCarsToPass(GiveWay * give_way) : GiveWayState(give_way) {}
  trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> modify_trajectory(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    bool is_vehicle_stopped) override;
};

class BackToNormalLane : public GiveWayState
{
public:
  explicit BackToNormalLane(GiveWay * give_way) : GiveWayState(give_way) {}
  trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> modify_trajectory(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    bool is_vehicle_stopped) override;
};

class GiveWay
{
private:
  ConnectedBidirectionalLanelets bidirectional_lanelets_;
  std::shared_ptr<GiveWayState> state_;
  std::optional<geometry_msgs::msg::Point> ego_stop_point_for_waiting_;

  const double vehicle_width_;
  const double shift_starting_length_;
  const double distance_to_shift_;
  const double min_distance_to_left_;

public:
  GiveWay(
    const ConnectedBidirectionalLanelets & bidirectional_lanelets,  //
    const double & vehicle_width,                                   //
    const double & shift_starting_length,                           //
    const double & distance_to_shift,                               //
    const double & min_distance_to_left = 0.0)
  : bidirectional_lanelets_(bidirectional_lanelets),
    state_(std::make_shared<NoNeedToGiveWay>(this)),
    vehicle_width_(vehicle_width),
    shift_starting_length_(shift_starting_length),
    distance_to_shift_(distance_to_shift),
    min_distance_to_left_(min_distance_to_left)
  {
  }

  [[nodiscard]] trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    bool is_vehicle_stopped)
  {
    return state_->modify_trajectory(trajectory, oncoming_cars, ego_pose, is_vehicle_stopped);
  }

  template <class State, class... Args>
  void transition_to(Args &&... args)
  {
    state_ = std::make_shared<State>(std::forward<Args>(args)...);
  }

  void decide_ego_stop_pose(const geometry_msgs::msg::Pose & ego_pose)
  {
    auto left = bidirectional_lanelets_.get_left_line();
    auto ego = autoware::trajectory::closest(left, ego_pose.position);
    auto tmp = left.compute(ego + shift_starting_length_);
    auto dir = left.azimuth(ego + shift_starting_length_);
    geometry_msgs::msg::Point point;
    double shift = min_distance_to_left_ + vehicle_width_ / 2.0;
    point.x = tmp.x + shift * std::sin(dir);
    point.y = tmp.y - shift * std::cos(dir);
    point.z = tmp.z;
    ego_stop_point_for_waiting_ = point;
  }

  bool ego_reached_stop_point(const geometry_msgs::msg::Pose & ego_pose) const
  {
    if (!ego_stop_point_for_waiting_.has_value()) {
      return false;
    }
    return autoware::universe_utils::calcDistance2d(
             ego_pose.position, ego_stop_point_for_waiting_.value()) < 0.1;
  }

  trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory_for_waiting(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    bool stop_at_stop_point = false) const
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
      auto stop_point =
        trajectory::closest(shifted_trajectory, ego_stop_point_for_waiting_.value());
      shifted_trajectory.longitudinal_velocity_mps()
        .range(stop_point, shifted_trajectory.length())
        .set(0.0);
    }
    return shifted_trajectory;
  }
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__GIVE_WAY_HPP_
