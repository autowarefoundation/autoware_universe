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

#include "autoware/behavior_path_bidirectional_traffic_module/connected_bidirectional_lanelets.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/oncoming_car.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/parameter.hpp"
#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/utils.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
namespace autoware::behavior_path_planner
{

class GiveWay;
class GiveWayState
{
protected:
  GiveWay * give_way_;

public:
  explicit GiveWayState(GiveWay * give_way) : give_way_(give_way) {}
  virtual trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
      trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    const double & vehicle_speed) = 0;

  [[nodiscard]] virtual bool is_stop_required() const = 0;
};

class NoNeedToGiveWay : public GiveWayState
{
public:
  explicit NoNeedToGiveWay(GiveWay * give_way) : GiveWayState(give_way) {}

  trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
      trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    const double & vehicle_speed) override;

  [[nodiscard]] bool is_stop_required() const override { return false; }
};

class ApproachingToShift : public GiveWayState
{
public:
  explicit ApproachingToShift(GiveWay * give_way) : GiveWayState(give_way) {}
  trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
      trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    const double & vehicle_speed) override;

  [[nodiscard]] bool is_stop_required() const override { return true; }
};

class ShiftingRoadside : public GiveWayState
{
public:
  explicit ShiftingRoadside(GiveWay * give_way) : GiveWayState(give_way) {}
  trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
      trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    const double & vehicle_speed) override;

  [[nodiscard]] bool is_stop_required() const override { return true; }
};

class WaitingForOncomingCarsToPass : public GiveWayState
{
public:
  explicit WaitingForOncomingCarsToPass(GiveWay * give_way) : GiveWayState(give_way) {}
  trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
      trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    const double & vehicle_speed) override;

  [[nodiscard]] bool is_stop_required() const override { return true; }
};

class BackToNormalLane : public GiveWayState
{
public:
  explicit BackToNormalLane(GiveWay * give_way) : GiveWayState(give_way) {}
  trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
      trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    const double & vehicle_speed) override;

  [[nodiscard]] bool is_stop_required() const override { return false; }
};

class GiveWay
{
private:
  ConnectedBidirectionalLanelets bidirectional_lanelets_;
  std::shared_ptr<GiveWayState> state_;
  std::optional<geometry_msgs::msg::Pose> ego_stop_point_for_waiting_;
  EgoParameters ego_params_;

  double time_to_prepare_pull_over_;
  double default_shift_distance_to_pull_over_;
  double shift_ratio_for_pull_over_;

public:
  GiveWay(
    ConnectedBidirectionalLanelets bidirectional_lanelets,  //
    const EgoParameters & ego_params,                       //
    const double & time_to_prepare_pull_over_,              //
    const double & default_shift_distance_to_pull_over,     //
    const double & shift_ratio_for_pull_over);

  [[nodiscard]] trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
      trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    const double & vehicle_speed);

  template <class State, class... Args>
  void transition_to(Args &&... args)
  {
    state_ = std::make_shared<State>(std::forward<Args>(args)...);
  }

  void decide_ego_stop_pose(
    const geometry_msgs::msg::Pose & ego_pose, const double & vehicle_speed);

  [[nodiscard]] trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory_for_waiting(
    const trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId> &
      trajectory,
    bool stop_at_stop_point = false) const;

  [[nodiscard]] bool is_stop_required() const { return state_->is_stop_required(); }

  [[nodiscard]] auto get_stop_pose() const { return ego_stop_point_for_waiting_; }

  [[nodiscard]] double get_shift_distance_to_pull_over() const
  {
    return default_shift_distance_to_pull_over_;
  }
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__GIVE_WAY_HPP_
