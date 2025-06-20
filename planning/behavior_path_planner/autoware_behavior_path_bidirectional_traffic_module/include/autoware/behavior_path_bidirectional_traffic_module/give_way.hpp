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

#include "autoware/behavior_path_bidirectional_traffic_module/bidirectional_lanelets.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/oncoming_car.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/parameter.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace autoware::behavior_path_planner
{
class GiveWay;
class GiveWayState
{
public:
  GiveWayState() = default;
  GiveWayState(const GiveWayState &) = delete;
  GiveWayState(GiveWayState &&) = delete;
  GiveWayState & operator=(const GiveWayState &) = delete;
  GiveWayState & operator=(GiveWayState &&) = delete;
  virtual ~GiveWayState() = default;

  virtual experimental::trajectory::Trajectory<
    autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    GiveWay * give_way,
    const experimental::trajectory::Trajectory<
      autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const OncomingCars & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    const double & ego_speed) = 0;

  [[nodiscard]] virtual std::string name() const = 0;
};

class NoNeedToGiveWay : public GiveWayState
{
public:
  experimental::trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    GiveWay * give_way,
    const experimental::trajectory::Trajectory<
      autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const OncomingCars & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    const double & ego_speed) override;

  [[nodiscard]] std::string name() const override { return "NoNeedToGiveWay"; }
};

class ApproachingToShift : public GiveWayState
{
public:
  experimental::trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    GiveWay * give_way,
    const experimental::trajectory::Trajectory<
      autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const OncomingCars & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    const double & ego_speed) override;

  [[nodiscard]] std::string name() const override { return "ApproachingToShift"; }
};

class ShiftingRoadside : public GiveWayState
{
public:
  experimental::trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    GiveWay * give_way,
    const experimental::trajectory::Trajectory<
      autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const OncomingCars & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    const double & ego_speed) override;

  [[nodiscard]] std::string name() const override { return "ShiftingRoadside"; }
};

class WaitingForOncomingCarsToPass : public GiveWayState
{
public:
  experimental::trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    GiveWay * give_way,
    const experimental::trajectory::Trajectory<
      autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const OncomingCars & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    const double & ego_speed) override;

  [[nodiscard]] std::string name() const override { return "WaitingForOncomingCarsToPass"; }
};

class BackToNormalLane : public GiveWayState
{
public:
  experimental::trajectory::Trajectory<autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    GiveWay * give_way,
    const experimental::trajectory::Trajectory<
      autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const OncomingCars & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    const double & ego_speed) override;

  [[nodiscard]] std::string name() const override { return "BackToNormalLane"; }
};

class GiveWay
{
private:
  ConnectedBidirectionalLanelets::SharedConstPtr bidirectional_lanelets_;
  std::shared_ptr<GiveWayState> state_;

  std::optional<geometry_msgs::msg::Pose> ego_stop_point_for_waiting_;
  std::optional<double> shift_distance_to_pull_over_;
  std::optional<double> shift_distance_to_back_to_normal_lane_;

  EgoParameters ego_params_;
  BidirectionalTrafficModuleParameters parameters_;

  std::function<void(geometry_msgs::msg::Pose)> insert_stop_wall_;

public:
  GiveWay(
    ConnectedBidirectionalLanelets::SharedConstPtr bidirectional_lanelets,  //
    const EgoParameters & ego_params,                                       //
    const BidirectionalTrafficModuleParameters & parameters,
    const std::function<void(geometry_msgs::msg::Pose)> & insert_stop_wall =
      [](geometry_msgs::msg::Pose) {});

  [[nodiscard]] experimental::trajectory::Trajectory<
    autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    const experimental::trajectory::Trajectory<
      autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const OncomingCars & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    const double & ego_speed);

  template <class State, class... Args>
  void transition_to(Args &&... args)
  {
    state_ = std::make_shared<State>(std::forward<Args>(args)...);
  }

  bool decide_ego_stop_pose(
    const geometry_msgs::msg::Pose & ego_pose, const double & ego_speed,
    const CarObject & front_oncoming_car);

  [[nodiscard]] experimental::trajectory::Trajectory<
    autoware_internal_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory_for_waiting(
    const experimental::trajectory::Trajectory<
      autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const geometry_msgs::msg::Pose & ego_pose, bool stop_at_stop_point = false) const;

  [[nodiscard]] std::optional<geometry_msgs::msg::Pose> get_pull_over_pose() const;
  [[nodiscard]] std::string get_state_name() const;
  [[nodiscard]] std::optional<double> get_shift_distance_to_pull_over() const;
  [[nodiscard]] std::optional<double> get_shift_distance_to_back_to_normal_lane() const;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__GIVE_WAY_HPP_
