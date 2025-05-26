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
#include "autoware/trajectory/forward.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/utils/closest.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/impl/utils.h>

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
  virtual trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> modify_trajectory(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    bool is_vehicle_stopped) = 0;

  [[nodiscard]] virtual bool is_stop_required() const = 0;
};

class NoNeedToGiveWay : public GiveWayState
{
public:
  explicit NoNeedToGiveWay(GiveWay * give_way) : GiveWayState(give_way) {}

  trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> modify_trajectory(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    bool is_vehicle_stopped) override;

  [[nodiscard]] bool is_stop_required() const override { return false; }
};

class ShiftingRoadside : public GiveWayState
{
public:
  explicit ShiftingRoadside(GiveWay * give_way) : GiveWayState(give_way) {}
  trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> modify_trajectory(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    bool is_vehicle_stopped) override;

  [[nodiscard]] bool is_stop_required() const override { return true; }
};

class WaitingForOncomingCarsToPass : public GiveWayState
{
public:
  explicit WaitingForOncomingCarsToPass(GiveWay * give_way) : GiveWayState(give_way) {}
  trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> modify_trajectory(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    bool is_vehicle_stopped) override;

  [[nodiscard]] bool is_stop_required() const override { return true; }
};

class BackToNormalLane : public GiveWayState
{
public:
  explicit BackToNormalLane(GiveWay * give_way) : GiveWayState(give_way) {}
  trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> modify_trajectory(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    bool is_vehicle_stopped) override;

  [[nodiscard]] bool is_stop_required() const override { return false; }
};

class GiveWay
{
private:
  ConnectedBidirectionalLanelets bidirectional_lanelets_;
  std::shared_ptr<GiveWayState> state_;
  std::optional<geometry_msgs::msg::Pose> ego_stop_point_for_waiting_;

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
    const double & min_distance_to_left = 0.0);

  [[nodiscard]] trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    const std::vector<OncomingCar> & oncoming_cars, const geometry_msgs::msg::Pose & ego_pose,
    bool is_vehicle_stopped);

  template <class State, class... Args>
  void transition_to(Args &&... args)
  {
    state_ = std::make_shared<State>(std::forward<Args>(args)...);
  }

  void decide_ego_stop_pose(const geometry_msgs::msg::Pose & ego_pose);

  [[nodiscard]] bool ego_reached_stop_point(const geometry_msgs::msg::Pose & ego_pose) const
  {
    if (!ego_stop_point_for_waiting_.has_value()) {
      return false;
    }
    double distance = autoware::universe_utils::calcDistance2d(
      ego_pose.position, ego_stop_point_for_waiting_.value());
    return distance < 0.1;
  }

  auto get_ego_stop_point() const { return ego_stop_point_for_waiting_; }

  [[nodiscard]] trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId>
  modify_trajectory_for_waiting(
    const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory,
    bool stop_at_stop_point = false) const;

  [[nodiscard]] bool is_stop_required() const { return state_->is_stop_required(); }

  [[nodiscard]] auto get_stop_pose() const { return ego_stop_point_for_waiting_; }
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__GIVE_WAY_HPP_
