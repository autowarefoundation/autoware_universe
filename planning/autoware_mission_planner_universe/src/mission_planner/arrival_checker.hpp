// Copyright 2022 TIER IV, Inc.
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

#ifndef MISSION_PLANNER__ARRIVAL_CHECKER_HPP_
#define MISSION_PLANNER__ARRIVAL_CHECKER_HPP_

#include <rclcpp/time.hpp>

#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <optional>

namespace autoware::mission_planner_universe
{

struct ArrivalCheckerThreshold
{
  // arrival_check_angle [rad] (stores the deg2rad-converted value)
  double angle{0.0};
  double lateral_distance{0.0};  // arrival_check_lateral_distance [m]
  double longitudinal_undershoot_distance{
    0.0};  // arrival_check_longitudinal_undershoot_distance [m]
  double longitudinal_overshoot_distance{0.0};  // arrival_check_longitudinal_overshoot_distance [m]
  double duration{0.0};                         // arrival_check_duration [s]
};

class ArrivalChecker
{
public:
  using PoseWithUuidStamped = autoware_planning_msgs::msg::PoseWithUuidStamped;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Odometry = nav_msgs::msg::Odometry;

  explicit ArrivalChecker(const ArrivalCheckerThreshold & threshold);

  void clear_goal();
  void set_goal(const PoseWithUuidStamped & goal);

  // Keeps the latest odometry and accumulates its twist into the buffer (command).
  // Equivalent to the odometry subscription + onOdom of the removed VehicleStopChecker.
  // Call it every cycle regardless of the route state.
  void update(const Odometry & odometry);

  // Judges arrival from the latest odometry pose and the twist buffer (const query).
  // Returns false while no odometry has been received. The reference time is the stamp of the
  // latest odometry.
  bool is_arrived() const;

private:
  ArrivalCheckerThreshold threshold_;
  std::optional<PoseWithUuidStamped> goal_with_uuid_;
  std::optional<Odometry> odometry_;
  std::deque<TwistStamped> twist_buffer_;

  static constexpr double velocity_buffer_time_sec = 10.0;
};

}  // namespace autoware::mission_planner_universe

#endif  // MISSION_PLANNER__ARRIVAL_CHECKER_HPP_
