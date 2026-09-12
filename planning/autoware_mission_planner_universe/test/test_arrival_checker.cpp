// Copyright 2026 TIER IV, Inc.
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

#include "../src/mission_planner/arrival_checker.hpp"

#include <rclcpp/time.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string>

namespace
{
using autoware::mission_planner_universe::ArrivalChecker;
using autoware::mission_planner_universe::ArrivalCheckerThreshold;
using Odometry = ArrivalChecker::Odometry;
using PoseWithUuidStamped = ArrivalChecker::PoseWithUuidStamped;

constexpr double default_lateral_distance = 1.0;
constexpr double default_longitudinal_undershoot_distance = 1.0;
constexpr double default_longitudinal_overshoot_distance = 1.0;
constexpr double default_angle = 45.0 * M_PI / 180.0;
constexpr double default_duration = 1.0;

ArrivalCheckerThreshold make_threshold(
  const double lateral_distance = default_lateral_distance,
  const double longitudinal_undershoot_distance = default_longitudinal_undershoot_distance,
  const double longitudinal_overshoot_distance = default_longitudinal_overshoot_distance)
{
  ArrivalCheckerThreshold threshold;
  threshold.angle = default_angle;
  threshold.lateral_distance = lateral_distance;
  threshold.longitudinal_undershoot_distance = longitudinal_undershoot_distance;
  threshold.longitudinal_overshoot_distance = longitudinal_overshoot_distance;
  threshold.duration = default_duration;
  return threshold;
}

Odometry make_odometry(
  const double time_sec, const double x, const double y, const double yaw, const double velocity_x,
  const std::string & frame_id = "map")
{
  Odometry odometry;
  odometry.header.stamp = rclcpp::Time(time_sec * 1e9);
  odometry.header.frame_id = frame_id;
  odometry.pose.pose.position.x = x;
  odometry.pose.pose.position.y = y;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  odometry.pose.pose.orientation = tf2::toMsg(quaternion);
  odometry.twist.twist.linear.x = velocity_x;
  return odometry;
}

PoseWithUuidStamped make_goal(
  const double x, const double y, const double yaw, const std::string & frame_id = "map")
{
  PoseWithUuidStamped goal;
  goal.header.frame_id = frame_id;
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  goal.pose.orientation = tf2::toMsg(quaternion);
  return goal;
}

// Feeds a stopped (zero-velocity) odometry sequence at the given pose that spans the stop duration,
// so that the internal stopped check becomes true.
void feed_stopped_at(
  ArrivalChecker & arrival_checker, const double x, const double y, const double yaw)
{
  for (double time_sec = 0.0; time_sec <= default_duration + 0.5; time_sec += 0.1) {
    arrival_checker.update(make_odometry(time_sec, x, y, yaw, 0.0));
  }
}

}  // namespace

TEST(ArrivalChecker, NoGoalReturnsFalse)
{
  // Arrange
  ArrivalChecker arrival_checker(make_threshold());
  feed_stopped_at(arrival_checker, 0.0, 0.0, 0.0);

  // Act & Assert
  EXPECT_FALSE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, NoOdometryReturnsFalse)
{
  // Arrange
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));

  // Act & Assert
  EXPECT_FALSE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, DifferentFrameIdReturnsFalse)
{
  // Arrange
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0, "map"));
  for (double time_sec = 0.0; time_sec <= default_duration + 0.5; time_sec += 0.1) {
    arrival_checker.update(make_odometry(time_sec, 0.0, 0.0, 0.0, 0.0, "base_link"));
  }

  // Act & Assert
  EXPECT_FALSE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, OutOfLateralDistanceReturnsFalse)
{
  // Arrange: offset perpendicular to the goal heading, beyond the lateral bound.
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  feed_stopped_at(arrival_checker, 0.0, default_lateral_distance + 0.5, 0.0);

  // Act & Assert
  EXPECT_FALSE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, OutOfLongitudinalOvershootReturnsFalse)
{
  // Arrange: ahead of the goal, beyond the overshoot bound.
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  feed_stopped_at(arrival_checker, default_longitudinal_overshoot_distance + 0.5, 0.0, 0.0);

  // Act & Assert
  EXPECT_FALSE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, OutOfLongitudinalUndershootReturnsFalse)
{
  // Arrange: behind the goal, beyond the undershoot bound.
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  feed_stopped_at(arrival_checker, -(default_longitudinal_undershoot_distance + 0.5), 0.0, 0.0);

  // Act & Assert
  EXPECT_FALSE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, LateralAtBoundaryArrived)
{
  // Exactly at the lateral bound is still an arrival (inclusive `<=`).
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  feed_stopped_at(arrival_checker, 0.0, default_lateral_distance, 0.0);

  // Act & Assert
  EXPECT_TRUE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, LongitudinalOvershootAtBoundaryArrived)
{
  // Exactly at the overshoot bound is still an arrival (inclusive `<=`).
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  feed_stopped_at(arrival_checker, default_longitudinal_overshoot_distance, 0.0, 0.0);

  // Act & Assert
  EXPECT_TRUE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, LongitudinalUndershootAtBoundaryArrived)
{
  // Exactly at the undershoot bound is still an arrival (inclusive `>=`).
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  feed_stopped_at(arrival_checker, -default_longitudinal_undershoot_distance, 0.0, 0.0);

  // Act & Assert
  EXPECT_TRUE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, ArrivedBehindGoalWithinUndershoot)
{
  // The undershoot (behind) bound is applied independently of the overshoot bound: a point behind
  // the goal, beyond the (narrow) overshoot but within the (wide) undershoot, is an arrival.
  ArrivalChecker arrival_checker(make_threshold(default_lateral_distance, 2.0, 0.5));
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  feed_stopped_at(arrival_checker, -1.5, 0.0, 0.0);

  // Act & Assert
  EXPECT_TRUE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, ArrivedAheadGoalWithinOvershoot)
{
  // The overshoot (ahead) bound is applied independently of the undershoot bound: a point ahead of
  // the goal, beyond the (narrow) undershoot but within the (wide) overshoot, is an arrival.
  ArrivalChecker arrival_checker(make_threshold(default_lateral_distance, 0.5, 2.0));
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  feed_stopped_at(arrival_checker, 1.5, 0.0, 0.0);

  // Act & Assert
  EXPECT_TRUE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, RotatedGoalLongitudinalWithinBoundArrived)
{
  // Goal facing +y, so its longitudinal axis is world +y. An offset along world +y must be
  // treated as longitudinal (within the wide longitudinal bound), not lateral.
  ArrivalChecker arrival_checker(make_threshold(0.5, 2.0, 2.0));
  const double goal_yaw = M_PI / 2.0;
  arrival_checker.set_goal(make_goal(0.0, 0.0, goal_yaw));
  feed_stopped_at(arrival_checker, 0.0, 1.5, goal_yaw);

  // Act & Assert
  EXPECT_TRUE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, RotatedGoalLateralOutOfBoundReturnsFalse)
{
  // Goal facing +y, so its lateral axis is world +x. An offset along world +x must be treated as
  // lateral (beyond the narrow lateral bound), not longitudinal.
  ArrivalChecker arrival_checker(make_threshold(0.5, 2.0, 2.0));
  const double goal_yaw = M_PI / 2.0;
  arrival_checker.set_goal(make_goal(0.0, 0.0, goal_yaw));
  feed_stopped_at(arrival_checker, 1.5, 0.0, goal_yaw);

  // Act & Assert
  EXPECT_FALSE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, OutOfAngleReturnsFalse)
{
  // Arrange
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  feed_stopped_at(arrival_checker, 0.0, 0.0, default_angle + 0.2);

  // Act & Assert
  EXPECT_FALSE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, AngleJustWithinBoundaryArrived)
{
  // Just inside the angle bound is an arrival (complements OutOfAngleReturnsFalse just outside it).
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  feed_stopped_at(arrival_checker, 0.0, 0.0, default_angle - 1e-3);

  // Act & Assert
  EXPECT_TRUE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, AngleWrapAroundArrived)
{
  // The heading difference is normalized: a goal near +pi and an ego near -pi differ by ~2*pi in
  // raw terms but are actually aligned, so this is an arrival.
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 3.0));
  feed_stopped_at(arrival_checker, 0.0, 0.0, -3.0);

  // Act & Assert
  EXPECT_TRUE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, NotStoppedReturnsFalse)
{
  // Arrange
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  for (double time_sec = 0.0; time_sec <= default_duration + 0.5; time_sec += 0.1) {
    arrival_checker.update(make_odometry(time_sec, 0.0, 0.0, 0.0, 1.0));  // moving
  }

  // Act & Assert
  EXPECT_FALSE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, NotStoppedLongEnoughReturnsFalse)
{
  // Arrange: stopped, but the buffer does not yet span the required stop duration.
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  arrival_checker.update(make_odometry(0.0, 0.0, 0.0, 0.0, 0.0));
  arrival_checker.update(make_odometry(default_duration / 2.0, 0.0, 0.0, 0.0, 0.0));

  // Act & Assert
  EXPECT_FALSE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, MovingEarlierThenStoppedIsArrived)
{
  // Only the most recent `duration` of history matters: moving earlier but stopped for the last
  // `duration` at the goal is an arrival.
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  for (double time_sec = 0.0; time_sec <= 2.0 * default_duration; time_sec += 0.1) {
    const double velocity_x = (time_sec < 0.5 * default_duration) ? 1.0 : 0.0;
    arrival_checker.update(make_odometry(time_sec, 0.0, 0.0, 0.0, velocity_x));
  }

  // Act & Assert
  EXPECT_TRUE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, ArrivedWhenStoppedAtGoal)
{
  // Arrange
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  feed_stopped_at(arrival_checker, 0.0, 0.0, 0.0);

  // Act & Assert
  EXPECT_TRUE(arrival_checker.is_arrived());
}

TEST(ArrivalChecker, ClearGoalAfterSet)
{
  // Arrange
  ArrivalChecker arrival_checker(make_threshold());
  arrival_checker.set_goal(make_goal(0.0, 0.0, 0.0));
  feed_stopped_at(arrival_checker, 0.0, 0.0, 0.0);
  ASSERT_TRUE(arrival_checker.is_arrived());

  // Act
  arrival_checker.clear_goal();

  // Assert
  EXPECT_FALSE(arrival_checker.is_arrived());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
