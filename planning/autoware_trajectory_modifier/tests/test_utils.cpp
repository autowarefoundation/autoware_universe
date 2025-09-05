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

#include "autoware/trajectory_modifier/utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using autoware::trajectory_modifier::utils::TrajectoryPoints;
using autoware_planning_msgs::msg::TrajectoryPoint;

class UtilsTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}

  TrajectoryPoint create_trajectory_point(double x, double y, double velocity = 1.0)
  {
    TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    point.pose.position.z = 0.0;
    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = 0.0;
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = velocity;
    return point;
  }

  geometry_msgs::msg::Pose create_pose(double x, double y)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
  }

  geometry_msgs::msg::Twist create_twist(double vx, double vy = 0.0, double vz = 0.0)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = vx;
    twist.linear.y = vy;
    twist.linear.z = vz;
    return twist;
  }
};

// Test validate_trajectory function
TEST_F(UtilsTest, ValidateEmptyTrajectory)
{
  TrajectoryPoints empty_trajectory;
  EXPECT_FALSE(autoware::trajectory_modifier::utils::validate_trajectory(empty_trajectory));
}

TEST_F(UtilsTest, ValidateNonEmptyTrajectory)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.0, 0.0));
  EXPECT_TRUE(autoware::trajectory_modifier::utils::validate_trajectory(trajectory));
}

TEST_F(UtilsTest, ValidateMultiplePointTrajectory)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.0, 0.0));
  trajectory.push_back(create_trajectory_point(1.0, 1.0));
  trajectory.push_back(create_trajectory_point(2.0, 2.0));
  EXPECT_TRUE(autoware::trajectory_modifier::utils::validate_trajectory(trajectory));
}

// Test calculate_distance_to_last_point function
TEST_F(UtilsTest, CalculateDistanceEmptyTrajectory)
{
  TrajectoryPoints empty_trajectory;
  auto ego_pose = create_pose(0.0, 0.0);

  double distance = autoware::trajectory_modifier::utils::calculate_distance_to_last_point(
    empty_trajectory, ego_pose);
  EXPECT_DOUBLE_EQ(distance, 0.0);
}

TEST_F(UtilsTest, CalculateDistanceSamePosition)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(4.0, 5.0));  // Different starting point
  trajectory.push_back(create_trajectory_point(5.0, 5.0));  // End at same position as ego
  auto ego_pose = create_pose(5.0, 5.0);

  double distance =
    autoware::trajectory_modifier::utils::calculate_distance_to_last_point(trajectory, ego_pose);
  EXPECT_DOUBLE_EQ(distance, 0.0);
}

TEST_F(UtilsTest, CalculateDistanceHorizontal)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.0, 0.0));
  trajectory.push_back(create_trajectory_point(10.0, 0.0));
  auto ego_pose = create_pose(0.0, 0.0);

  double distance =
    autoware::trajectory_modifier::utils::calculate_distance_to_last_point(trajectory, ego_pose);
  EXPECT_DOUBLE_EQ(distance, 10.0);
}

TEST_F(UtilsTest, CalculateDistanceVertical)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.0, 0.0));
  trajectory.push_back(create_trajectory_point(0.0, 8.0));
  auto ego_pose = create_pose(0.0, 0.0);

  double distance =
    autoware::trajectory_modifier::utils::calculate_distance_to_last_point(trajectory, ego_pose);
  EXPECT_DOUBLE_EQ(distance, 8.0);
}

TEST_F(UtilsTest, CalculateDistanceDiagonal)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.0, 0.0));
  trajectory.push_back(create_trajectory_point(3.0, 4.0));
  auto ego_pose = create_pose(0.0, 0.0);

  double distance =
    autoware::trajectory_modifier::utils::calculate_distance_to_last_point(trajectory, ego_pose);
  EXPECT_DOUBLE_EQ(distance, 5.0);  // 3-4-5 triangle
}

TEST_F(UtilsTest, CalculateDistanceMultiplePointsUsesLast)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(1.0, 1.0));
  trajectory.push_back(create_trajectory_point(2.0, 2.0));
  trajectory.push_back(create_trajectory_point(6.0, 8.0));  // Last point
  auto ego_pose = create_pose(0.0, 0.0);

  double distance =
    autoware::trajectory_modifier::utils::calculate_distance_to_last_point(trajectory, ego_pose);
  EXPECT_NEAR(
    distance, 10.0, 0.1);  // 6-8-10 triangle, allowing tolerance for arc length calculation
}

TEST_F(UtilsTest, CalculateDistanceNegativeCoordinates)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.0, 0.0));
  trajectory.push_back(create_trajectory_point(-3.0, -4.0));
  auto ego_pose = create_pose(0.0, 0.0);

  double distance =
    autoware::trajectory_modifier::utils::calculate_distance_to_last_point(trajectory, ego_pose);
  EXPECT_DOUBLE_EQ(distance, 5.0);
}

TEST_F(UtilsTest, CalculateDistanceLargeDistance)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.0, 0.0));
  trajectory.push_back(create_trajectory_point(1000.0, 1000.0));
  auto ego_pose = create_pose(0.0, 0.0);

  double distance =
    autoware::trajectory_modifier::utils::calculate_distance_to_last_point(trajectory, ego_pose);
  EXPECT_NEAR(distance, 1414.2135, 0.001);  // sqrt(2) * 1000
}

// Test is_ego_vehicle_moving function
TEST_F(UtilsTest, IsEgoVehicleMovingZeroVelocity)
{
  auto twist = create_twist(0.0, 0.0, 0.0);
  double threshold = 0.1;

  bool is_moving = autoware::trajectory_modifier::utils::is_ego_vehicle_moving(twist, threshold);
  EXPECT_FALSE(is_moving);
}

TEST_F(UtilsTest, IsEgoVehicleMovingBelowThreshold)
{
  auto twist = create_twist(0.05, 0.0, 0.0);
  double threshold = 0.1;

  bool is_moving = autoware::trajectory_modifier::utils::is_ego_vehicle_moving(twist, threshold);
  EXPECT_FALSE(is_moving);
}

TEST_F(UtilsTest, IsEgoVehicleMovingAtThreshold)
{
  auto twist = create_twist(0.1, 0.0, 0.0);
  double threshold = 0.1;

  bool is_moving = autoware::trajectory_modifier::utils::is_ego_vehicle_moving(twist, threshold);
  EXPECT_FALSE(is_moving);  // Equal to threshold, not greater
}

TEST_F(UtilsTest, IsEgoVehicleMovingAboveThreshold)
{
  auto twist = create_twist(0.15, 0.0, 0.0);
  double threshold = 0.1;

  bool is_moving = autoware::trajectory_modifier::utils::is_ego_vehicle_moving(twist, threshold);
  EXPECT_TRUE(is_moving);
}

TEST_F(UtilsTest, IsEgoVehicleMovingYAxisOnly)
{
  auto twist = create_twist(0.0, 0.15, 0.0);
  double threshold = 0.1;

  bool is_moving = autoware::trajectory_modifier::utils::is_ego_vehicle_moving(twist, threshold);
  EXPECT_TRUE(is_moving);
}

TEST_F(UtilsTest, IsEgoVehicleMovingZAxisOnly)
{
  auto twist = create_twist(0.0, 0.0, 0.15);
  double threshold = 0.1;

  bool is_moving = autoware::trajectory_modifier::utils::is_ego_vehicle_moving(twist, threshold);
  EXPECT_TRUE(is_moving);
}

TEST_F(UtilsTest, IsEgoVehicleMoving3DVelocity)
{
  auto twist = create_twist(0.06, 0.06, 0.06);
  double threshold = 0.1;

  // sqrt(0.06^2 + 0.06^2 + 0.06^2) = sqrt(0.0108) ≈ 0.104 > 0.1
  bool is_moving = autoware::trajectory_modifier::utils::is_ego_vehicle_moving(twist, threshold);
  EXPECT_TRUE(is_moving);
}

TEST_F(UtilsTest, IsEgoVehicleMoving3DVelocityBelow)
{
  auto twist = create_twist(0.05, 0.05, 0.05);
  double threshold = 0.1;

  // sqrt(0.05^2 + 0.05^2 + 0.05^2) = sqrt(0.0075) ≈ 0.087 < 0.1
  bool is_moving = autoware::trajectory_modifier::utils::is_ego_vehicle_moving(twist, threshold);
  EXPECT_FALSE(is_moving);
}

TEST_F(UtilsTest, IsEgoVehicleMovingHighThreshold)
{
  auto twist = create_twist(1.0, 0.0, 0.0);
  double threshold = 2.0;

  bool is_moving = autoware::trajectory_modifier::utils::is_ego_vehicle_moving(twist, threshold);
  EXPECT_FALSE(is_moving);
}

TEST_F(UtilsTest, IsEgoVehicleMovingNegativeVelocity)
{
  auto twist = create_twist(-0.15, 0.0, 0.0);
  double threshold = 0.1;

  bool is_moving = autoware::trajectory_modifier::utils::is_ego_vehicle_moving(twist, threshold);
  EXPECT_TRUE(is_moving);  // Magnitude is what matters
}

// Test replace_trajectory_with_stop_point function
TEST_F(UtilsTest, ReplaceTrajectoryWithStopPointEmptyTrajectory)
{
  TrajectoryPoints trajectory;
  auto ego_pose = create_pose(5.0, 10.0);

  autoware::trajectory_modifier::utils::replace_trajectory_with_stop_point(trajectory, ego_pose);

  EXPECT_EQ(trajectory.size(), 2);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.x, 5.0);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.y, 10.0);
  EXPECT_DOUBLE_EQ(trajectory[0].longitudinal_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].lateral_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].acceleration_mps2, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].heading_rate_rps, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].front_wheel_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].rear_wheel_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[1].pose.position.x, 5.0);
  EXPECT_DOUBLE_EQ(trajectory[1].pose.position.y, 10.0);
  EXPECT_DOUBLE_EQ(trajectory[1].longitudinal_velocity_mps, 0.0);
}

TEST_F(UtilsTest, ReplaceTrajectoryWithStopPointNonEmptyTrajectory)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(1.0, 1.0, 5.0));
  trajectory.push_back(create_trajectory_point(2.0, 2.0, 10.0));
  trajectory.push_back(create_trajectory_point(3.0, 3.0, 15.0));

  auto ego_pose = create_pose(7.0, 8.0);

  autoware::trajectory_modifier::utils::replace_trajectory_with_stop_point(trajectory, ego_pose);

  EXPECT_EQ(trajectory.size(), 2);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.x, 7.0);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.y, 8.0);
  EXPECT_DOUBLE_EQ(trajectory[0].longitudinal_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[1].pose.position.x, 7.0);
  EXPECT_DOUBLE_EQ(trajectory[1].pose.position.y, 8.0);
  EXPECT_DOUBLE_EQ(trajectory[1].longitudinal_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].lateral_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].acceleration_mps2, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].heading_rate_rps, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].front_wheel_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].rear_wheel_angle_rad, 0.0);
}

TEST_F(UtilsTest, ReplaceTrajectoryWithStopPointPoseOrientation)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(1.0, 1.0));

  auto ego_pose = create_pose(2.0, 3.0);
  ego_pose.orientation.x = 0.1;
  ego_pose.orientation.y = 0.2;
  ego_pose.orientation.z = 0.3;
  ego_pose.orientation.w = 0.9;

  autoware::trajectory_modifier::utils::replace_trajectory_with_stop_point(trajectory, ego_pose);

  EXPECT_EQ(trajectory.size(), 2);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.y, 3.0);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.orientation.y, 0.2);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.orientation.z, 0.3);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.orientation.w, 0.9);
  EXPECT_DOUBLE_EQ(trajectory[1].pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(trajectory[1].pose.position.y, 3.0);
  EXPECT_DOUBLE_EQ(trajectory[1].pose.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(trajectory[1].pose.orientation.y, 0.2);
  EXPECT_DOUBLE_EQ(trajectory[1].pose.orientation.z, 0.3);
  EXPECT_DOUBLE_EQ(trajectory[1].pose.orientation.w, 0.9);
}

TEST_F(UtilsTest, ReplaceTrajectoryWithStopPointNegativeCoordinates)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(10.0, 10.0));

  auto ego_pose = create_pose(-5.0, -7.5);

  autoware::trajectory_modifier::utils::replace_trajectory_with_stop_point(trajectory, ego_pose);

  EXPECT_EQ(trajectory.size(), 2);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.x, -5.0);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.y, -7.5);
  EXPECT_DOUBLE_EQ(trajectory[1].pose.position.x, -5.0);
  EXPECT_DOUBLE_EQ(trajectory[1].pose.position.y, -7.5);
}
