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

#include "autoware/trajectory_ranker/metrics/metrics_utils.hpp"

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

namespace autoware::trajectory_ranker::metrics::utils
{

using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

class TestMetricsUtils : public ::testing::Test
{
protected:
  void SetUp() override {}
};

TEST_F(TestMetricsUtils, TransformToRelativeCoordinate2D)
{
  Point point;
  point.x = 10.0;
  point.y = 5.0;
  point.z = 0.0;

  Pose origin;
  origin.position.x = 5.0;
  origin.position.y = 5.0;
  origin.position.z = 0.0;
  origin.orientation.x = 0.0;
  origin.orientation.y = 0.0;
  origin.orientation.z = 0.0;
  origin.orientation.w = 1.0;  // No rotation

  auto result = transform_to_relative_coordinate2_d(point, origin);

  // Point (10,5) relative to origin (5,5) with no rotation should be (5,0)
  EXPECT_NEAR(result.x, 5.0, 1e-6);
  EXPECT_NEAR(result.y, 0.0, 1e-6);
}

TEST_F(TestMetricsUtils, TransformToRelativeCoordinate2D_WithRotation)
{
  // Test with a point that after translation is at (0,5) in world frame
  // Origin is rotated 45 degrees
  Point point;
  point.x = 5.0;
  point.y = 10.0;

  Pose origin;
  origin.position.x = 5.0;
  origin.position.y = 5.0;
  origin.orientation.x = 0.0;
  origin.orientation.y = 0.0;
  // For 45 degree rotation, quaternion uses half angle
  origin.orientation.z = std::sin(M_PI / 8.0);  // sin(22.5 degrees)
  origin.orientation.w = std::cos(M_PI / 8.0);  // cos(22.5 degrees)

  auto result = transform_to_relative_coordinate2_d(point, origin);

  // After translation: (0,5) in world frame
  // The function rotates by yaw angle (45 degrees) to transform to local frame
  // x' = cos(45) * 0 + sin(45) * 5 = 3.536
  // y' = -sin(45) * 0 + cos(45) * 5 = 3.536
  EXPECT_NEAR(result.x, 5.0 / std::sqrt(2.0), 1e-6);
  EXPECT_NEAR(result.y, 5.0 / std::sqrt(2.0), 1e-6);
}

TEST_F(TestMetricsUtils, CalcRadius_StraightPath)
{
  Point target;
  target.x = 10.0;
  target.y = 0.0;

  Pose current_pose;
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.orientation.w = 1.0;  // Facing +X direction

  float radius = calc_radius(target, current_pose);

  // For a straight path, radius should be very large
  EXPECT_GT(radius, 1e8);
}

TEST_F(TestMetricsUtils, CalcRadius_CircularPath)
{
  Point target;
  target.x = 5.0;
  target.y = 5.0;

  Pose current_pose;
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.orientation.w = 1.0;

  float radius = calc_radius(target, current_pose);

  // For this configuration, we can calculate expected radius
  // The radius should be positive and finite
  EXPECT_GT(radius, 0.0);
  EXPECT_LT(radius, 1e8);
}

TEST_F(TestMetricsUtils, Curvature_StraightPath)
{
  Point target;
  target.x = 10.0;
  target.y = 0.0;

  Pose current_pose;
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.orientation.w = 1.0;

  float kappa = curvature(target, current_pose);

  // For a straight path, curvature should be nearly zero
  EXPECT_NEAR(kappa, 0.0, 1e-6);
}

TEST_F(TestMetricsUtils, PurePursuit_BasicTest)
{
  auto points = std::make_shared<TrajectoryPoints>();

  // Create a straight trajectory
  for (int i = 0; i < 20; ++i) {
    TrajectoryPoint pt;
    pt.pose.position.x = static_cast<float>(i);
    pt.pose.position.y = 0.0;
    pt.pose.orientation.w = 1.0;
    points->push_back(pt);
  }

  Pose ego_pose;
  ego_pose.position.x = 0.0;
  ego_pose.position.y = 0.0;
  ego_pose.orientation.w = 1.0;

  float kappa = pure_pursuit(points, ego_pose);

  // For a straight trajectory, pure pursuit should return near-zero curvature
  EXPECT_NEAR(kappa, 0.0, 1e-6);
}

TEST_F(TestMetricsUtils, SteerCommand_StraightPath)
{
  auto points = std::make_shared<TrajectoryPoints>();

  // Create a straight trajectory
  for (int i = 0; i < 20; ++i) {
    TrajectoryPoint pt;
    pt.pose.position.x = static_cast<float>(i);
    pt.pose.position.y = 0.0;
    pt.pose.orientation.w = 1.0;
    points->push_back(pt);
  }

  Pose ego_pose;
  ego_pose.position.x = 0.0;
  ego_pose.position.y = 0.0;
  ego_pose.orientation.w = 1.0;

  float wheel_base = 2.5f;
  float steer = steer_command(points, ego_pose, wheel_base);

  // For a straight path, steering command should be near zero
  EXPECT_NEAR(steer, 0.0, 1e-6);
}

TEST_F(TestMetricsUtils, TimeToCollision_BetweenPoints)
{
  TrajectoryPoint pt1;
  pt1.pose.position.x = 0.0;
  pt1.pose.position.y = 0.0;
  pt1.pose.position.z = 0.0;
  pt1.pose.orientation.w = 1.0;          // Facing +X
  pt1.longitudinal_velocity_mps = 10.0;  // Moving at 10 m/s forward
  pt1.lateral_velocity_mps = 0.0;

  TrajectoryPoint pt2;
  pt2.pose.position.x = 100.0;  // 100m ahead
  pt2.pose.position.y = 0.0;
  pt2.pose.position.z = 0.0;
  pt2.pose.orientation.x = 0.0;
  pt2.pose.orientation.y = 0.0;
  pt2.pose.orientation.z = 1.0;  // Facing -X (opposite direction)
  pt2.pose.orientation.w = 0.0;
  pt2.longitudinal_velocity_mps = 10.0;  // Moving at 10 m/s in opposite direction
  pt2.lateral_velocity_mps = 0.0;

  float ttc = time_to_collision(pt1, pt2);

  // Two objects 100m apart approaching at relative speed of 20 m/s
  // TTC should be 5 seconds
  EXPECT_NEAR(ttc, 5.0, 1e-6);
}

TEST_F(TestMetricsUtils, TimeToCollision_NoCollision)
{
  TrajectoryPoint pt1;
  pt1.pose.position.x = 0.0;
  pt1.pose.position.y = 0.0;
  pt1.pose.orientation.w = 1.0;
  pt1.longitudinal_velocity_mps = 10.0;  // Moving away
  pt1.lateral_velocity_mps = 0.0;

  TrajectoryPoint pt2;
  pt2.pose.position.x = 100.0;
  pt2.pose.position.y = 0.0;
  pt2.pose.orientation.w = 1.0;          // Same direction
  pt2.longitudinal_velocity_mps = 10.0;  // Same speed
  pt2.lateral_velocity_mps = 0.0;

  float ttc = time_to_collision(pt1, pt2);

  // Moving in same direction at same speed, no collision
  EXPECT_TRUE(std::isinf(ttc));
}

TEST_F(TestMetricsUtils, TimeToCollision_WithObjects_NoObjects)
{
  auto points = std::make_shared<TrajectoryPoints>();
  TrajectoryPoint pt;
  pt.pose.position.x = 0.0;
  pt.pose.position.y = 0.0;
  pt.pose.orientation.w = 1.0;
  pt.time_from_start = rclcpp::Duration::from_seconds(0.0);
  points->push_back(pt);

  auto objects = std::make_shared<PredictedObjects>();  // Empty
  float max_ttc_value = 10.0f;

  float ttc = time_to_collision(points, objects, 0, max_ttc_value);

  // No objects, should return max TTC value (10.0)
  EXPECT_NEAR(ttc, max_ttc_value, 1e-6);
}

TEST_F(TestMetricsUtils, TimeToCollision_WithObjects_BasicCollision)
{
  auto points = std::make_shared<TrajectoryPoints>();
  TrajectoryPoint ego_pt;
  ego_pt.pose.position.x = 0.0;
  ego_pt.pose.position.y = 0.0;
  ego_pt.pose.orientation.w = 1.0;
  ego_pt.longitudinal_velocity_mps = 10.0;
  ego_pt.lateral_velocity_mps = 0.0;
  ego_pt.time_from_start = rclcpp::Duration::from_seconds(0.0);
  points->push_back(ego_pt);

  auto objects = std::make_shared<PredictedObjects>();
  PredictedObject obj;
  obj.kinematics.initial_pose_with_covariance.pose.position.x = 50.0;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  obj.kinematics.initial_pose_with_covariance.pose.orientation.z = 1.0;  // Facing opposite
  obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 0.0;
  obj.kinematics.initial_twist_with_covariance.twist.linear.x = 10.0;  // Moving opposite
  obj.kinematics.initial_twist_with_covariance.twist.linear.y = 0.0;

  // Create a simple predicted path
  autoware_perception_msgs::msg::PredictedPath path;
  path.confidence = 1.0;
  path.time_step = rclcpp::Duration::from_seconds(0.1);

  geometry_msgs::msg::Pose path_pose;
  path_pose.position.x = 50.0;
  path_pose.position.y = 0.0;
  path_pose.orientation = obj.kinematics.initial_pose_with_covariance.pose.orientation;
  path.path.push_back(path_pose);

  path_pose.position.x = 49.0;  // Moving towards ego
  path.path.push_back(path_pose);

  obj.kinematics.predicted_paths.push_back(path);
  objects->objects.push_back(obj);

  float max_ttc_value = 10.0f;
  float ttc = time_to_collision(points, objects, 0, max_ttc_value);

  // Should calculate collision time between ego and object
  EXPECT_GT(ttc, 0.0);
  EXPECT_LE(ttc, max_ttc_value);  // Capped at max value
}

}  // namespace autoware::trajectory_ranker::metrics::utils

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
