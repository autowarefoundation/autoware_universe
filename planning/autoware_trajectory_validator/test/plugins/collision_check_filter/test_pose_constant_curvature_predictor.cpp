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

#include "../../../src/filters/safety/collision_check_filter/trajectory_utils.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <cmath>

namespace autoware::trajectory_validator::plugin::safety::trajectory::pose::
  constant_curvature_predictor
{

TEST(PoseConstantCurvaturePredictorTest, PoseToIsometry)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, M_PI_2);
  pose.orientation = tf2::toMsg(q);

  const auto iso = detail::pose_to_isometry(pose);

  EXPECT_DOUBLE_EQ(iso.translation().x(), 1.0);
  EXPECT_DOUBLE_EQ(iso.translation().y(), 2.0);
  EXPECT_NEAR(iso.linear()(0, 0), 0.0, 1e-6);
  EXPECT_NEAR(iso.linear()(0, 1), -1.0, 1e-6);
  EXPECT_NEAR(iso.linear()(1, 0), 1.0, 1e-6);
  EXPECT_NEAR(iso.linear()(1, 1), 0.0, 1e-6);
}

TEST(PoseConstantCurvaturePredictorTest, ComputeTwistPerDistance)
{
  geometry_msgs::msg::Twist twist_normal;
  twist_normal.linear.x = 3.0;
  twist_normal.linear.y = 4.0;
  twist_normal.angular.z = 10.0;

  const auto res_normal = detail::compute_twist_per_distance(twist_normal);

  EXPECT_DOUBLE_EQ(res_normal.linear.x(), 3.0 / 5.0);
  EXPECT_DOUBLE_EQ(res_normal.linear.y(), 4.0 / 5.0);
  EXPECT_DOUBLE_EQ(res_normal.angular, 10.0 / 5.0);

  geometry_msgs::msg::Twist twist_stop;
  twist_stop.linear.x = 1e-7;
  twist_stop.linear.y = 0.0;
  twist_stop.angular.z = 5.0;

  const auto res_stop = detail::compute_twist_per_distance(twist_stop);

  EXPECT_NEAR(res_stop.linear.x(), 0.0, 1e-6);
  EXPECT_NEAR(res_stop.linear.y(), 0.0, 1e-6);
  EXPECT_NEAR(res_stop.angular, 0.0, 1e-6);
}

TEST(PoseConstantCurvaturePredictorTest, ComputeDeltaIsometry)
{
  {
    TwistPerDistance tpd_straight;
    tpd_straight.linear = Eigen::Vector2d(1.0, 0.0);
    tpd_straight.angular = 0.0;

    const auto iso_straight = detail::compute_delta_isometry(tpd_straight, 2.0);
    EXPECT_NEAR(iso_straight.translation().x(), 2.0, 1e-6);
    EXPECT_NEAR(iso_straight.translation().y(), 0.0, 1e-6);
    EXPECT_NEAR(Eigen::Rotation2Dd(iso_straight.linear()).angle(), 0.0, 1e-6);
  }

  {
    TwistPerDistance tpd_curve;
    tpd_curve.linear = Eigen::Vector2d(1.0, 0.0);
    tpd_curve.angular = M_PI_2;

    const auto iso_curve = detail::compute_delta_isometry(tpd_curve, 1.0);
    EXPECT_NEAR(iso_curve.translation().x(), 1.0 / M_PI_2, 1e-3);
    EXPECT_NEAR(iso_curve.translation().y(), 1.0 / M_PI_2, 1e-3);
    EXPECT_NEAR(Eigen::Rotation2Dd(iso_curve.linear()).angle(), M_PI_2, 1e-3);
  }
}

TEST(PoseConstantCurvaturePredictorTest, IsometryToPose)
{
  Eigen::Isometry2d iso = Eigen::Isometry2d::Identity();
  iso.translation() << 3.0, 4.0;
  iso.linear() = Eigen::Rotation2Dd(M_PI).toRotationMatrix();

  const double initial_z = 5.0;
  const auto pose = detail::isometry_to_pose(iso, initial_z);

  EXPECT_DOUBLE_EQ(pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 4.0);
  EXPECT_DOUBLE_EQ(pose.position.z, 5.0);

  const double yaw = tf2::getYaw(pose.orientation);
  EXPECT_NEAR(yaw, M_PI, 1e-5);
}

TEST(PoseConstantCurvaturePredictorTest, ComputeTrajectory)
{
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position.x = 1000.0;
  initial_pose.position.y = 2000.0;
  initial_pose.position.z = 3000.0;
  tf2::Quaternion q;
  q.setRPY(0.1, 0.2, M_PI_2);
  initial_pose.orientation = tf2::toMsg(q);

  geometry_msgs::msg::Twist initial_twist;
  initial_twist.linear.x = 0.8;
  initial_twist.linear.y = 0.6;
  initial_twist.angular.z = M_PI_4;

  TravelDistanceTrajectory distance_trajectory = {0.0, 1.0, 2.0};

  const auto curvature_pose_trajectory = compute(initial_pose, initial_twist, distance_trajectory);

  ASSERT_EQ(curvature_pose_trajectory.size(), 3u);
  {
    const auto pose = curvature_pose_trajectory[0];
    EXPECT_DOUBLE_EQ(pose.position.x, 1000.0);
    EXPECT_DOUBLE_EQ(pose.position.y, 2000.0);
    EXPECT_DOUBLE_EQ(pose.position.z, 3000.0);
    EXPECT_NEAR(tf2::getYaw(pose.orientation), M_PI_2, 1e-5);
  }
  {
    const auto pose = curvature_pose_trajectory[1];
    EXPECT_DOUBLE_EQ(pose.position.z, 3000.0);
    EXPECT_NEAR(tf2::getYaw(pose.orientation), M_PI_2 + M_PI_4, 1e-3);
  }
  {
    const auto pose = curvature_pose_trajectory[2];
    EXPECT_DOUBLE_EQ(pose.position.z, 3000.0);
    EXPECT_NEAR(tf2::getYaw(pose.orientation), M_PI_2 + M_PI_4 * 2.0, 1e-3);
  }

  initial_twist.linear.x = 1.0;
  initial_twist.linear.y = 0.0;
  initial_twist.angular.z = 0.0;

  const auto straight_pose_trajectory = compute(initial_pose, initial_twist, distance_trajectory);

  ASSERT_EQ(straight_pose_trajectory.size(), 3u);
  {
    const auto pose = straight_pose_trajectory[0];
    EXPECT_NEAR(pose.position.x, 1000.0, 1e-5);
    EXPECT_NEAR(pose.position.y, 2000.0, 1e-5);
    EXPECT_NEAR(pose.position.z, 3000.0, 1e-5);
    EXPECT_NEAR(tf2::getYaw(pose.orientation), M_PI_2, 1e-5);
  }
  {
    const auto pose = straight_pose_trajectory[1];
    EXPECT_NEAR(pose.position.x, 1000.0, 1e-5);
    EXPECT_NEAR(pose.position.y, 2001.0, 1e-5);
    EXPECT_NEAR(pose.position.z, 3000.0, 1e-5);
    EXPECT_NEAR(tf2::getYaw(pose.orientation), M_PI_2, 1e-3);
  }
  {
    const auto pose = straight_pose_trajectory[2];
    EXPECT_NEAR(pose.position.x, 1000.0, 1e-5);
    EXPECT_NEAR(pose.position.y, 2002.0, 1e-5);
    EXPECT_NEAR(pose.position.z, 3000.0, 1e-5);
    EXPECT_NEAR(tf2::getYaw(pose.orientation), M_PI_2, 1e-3);
  }

  double r = 1.0 / M_PI_4;
  initial_pose.position.x = r / sqrt(2.0);
  initial_pose.position.y = r / sqrt(2.0);
  initial_pose.position.z = 3000.0;
  q.setRPY(0.1, 0.2, 3.0 * M_PI_4);
  initial_pose.orientation = tf2::toMsg(q);

  initial_twist.linear.x = 1.0;
  initial_twist.linear.y = 0.0;
  initial_twist.angular.z = M_PI_4;

  const auto circular_pose_trajectory = compute(initial_pose, initial_twist, distance_trajectory);
  {
    const auto pose = circular_pose_trajectory[0];
    EXPECT_DOUBLE_EQ(pose.position.x, r / sqrt(2.0));
    EXPECT_DOUBLE_EQ(pose.position.y, r / sqrt(2.0));
    EXPECT_DOUBLE_EQ(pose.position.z, 3000.0);
    EXPECT_NEAR(tf2::getYaw(pose.orientation), 3.0 * M_PI_4, 1e-3);
  }
  {
    const auto pose = circular_pose_trajectory[1];
    EXPECT_NEAR(pose.position.x, 0.0, 1e-5);
    EXPECT_NEAR(pose.position.y, r, 1e-5);
    EXPECT_NEAR(pose.position.z, 3000.0, 1e-5);
    EXPECT_NEAR(tf2::getYaw(pose.orientation), 4.0 * M_PI_4, 1e-3);
  }
  {
    const auto pose = circular_pose_trajectory[2];
    EXPECT_NEAR(pose.position.x, -r / sqrt(2.0), 1e-5);
    EXPECT_NEAR(pose.position.y, r / sqrt(2.0), 1e-5);
    EXPECT_NEAR(pose.position.z, 3000.0, 1e-5);
    EXPECT_NEAR(
      tf2NormalizeAngle(tf2::getYaw(pose.orientation)), tf2NormalizeAngle(5.0 * M_PI_4), 1e-3);
  }
}

}  // namespace
   // autoware::trajectory_validator::plugin::safety::trajectory::pose::constant_curvature_predictor
