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

#include "autoware/trajectory_ranker/utils.hpp"

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace autoware::trajectory_ranker::utils
{

using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;

class TestUtils : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create sample trajectory points
    createSampleTrajectory();
  }

  void createSampleTrajectory()
  {
    sample_points_.clear();
    for (size_t i = 0; i < 10; ++i) {
      TrajectoryPoint pt;
      pt.pose.position.x = static_cast<double>(i) * 1.0;
      pt.pose.position.y = 0.0;
      pt.pose.position.z = 0.0;
      pt.pose.orientation.x = 0.0;
      pt.pose.orientation.y = 0.0;
      pt.pose.orientation.z = 0.0;
      pt.pose.orientation.w = 1.0;
      pt.longitudinal_velocity_mps = 5.0;
      pt.lateral_velocity_mps = 0.0;
      pt.acceleration_mps2 = 0.0;
      pt.heading_rate_rps = 0.0;
      pt.front_wheel_angle_rad = 0.0;
      pt.rear_wheel_angle_rad = 0.0;
      pt.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(i) * 0.5);
      sample_points_.push_back(pt);
    }
  }

  TrajectoryPoints sample_points_;
};

TEST_F(TestUtils, CalcInterpolatedPoint_BasicInterpolation)
{
  TrajectoryPoint pt1;
  pt1.pose.position.x = 0.0;
  pt1.pose.position.y = 0.0;
  pt1.pose.position.z = 0.0;
  pt1.pose.orientation.w = 1.0;
  pt1.longitudinal_velocity_mps = 10.0;
  pt1.lateral_velocity_mps = 0.0;
  pt1.time_from_start = rclcpp::Duration::from_seconds(0.0);

  TrajectoryPoint pt2;
  pt2.pose.position.x = 10.0;
  pt2.pose.position.y = 0.0;
  pt2.pose.position.z = 0.0;
  pt2.pose.orientation.w = 1.0;
  pt2.longitudinal_velocity_mps = 20.0;
  pt2.lateral_velocity_mps = 0.0;
  pt2.time_from_start = rclcpp::Duration::from_seconds(1.0);

  // Test interpolation at ratio 0.5
  auto result = calc_interpolated_point(pt1, pt2, 0.5, false);
  EXPECT_NEAR(result.pose.position.x, 5.0, 1e-6);
  EXPECT_NEAR(result.longitudinal_velocity_mps, 15.0, 1e-6);
  EXPECT_NEAR(rclcpp::Duration(result.time_from_start).seconds(), 0.5, 1e-6);

  // Test interpolation at ratio 0.0 (should return pt1)
  result = calc_interpolated_point(pt1, pt2, 0.0, false);
  EXPECT_NEAR(result.pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(result.longitudinal_velocity_mps, 10.0, 1e-6);

  // Test interpolation at ratio 1.0 (should return pt2)
  result = calc_interpolated_point(pt1, pt2, 1.0, false);
  EXPECT_NEAR(result.pose.position.x, 10.0, 1e-6);
  EXPECT_NEAR(result.longitudinal_velocity_mps, 20.0, 1e-6);
}

TEST_F(TestUtils, CalcInterpolatedPoint_ZeroOrderHold)
{
  TrajectoryPoint pt1;
  pt1.longitudinal_velocity_mps = 10.0;
  pt1.lateral_velocity_mps = 1.0;
  pt1.acceleration_mps2 = 2.0;
  pt1.pose.orientation.w = 1.0;
  pt1.time_from_start = rclcpp::Duration::from_seconds(0.0);

  TrajectoryPoint pt2;
  pt2.longitudinal_velocity_mps = 20.0;
  pt2.lateral_velocity_mps = 2.0;
  pt2.acceleration_mps2 = 4.0;
  pt2.pose.orientation.w = 1.0;
  pt2.time_from_start = rclcpp::Duration::from_seconds(1.0);

  // Test with zero-order hold for twist
  auto result = calc_interpolated_point(pt1, pt2, 0.5, true);
  EXPECT_NEAR(result.longitudinal_velocity_mps, 10.0, 1e-6);  // Should keep pt1's velocity
  EXPECT_NEAR(result.lateral_velocity_mps, 1.0, 1e-6);
  EXPECT_NEAR(result.acceleration_mps2, 2.0, 1e-6);
}

TEST_F(TestUtils, Sampling_BasicSampling)
{
  Pose ego_pose;
  ego_pose.position.x = 0.0;
  ego_pose.position.y = 0.0;
  ego_pose.position.z = 0.0;
  ego_pose.orientation.w = 1.0;

  const size_t sample_num = 5;
  const double resolution = 0.5;  // 0.5 seconds

  auto sampled = sampling(sample_points_, ego_pose, sample_num, resolution);

  EXPECT_EQ(sampled.size(), sample_num);

  // Check time intervals
  for (size_t i = 1; i < sampled.size(); ++i) {
    double dt = rclcpp::Duration(sampled[i].time_from_start).seconds() -
                rclcpp::Duration(sampled[i - 1].time_from_start).seconds();
    EXPECT_NEAR(dt, resolution, 1e-6);
  }
}

TEST_F(TestUtils, Sampling_EmptyInput)
{
  Pose ego_pose;
  ego_pose.orientation.w = 1.0;
  TrajectoryPoints empty_points;

  auto sampled = sampling(empty_points, ego_pose, 5, 0.5);
  EXPECT_EQ(sampled.size(), 0);
}

TEST_F(TestUtils, Sampling_ZeroSamples)
{
  Pose ego_pose;
  ego_pose.orientation.w = 1.0;

  auto sampled = sampling(sample_points_, ego_pose, 0, 0.5);
  EXPECT_EQ(sampled.size(), 0);
}

TEST_F(TestUtils, Sampling_EgoFarFromTrajectory)
{
  Pose ego_pose;
  ego_pose.position.x = 100.0;  // Far from trajectory
  ego_pose.position.y = 100.0;
  ego_pose.orientation.w = 1.0;

  auto sampled = sampling(sample_points_, ego_pose, 5, 0.5);
  // Should return empty if ego is too far from trajectory
  EXPECT_EQ(sampled.size(), 0);
}

}  // namespace autoware::trajectory_ranker::utils

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
