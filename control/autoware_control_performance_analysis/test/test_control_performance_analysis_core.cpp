// Copyright 2026 The Autoware Foundation
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

#include "autoware/control_performance_analysis/control_performance_analysis_core.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

namespace
{
using autoware::control_performance_analysis::ControlPerformanceAnalysisCore;
using autoware::control_performance_analysis::Params;
using autoware_control_msgs::msg::Control;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;

Params makeParams()
{
  Params p{};
  p.wheelbase_ = 2.7;
  p.curvature_interval_length_ = 10.0;
  p.odom_interval_ = 0;
  p.acceptable_max_distance_to_waypoint_ = 1.5;
  p.acceptable_max_yaw_difference_rad_ = 1.0472;
  p.prevent_zero_division_value_ = 0.001;
  p.lpf_gain_ = 0.8;
  return p;
}

Trajectory makeStraightTrajectory()
{
  Trajectory trajectory;
  for (int i = 0; i < 30; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = static_cast<double>(i);
    p.pose.orientation.w = 1.0;
    p.longitudinal_velocity_mps = 1.0;
    trajectory.points.push_back(p);
  }
  return trajectory;
}

Odometry makeOdometry(const int64_t nanoseconds)
{
  Odometry odom;
  odom.header.stamp = rclcpp::Time(nanoseconds);
  odom.pose.pose.orientation.w = 1.0;
  return odom;
}

Pose makePose(const double x, const double y)
{
  Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

void feedAllInputs(ControlPerformanceAnalysisCore & core, const Pose & ego_pose)
{
  core.setCurrentWaypoints(makeStraightTrajectory());
  core.setCurrentPose(ego_pose);
  core.setOdomHistory(makeOdometry(0));
  core.setOdomHistory(makeOdometry(100000000));
  core.setCurrentControlValue(Control{});
  core.setSteeringStatus(SteeringReport{});
}
}  // namespace

class ControlPerformanceAnalysisCoreTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

// https://github.com/autowarefoundation/autoware_universe/issues/4498
// Ego farther than acceptable_max_distance_to_waypoint_ from the trajectory before a nearest
// segment was ever found must not dereference the uninitialized waypoint index.
TEST_F(ControlPerformanceAnalysisCoreTest, ReturnsFalseWhenEgoIsFarFromTrajectory)
{
  auto params = makeParams();
  ControlPerformanceAnalysisCore core(params);
  feedAllInputs(core, makePose(5.0, 50.0));

  EXPECT_FALSE(core.calculateErrorVars());
  EXPECT_FALSE(core.calculateErrorVars());
}

TEST_F(ControlPerformanceAnalysisCoreTest, RecoversAfterEgoReturnsToTrajectory)
{
  auto params = makeParams();
  ControlPerformanceAnalysisCore core(params);
  feedAllInputs(core, makePose(5.0, 0.3));
  EXPECT_TRUE(core.calculateErrorVars());

  core.setCurrentPose(makePose(5.0, 50.0));
  EXPECT_FALSE(core.calculateErrorVars());

  core.setCurrentPose(makePose(6.0, 0.3));
  EXPECT_TRUE(core.calculateErrorVars());
}
