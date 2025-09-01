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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/stop_point_fixer.hpp"
#include "autoware/trajectory_modifier/utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::TrajectoryModifierParams;
using autoware::trajectory_modifier::plugin::StopPointFixer;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware_planning_msgs::msg::TrajectoryPoint;

class StopPointFixerIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();

    TrajectoryModifierParams params;
    plugin_ =
      std::make_unique<StopPointFixer>("test_stop_point_fixer", node_.get(), time_keeper_, params);
  }

  void TearDown() override
  {
    plugin_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

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

  TrajectoryModifierData create_data(double ego_x, double ego_y, double velocity)
  {
    TrajectoryModifierData data;
    data.current_odometry.pose.pose.position.x = ego_x;
    data.current_odometry.pose.pose.position.y = ego_y;
    data.current_odometry.pose.pose.position.z = 0.0;
    data.current_odometry.twist.twist.linear.x = velocity;
    data.current_odometry.twist.twist.linear.y = 0.0;
    data.current_odometry.twist.twist.linear.z = 0.0;
    return data;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::unique_ptr<StopPointFixer> plugin_;
};

// Test is_trajectory_modification_required method
TEST_F(StopPointFixerIntegrationTest, TrajectoryModificationNotRequiredWhenDisabled)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(10.0, 0.0));

  TrajectoryModifierParams params;
  params.use_stop_point_fixer = false;  // Disabled

  auto data = create_data(0.0, 0.0, 0.05);  // Stationary, close to target

  bool required = plugin_->is_trajectory_modification_required(trajectory, params, data);
  EXPECT_FALSE(required);
}

TEST_F(StopPointFixerIntegrationTest, TrajectoryModificationNotRequiredForEmptyTrajectory)
{
  TrajectoryPoints empty_trajectory;

  TrajectoryModifierParams params;
  params.use_stop_point_fixer = true;

  auto data = create_data(0.0, 0.0, 0.05);

  bool required = plugin_->is_trajectory_modification_required(empty_trajectory, params, data);
  EXPECT_FALSE(required);
}

TEST_F(StopPointFixerIntegrationTest, TrajectoryModificationNotRequiredWhenMoving)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.5, 0.0));  // Close point

  TrajectoryModifierParams params;
  params.use_stop_point_fixer = true;

  auto data = create_data(0.0, 0.0, 0.5);  // Moving fast

  bool required = plugin_->is_trajectory_modification_required(trajectory, params, data);
  EXPECT_FALSE(required);
}

TEST_F(StopPointFixerIntegrationTest, TrajectoryModificationNotRequiredWhenFarFromTarget)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(10.0, 0.0));  // Far point

  TrajectoryModifierParams params;
  params.use_stop_point_fixer = true;

  auto data = create_data(0.0, 0.0, 0.05);  // Stationary

  bool required = plugin_->is_trajectory_modification_required(trajectory, params, data);
  EXPECT_FALSE(required);  // Distance > default min_distance_threshold_m (1.0)
}

TEST_F(StopPointFixerIntegrationTest, TrajectoryModificationRequiredWhenStationaryAndClose)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.0, 0.0));  // Starting point
  trajectory.push_back(create_trajectory_point(0.5, 0.0));  // Close point

  TrajectoryModifierParams params;
  params.use_stop_point_fixer = true;

  auto data = create_data(0.0, 0.0, 0.05);  // Stationary

  bool required = plugin_->is_trajectory_modification_required(trajectory, params, data);
  EXPECT_TRUE(required);  // Distance < default min_distance_threshold_m (1.0)
}

TEST_F(StopPointFixerIntegrationTest, TrajectoryModificationBoundaryConditions)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(1.0, 0.0));  // Exactly at threshold distance

  TrajectoryModifierParams params;
  params.use_stop_point_fixer = true;

  auto data = create_data(0.0, 0.0, 0.1);  // Exactly at velocity threshold

  bool required = plugin_->is_trajectory_modification_required(trajectory, params, data);
  EXPECT_FALSE(required);  // Distance == threshold, velocity == threshold
}

// Test modify_trajectory method
TEST_F(StopPointFixerIntegrationTest, ModifyTrajectoryWhenRequired)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(1.0, 1.0, 5.0));
  trajectory.push_back(create_trajectory_point(2.0, 2.0, 10.0));
  trajectory.push_back(create_trajectory_point(0.5, 0.0, 15.0));  // Last point close to ego

  TrajectoryModifierParams params;
  params.use_stop_point_fixer = true;

  auto data = create_data(0.0, 0.0, 0.05);  // Stationary at origin

  // Verify modification is required first
  EXPECT_TRUE(plugin_->is_trajectory_modification_required(trajectory, params, data));

  plugin_->modify_trajectory(trajectory, params, data);

  // Should replace with two stop points at ego position (minimum for Control)
  EXPECT_EQ(trajectory.size(), 2);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].longitudinal_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].lateral_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].acceleration_mps2, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[1].pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[1].pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[1].longitudinal_velocity_mps, 0.0);
}

TEST_F(StopPointFixerIntegrationTest, ModifyTrajectoryWhenNotRequired)
{
  TrajectoryPoints original_trajectory;
  original_trajectory.push_back(create_trajectory_point(1.0, 1.0, 5.0));
  original_trajectory.push_back(create_trajectory_point(2.0, 2.0, 10.0));

  TrajectoryPoints trajectory = original_trajectory;  // Copy

  TrajectoryModifierParams params;
  params.use_stop_point_fixer = true;

  auto data = create_data(0.0, 0.0, 0.5);  // Moving

  // Verify modification is not required
  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory, params, data));

  plugin_->modify_trajectory(trajectory, params, data);

  // Trajectory should remain unchanged
  EXPECT_EQ(trajectory.size(), original_trajectory.size());
  for (size_t i = 0; i < trajectory.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory[i].pose.position.x, original_trajectory[i].pose.position.x);
    EXPECT_DOUBLE_EQ(trajectory[i].pose.position.y, original_trajectory[i].pose.position.y);
    EXPECT_DOUBLE_EQ(
      trajectory[i].longitudinal_velocity_mps, original_trajectory[i].longitudinal_velocity_mps);
  }
}

// Test parameter handling
TEST_F(StopPointFixerIntegrationTest, ParameterUpdateSuccess)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("stop_point_fixer.velocity_threshold_mps", 0.2);
  parameters.emplace_back("stop_point_fixer.min_distance_threshold_m", 2.0);

  auto result = plugin_->on_parameter(parameters);

  EXPECT_TRUE(result.successful);
  EXPECT_EQ(result.reason, "success");

  // Verify new parameters take effect
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(2.5, 0.0));  // Outside new distance threshold

  TrajectoryModifierParams params;
  params.use_stop_point_fixer = true;

  auto data = create_data(0.0, 0.0, 0.15);  // Below new velocity threshold (stationary)

  bool required = plugin_->is_trajectory_modification_required(trajectory, params, data);
  EXPECT_FALSE(required);  // Should use new thresholds - distance (2.5) > threshold (2.0)
}

TEST_F(StopPointFixerIntegrationTest, ParameterUpdateWithUnknownParameter)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("stop_point_fixer.velocity_threshold_mps", 0.2);
  parameters.emplace_back("unknown_parameter", 123);  // Unknown parameter

  auto result = plugin_->on_parameter(parameters);

  EXPECT_TRUE(result.successful);  // Should still succeed for known parameters
}

// Integration test with multiple trajectory points
TEST_F(StopPointFixerIntegrationTest, MultipleTrajectoryPointsUsesLastPoint)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(10.0, 10.0));  // Far first point
  trajectory.push_back(create_trajectory_point(5.0, 5.0));    // Medium distance
  trajectory.push_back(create_trajectory_point(0.3, 0.4));    // Close last point (0.5 distance)

  TrajectoryModifierParams params;
  params.use_stop_point_fixer = true;

  auto data = create_data(0.0, 0.0, 0.05);  // Stationary at origin

  bool required = plugin_->is_trajectory_modification_required(trajectory, params, data);
  EXPECT_TRUE(required);  // Should be true because last point is close
}

// Test with 3D velocity components
TEST_F(StopPointFixerIntegrationTest, ThreeDimensionalVelocity)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.5, 0.0));

  TrajectoryModifierParams params;
  params.use_stop_point_fixer = true;

  TrajectoryModifierData data;
  data.current_odometry.pose.pose.position.x = 0.0;
  data.current_odometry.pose.pose.position.y = 0.0;
  data.current_odometry.pose.pose.position.z = 0.0;
  data.current_odometry.twist.twist.linear.x = 0.06;
  data.current_odometry.twist.twist.linear.y = 0.06;
  data.current_odometry.twist.twist.linear.z = 0.06;  // 3D velocity magnitude > 0.1

  bool required = plugin_->is_trajectory_modification_required(trajectory, params, data);
  EXPECT_FALSE(required);  // Should detect vehicle as moving due to 3D velocity
}
