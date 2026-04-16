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
#include "autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::plugin::StopPointFixer;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware_planning_msgs::msg::TrajectoryPoint;

class StopPointFixerIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    auto node_options = rclcpp::NodeOptions{};
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    autoware::test_utils::updateNodeOptions(
      node_options, {autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<rclcpp::Node>("test_node", node_options);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();
    data_ = std::make_shared<TrajectoryModifierData>(node_.get());
    params_.use_stop_point_fixer = true;
    params_.stop_point_fixer.velocity_threshold = 0.1;
    params_.stop_point_fixer.min_distance_threshold = 1.0;
    plugin_ = std::make_unique<StopPointFixer>();
    plugin_->initialize("test_stop_point_fixer", node_.get(), time_keeper_, data_, params_);
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

  void set_odometry_data(double ego_x, double ego_y, double velocity)
  {
    nav_msgs::msg::Odometry current_odometry;
    current_odometry.pose.pose.position.x = ego_x;
    current_odometry.pose.pose.position.y = ego_y;
    current_odometry.pose.pose.position.z = 0.0;
    current_odometry.twist.twist.linear.x = velocity;
    current_odometry.twist.twist.linear.y = 0.0;
    current_odometry.twist.twist.linear.z = 0.0;
    data_->current_odometry = std::make_shared<nav_msgs::msg::Odometry>(current_odometry);
  }

  TrajectoryPoint create_trajectory_point_with_time(
    double x, double y, double velocity, int32_t sec, uint32_t nanosec = 0)
  {
    auto point = create_trajectory_point(x, y, velocity);
    point.time_from_start.sec = sec;
    point.time_from_start.nanosec = nanosec;
    return point;
  }

  TrajectoryPoint create_trajectory_point_with_duration(
    double x, double y, double velocity, double time_s)
  {
    const auto sec = static_cast<int32_t>(time_s);
    const auto nanosec = static_cast<uint32_t>((time_s - static_cast<double>(sec)) * 1e9);
    return create_trajectory_point_with_time(x, y, velocity, sec, nanosec);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::unique_ptr<StopPointFixer> plugin_;
  trajectory_modifier_params::Params params_;
  std::shared_ptr<TrajectoryModifierData> data_;
};

// Test is_trajectory_modification_required method
TEST_F(StopPointFixerIntegrationTest, TrajectoryModificationNotRequiredWhenDisabled)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(10.0, 0.0));

  params_.use_stop_point_fixer = false;  // Disabled
  plugin_->update_params(params_);

  set_odometry_data(0.0, 0.0, 0.05);  // Stationary, close to target

  bool required = plugin_->is_trajectory_modification_required(trajectory);
  EXPECT_FALSE(required);
}

TEST_F(StopPointFixerIntegrationTest, TrajectoryModificationNotRequiredForEmptyTrajectory)
{
  TrajectoryPoints empty_trajectory;

  set_odometry_data(0.0, 0.0, 0.05);

  bool required = plugin_->is_trajectory_modification_required(empty_trajectory);
  EXPECT_FALSE(required);
}

TEST_F(StopPointFixerIntegrationTest, TrajectoryModificationNotRequiredWhenMoving)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.5, 0.0));  // Close point

  set_odometry_data(0.0, 0.0, 0.5);  // Moving fast

  bool required = plugin_->is_trajectory_modification_required(trajectory);
  EXPECT_FALSE(required);
}

TEST_F(StopPointFixerIntegrationTest, TrajectoryModificationNotRequiredWhenFarFromTarget)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(10.0, 0.0));  // Far point

  set_odometry_data(0.0, 0.0, 0.05);  // Stationary

  bool required = plugin_->is_trajectory_modification_required(trajectory);
  EXPECT_FALSE(required);  // Distance > default min_distance_threshold_m (1.0)
}

TEST_F(StopPointFixerIntegrationTest, TrajectoryModificationRequiredWhenStationaryAndClose)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.0, 0.0));  // Starting point
  trajectory.push_back(create_trajectory_point(0.5, 0.0));  // Close point

  set_odometry_data(0.0, 0.0, 0.05);  // Stationary

  bool required = plugin_->is_trajectory_modification_required(trajectory);
  EXPECT_TRUE(required);  // Distance < default min_distance_threshold_m (1.0)
}

TEST_F(StopPointFixerIntegrationTest, TrajectoryModificationBoundaryConditions)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(1.0, 0.0));  // Exactly at threshold distance

  set_odometry_data(0.0, 0.0, 0.1);  // Exactly at velocity threshold

  bool required = plugin_->is_trajectory_modification_required(trajectory);
  EXPECT_FALSE(required);  // Distance == threshold, velocity == threshold
}

// Test modify_trajectory method
TEST_F(StopPointFixerIntegrationTest, ModifyTrajectoryWhenRequired)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(1.0, 1.0, 5.0));
  trajectory.push_back(create_trajectory_point(2.0, 2.0, 10.0));
  trajectory.push_back(create_trajectory_point(0.5, 0.0, 15.0));  // Last point close to ego

  set_odometry_data(0.0, 0.0, 0.05);  // Stationary at origin

  // Verify modification is required first
  EXPECT_TRUE(plugin_->is_trajectory_modification_required(trajectory));

  plugin_->modify_trajectory(trajectory);

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

  set_odometry_data(0.0, 0.0, 0.5);  // Moving

  // Verify modification is not required
  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory));

  plugin_->modify_trajectory(trajectory);

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
  params_.stop_point_fixer.velocity_threshold = 0.2;
  params_.stop_point_fixer.min_distance_threshold = 2.0;

  plugin_->update_params(params_);

  EXPECT_FLOAT_EQ(
    plugin_->get_params().velocity_threshold, params_.stop_point_fixer.velocity_threshold);
  EXPECT_FLOAT_EQ(
    plugin_->get_params().min_distance_threshold, params_.stop_point_fixer.min_distance_threshold);

  // Verify new parameters take effect
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(2.5, 0.0));  // Outside new distance threshold

  set_odometry_data(0.0, 0.0, 0.15);  // Below new velocity threshold (stationary)

  bool required = plugin_->is_trajectory_modification_required(trajectory);
  EXPECT_FALSE(required);  // Should use new thresholds - distance (2.5) > threshold (2.0)
}

// Integration test with multiple trajectory points
TEST_F(StopPointFixerIntegrationTest, MultipleTrajectoryPointsUsesLastPoint)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(10.0, 10.0));  // Far first point
  trajectory.push_back(create_trajectory_point(5.0, 5.0));    // Medium distance
  trajectory.push_back(create_trajectory_point(0.3, 0.4));    // Close last point (0.5 distance)

  set_odometry_data(0.0, 0.0, 0.05);  // Stationary at origin

  bool required = plugin_->is_trajectory_modification_required(trajectory);
  EXPECT_TRUE(required);  // Should be true because last point is close
}

// Test with 3D velocity components
TEST_F(StopPointFixerIntegrationTest, ThreeDimensionalVelocity)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.5, 0.0));

  nav_msgs::msg::Odometry current_odometry;
  current_odometry.pose.pose.position.x = 0.0;
  current_odometry.pose.pose.position.y = 0.0;
  current_odometry.pose.pose.position.z = 0.0;
  current_odometry.twist.twist.linear.x = 0.06;
  current_odometry.twist.twist.linear.y = 0.06;
  current_odometry.twist.twist.linear.z = 0.06;  // 3D velocity magnitude > 0.1
  data_->current_odometry = std::make_shared<nav_msgs::msg::Odometry>(current_odometry);

  bool required = plugin_->is_trajectory_modification_required(trajectory);
  EXPECT_FALSE(required);  // Should detect vehicle as moving due to 3D velocity
}

// Tests for is_long_stop_trajectory

TEST_F(
  StopPointFixerIntegrationTest, IsLongStopTrajectory_ReturnsTrueWhenTimeExceedsMinStopDuration)
{
  auto min_stop_duration_s = params_.stop_point_fixer.min_stop_duration;
  TrajectoryPoints trajectory;
  trajectory.push_back(
    create_trajectory_point_with_duration(5.0, 0.0, 0.0, min_stop_duration_s * 2.0));

  EXPECT_TRUE(plugin_->is_long_stop_trajectory(trajectory));
}

TEST_F(StopPointFixerIntegrationTest, IsLongStopTrajectory_ReturnsFalseWhenMovingPointEncountered)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point_with_time(5.0, 0.0, 1.0, 0));

  EXPECT_FALSE(plugin_->is_long_stop_trajectory(trajectory));
}

TEST_F(StopPointFixerIntegrationTest, IsLongStopTrajectory_ReturnsFalseForEmptyTrajectory)
{
  TrajectoryPoints trajectory;
  EXPECT_FALSE(plugin_->is_long_stop_trajectory(trajectory));
}

TEST_F(StopPointFixerIntegrationTest, IsLongStopTrajectory_ReturnsTrueWhenAllPointsStoppedNoTime)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point_with_time(5.0, 0.0, 0.0, 0));
  trajectory.push_back(create_trajectory_point_with_time(6.0, 0.0, 0.0, 0));

  EXPECT_TRUE(plugin_->is_long_stop_trajectory(trajectory));
}

TEST_F(
  StopPointFixerIntegrationTest, IsLongStopTrajectory_MovingPointBeforeStoppedLongPointReturnsFalse)
{
  auto min_stop_duration_s = params_.stop_point_fixer.min_stop_duration;
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point_with_duration(2.0, 0.0, 1.0, 0.0));
  trajectory.push_back(
    create_trajectory_point_with_duration(5.0, 0.0, 0.0, min_stop_duration_s * 2.0));

  EXPECT_FALSE(plugin_->is_long_stop_trajectory(trajectory));
}

// Tests for force_stop_long_stopped_trajectories flag

TEST_F(StopPointFixerIntegrationTest, ForceLongStopFlag_False_LongStopConditionDoesNotTrigger)
{
  std::vector<rclcpp::Parameter> parameters;
  parameters.emplace_back("stop_point_fixer.force_stop_long_stopped_trajectories", false);
  params_.stop_point_fixer.force_stop_long_stopped_trajectories = false;
  params_.use_stop_point_fixer = true;
  plugin_->update_params(params_);

  auto min_stop_duration_s = params_.stop_point_fixer.min_stop_duration;
  TrajectoryPoints trajectory;
  trajectory.push_back(
    create_trajectory_point_with_duration(5.0, 0.0, 0.0, min_stop_duration_s * 2.0));

  set_odometry_data(0.0, 0.0, 0.05);

  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory));
}

TEST_F(StopPointFixerIntegrationTest, ForceLongStopFlag_True_LongStopConditionTriggers)
{
  params_.stop_point_fixer.force_stop_long_stopped_trajectories = true;
  params_.stop_point_fixer.force_stop_close_stopped_trajectories = false;
  params_.use_stop_point_fixer = true;
  plugin_->update_params(params_);

  auto min_stop_duration_s = params_.stop_point_fixer.min_stop_duration;
  TrajectoryPoints trajectory;
  trajectory.push_back(
    create_trajectory_point_with_duration(5.0, 0.0, 0.0, min_stop_duration_s * 2.0));

  set_odometry_data(0.0, 0.0, 0.05);

  EXPECT_TRUE(plugin_->is_trajectory_modification_required(trajectory));
}

// Tests for force_stop_close_stopped_trajectories flag

TEST_F(StopPointFixerIntegrationTest, ForceCloseStopFlag_False_CloseStopConditionDoesNotTrigger)
{
  params_.stop_point_fixer.force_stop_long_stopped_trajectories = true;
  params_.stop_point_fixer.force_stop_close_stopped_trajectories = false;
  params_.use_stop_point_fixer = true;
  plugin_->update_params(params_);

  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point_with_time(0.5, 0.0, 1.0, 0));

  set_odometry_data(0.0, 0.0, 0.05);

  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory));
}

TEST_F(StopPointFixerIntegrationTest, ForceCloseStopFlag_True_CloseStopConditionTriggers)
{
  params_.stop_point_fixer.force_stop_close_stopped_trajectories = true;
  params_.stop_point_fixer.force_stop_long_stopped_trajectories = false;
  params_.use_stop_point_fixer = true;
  plugin_->update_params(params_);

  TrajectoryPoints trajectory;
  trajectory.push_back(
    create_trajectory_point(0.0, 0.0, params_.stop_point_fixer.velocity_threshold * 2.0));
  trajectory.push_back(
    create_trajectory_point(0.5, 0.0, params_.stop_point_fixer.velocity_threshold * 2.0));

  set_odometry_data(0.0, 0.0, params_.stop_point_fixer.velocity_threshold * 0.2);

  EXPECT_TRUE(plugin_->is_trajectory_modification_required(trajectory));
}

TEST_F(StopPointFixerIntegrationTest, BothFlags_False_NeitherConditionTriggers)
{
  params_.stop_point_fixer.force_stop_long_stopped_trajectories = false;
  params_.stop_point_fixer.force_stop_close_stopped_trajectories = false;
  params_.use_stop_point_fixer = true;
  plugin_->update_params(params_);

  auto min_stop_duration_s = params_.stop_point_fixer.min_stop_duration;
  TrajectoryPoints trajectory;
  trajectory.push_back(
    create_trajectory_point_with_duration(0.5, 0.0, 0.0, min_stop_duration_s * 2.0));

  set_odometry_data(0.0, 0.0, 0.05);

  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory));
}

// Velocity threshold boundary tests at runtime default (0.1 mps from set_up_params).
// Uses two trajectory points so calcSignedArcLength produces a valid close-stop distance.

TEST_F(StopPointFixerIntegrationTest, RuntimeDefault_StationaryEgo_TriggersModification)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.0, 0.0, 1.0));
  trajectory.push_back(create_trajectory_point(0.5, 0.0, 1.0));

  params_.use_stop_point_fixer = true;
  plugin_->update_params(params_);

  set_odometry_data(0.0, 0.0, 0.05);

  EXPECT_TRUE(plugin_->is_trajectory_modification_required(trajectory));
}

TEST_F(StopPointFixerIntegrationTest, RuntimeDefault_MovingEgoAboveThreshold_SuppressesModification)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.0, 0.0, 1.0));
  trajectory.push_back(create_trajectory_point(0.5, 0.0, 1.0));

  params_.use_stop_point_fixer = true;
  plugin_->update_params(params_);

  set_odometry_data(0.0, 0.0, 0.15);

  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory));
}

TEST_F(
  StopPointFixerIntegrationTest,
  RuntimeDefault_VelocityBetweenStructAndRuntimeDefault_TreatedAsMoving)
{
  // ego velocity = 0.15 is above the runtime default (0.1) but below the struct field default
  // (0.25), verifying that the runtime-declared value is used
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(0.0, 0.0, 1.0));
  trajectory.push_back(create_trajectory_point(0.5, 0.0, 1.0));

  params_.use_stop_point_fixer = true;
  plugin_->update_params(params_);

  set_odometry_data(0.0, 0.0, 0.15);

  EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory));
}

// Decimal time tests for is_long_stop_trajectory.
// The implementation computes time_from_start = sec + nanosec * 1e-9 and compares against
// min_stop_duration_s with sub-second precision. Times are expressed as multiples of
// min_stop_duration_s so the tests remain valid if the default is changed.

TEST_F(
  StopPointFixerIntegrationTest,
  IsLongStopTrajectory_StopBelowThresholdWithMovementAlsoBelowThreshold_ReturnsFalse)
{
  const double T = params_.stop_point_fixer.min_stop_duration;
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point_with_duration(0.0, 0.0, 0.0, T * 0.6));
  trajectory.push_back(create_trajectory_point_with_duration(5.0, 0.0, 1.0, T * 0.8));

  EXPECT_FALSE(plugin_->is_long_stop_trajectory(trajectory));
}

TEST_F(
  StopPointFixerIntegrationTest,
  IsLongStopTrajectory_StopJustAboveThresholdFollowedByMovement_ReturnsTrue)
{
  const double T = params_.stop_point_fixer.min_stop_duration;
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point_with_duration(0.0, 0.0, 0.0, T * 1.4));
  trajectory.push_back(create_trajectory_point_with_duration(5.0, 0.0, 1.0, T * 1.6));

  EXPECT_TRUE(plugin_->is_long_stop_trajectory(trajectory));
}

TEST_F(
  StopPointFixerIntegrationTest,
  IsLongStopTrajectory_StopWellAboveThresholdFollowedByMovement_ReturnsTrue)
{
  const double T = params_.stop_point_fixer.min_stop_duration;
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point_with_duration(0.0, 0.0, 0.0, T * 2.4));
  trajectory.push_back(create_trajectory_point_with_duration(5.0, 0.0, 1.0, T * 4.0));

  EXPECT_TRUE(plugin_->is_long_stop_trajectory(trajectory));
}

TEST_F(
  StopPointFixerIntegrationTest,
  IsLongStopTrajectory_StoppedBelowThresholdThenMovingAfterThreshold_ReturnsTrue)
{
  // Stopped points at 0.6*T, 0.8*T, and exactly T (not > T so no early exit yet),
  // then a moving point at 1.2*T (past the threshold). The trajectory did not begin
  // moving before min_stop_duration_s elapsed, so it is nuked.
  const double T = params_.stop_point_fixer.min_stop_duration;
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point_with_duration(0.0, 0.0, 0.0, T * 0.6));
  trajectory.push_back(create_trajectory_point_with_duration(0.0, 0.0, 0.0, T * 0.8));
  trajectory.push_back(create_trajectory_point_with_duration(5.0, 0.0, 0.0, T * 1.0));
  trajectory.push_back(create_trajectory_point_with_duration(5.0, 0.0, 1.0, T * 1.2));

  EXPECT_TRUE(plugin_->is_long_stop_trajectory(trajectory));
}

// Long-stop flag parameter update is reflected in behavior

TEST_F(StopPointFixerIntegrationTest, ParameterUpdate_LongStopFlagCanBeToggledAtRuntime)
{
  auto min_stop_duration_s = params_.stop_point_fixer.min_stop_duration;
  TrajectoryPoints trajectory;
  trajectory.push_back(
    create_trajectory_point_with_duration(5.0, 0.0, 0.0, min_stop_duration_s * 2.0));

  params_.use_stop_point_fixer = true;
  plugin_->update_params(params_);

  set_odometry_data(0.0, 0.0, 0.05);

  {
    params_.stop_point_fixer.force_stop_long_stopped_trajectories = false;
    params_.stop_point_fixer.force_stop_close_stopped_trajectories = false;
    plugin_->update_params(params_);
    EXPECT_FALSE(plugin_->is_trajectory_modification_required(trajectory));
  }

  {
    params_.stop_point_fixer.force_stop_long_stopped_trajectories = true;
    params_.stop_point_fixer.force_stop_close_stopped_trajectories = false;
    plugin_->update_params(params_);
    EXPECT_TRUE(plugin_->is_trajectory_modification_required(trajectory));
  }
}
