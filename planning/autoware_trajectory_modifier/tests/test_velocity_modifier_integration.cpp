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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/velocity_modifier.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
using autoware::trajectory_modifier::TrajectoryModifierContext;
using autoware::trajectory_modifier::plugin::InputData;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using autoware::trajectory_modifier::plugin::VelocityModifier;
using autoware_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoint create_trajectory_point(
  double x, double y, double velocity, double acceleration = 0.0)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = 0.0;
  point.pose.orientation.x = 0.0;
  point.pose.orientation.y = 0.0;
  point.pose.orientation.z = 0.0;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = static_cast<float>(velocity);
  point.acceleration_mps2 = static_cast<float>(acceleration);
  return point;
}

TrajectoryPoints create_constant_velocity_trajectory(
  double length, double velocity, double spacing = 1.0)
{
  TrajectoryPoints trajectory;
  for (double x = 0.0; x <= length + 1e-6; x += spacing) {
    trajectory.push_back(create_trajectory_point(x, 0.0, velocity));
  }
  return trajectory;
}

TrajectoryPoints create_step_change_zero_velocity_trajectory(
  double length, double default_velocity, double zero_velocity_length, double spacing = 1.0)
{
  TrajectoryPoints trajectory;
  for (double x = 0.0; x <= length + 1e-6; x += spacing) {
    auto velocity = x < zero_velocity_length ? default_velocity : 0.0;
    trajectory.push_back(create_trajectory_point(x, 0.0, velocity, 0.0));
  }
  return trajectory;
}

TrajectoryPoints create_constant_deceleration_trajectory(
  double length, double initial_velocity, double spacing = 1.0)
{
  const auto accel = -1.0 * initial_velocity * initial_velocity / (2.0 * length);
  TrajectoryPoints trajectory;
  for (double x = 0.0; x <= length + 1e-6; x += spacing) {
    auto velocity = std::sqrt(initial_velocity * initial_velocity + 2.0 * accel * x);
    trajectory.push_back(create_trajectory_point(x, 0.0, velocity, accel));
  }
  return trajectory;
}

}  // namespace

class VelocityModifierIntegrationTest : public ::testing::Test
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

    node_ = std::make_shared<rclcpp::Node>("test_velocity_modifier_node", node_options);
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();

    set_up_default_params();

    // Create the context and the plugin once. Tests build per-frame InputData inline,
    // and inject any required TF directly into context_->tf_buffer.
    context_ = std::make_shared<TrajectoryModifierContext>(node_.get());
    plugin_ = std::make_unique<VelocityModifier>();
    plugin_->initialize("test_velocity_modifier", node_.get(), time_keeper_, context_, params_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    plugin_.reset();
    context_.reset();
    node_.reset();
  }

  void set_up_default_params()
  {
    params_.use_velocity_modifier = true;
    params_.trajectory_time_step = 0.1;
    params_.stopping_constraints.nominal_deceleration = 1.0;
    params_.stopping_constraints.maximum_deceleration = 4.0;
    params_.stopping_constraints.jerk_limit = 3.0;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::unique_ptr<VelocityModifier> plugin_;
  trajectory_modifier_params::Params params_;
  std::shared_ptr<TrajectoryModifierContext> context_;
};

TEST_F(VelocityModifierIntegrationTest, TrajectoryNotModifiedWhenDisabled)
{
  // Arrange
  params_.use_velocity_modifier = false;
  plugin_->update_params(params_);
  TrajectoryPoints trajectory;

  // Act
  const bool modified = plugin_->modify_trajectory(trajectory, InputData{});

  // Assert
  EXPECT_FALSE(modified);
}

TEST_F(VelocityModifierIntegrationTest, TrajectoryNotModifiedForEmptyTrajectory)
{
  // Arrange
  TrajectoryPoints trajectory;

  // Act
  const bool modified = plugin_->modify_trajectory(trajectory, InputData{});

  // Assert
  EXPECT_FALSE(modified);
}

TEST_F(VelocityModifierIntegrationTest, TrajectoryNotModifiedWhenNoStopPoint)
{
  // Arrange
  auto trajectory = create_constant_velocity_trajectory(20.0, 10.0);

  // Act
  const bool modified = plugin_->modify_trajectory(trajectory, InputData{});

  // Assert
  EXPECT_FALSE(modified);
}

TEST_F(VelocityModifierIntegrationTest, TrajectoryNotModifiedForSmoothStoppingTrajectory)
{
  // Arrange
  auto trajectory = create_constant_deceleration_trajectory(20.0, 10.0);

  // Act
  const bool modified = plugin_->modify_trajectory(trajectory, InputData{});

  // Assert
  EXPECT_FALSE(modified);
}

TEST_F(VelocityModifierIntegrationTest, TrajectoryModifiedForStepChangeZeroVelocityTrajectory)
{
  // Arrange
  auto trajectory = create_step_change_zero_velocity_trajectory(20.0, 10.0, 15.0);

  // Act
  const bool modified = plugin_->modify_trajectory(trajectory, InputData{});

  // Assert
  EXPECT_TRUE(modified);
  EXPECT_NEAR(trajectory.back().longitudinal_velocity_mps, 0.0, 0.1);
  EXPECT_NEAR(trajectory.back().acceleration_mps2, 0.0, 0.1);
}
