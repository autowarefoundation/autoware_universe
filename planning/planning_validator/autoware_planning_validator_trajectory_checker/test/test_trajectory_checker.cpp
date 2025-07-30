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

#include "autoware/planning_validator_trajectory_checker/trajectory_checker.hpp"
#include "autoware/planning_validator_trajectory_checker/utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/planning_validator/types.hpp>
#include <autoware/planning_validator_test_utils/planning_validator_test_utils.hpp>
#include <autoware/planning_validator_test_utils/test_parameters.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <rclcpp/node.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_validator
{
using autoware::planning_validator::PlanningValidatorContext;
using autoware::planning_validator::TrajectoryChecker;
using autoware_planning_msgs::msg::Trajectory;

using test_utils::DEFAULT_HANDLING_TYPE;
using test_utils::generateTrajectory;
using test_utils::THRESHOLD_INTERVAL;

class TestTrajectoryChecker : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory(
         "autoware_planning_validator_trajectory_checker") +
         "/config/trajectory_checker.param.yaml",
       "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_test_utils") +
         "/config/test_vehicle_info.param.yaml"});
    options.append_parameter_override("trajectory_checker.interval.threshold", THRESHOLD_INTERVAL);
    options.append_parameter_override("default_handling_type", DEFAULT_HANDLING_TYPE);
    node_ = std::make_shared<rclcpp::Node>("test_node", options);
    context_ = std::make_shared<PlanningValidatorContext>(node_.get());
    trajectory_checker_ = std::make_shared<TrajectoryChecker>();
    trajectory_checker_->init(*node_, "trajectory_checker", context_);
  }

  void TearDown() override
  {
    node_.reset();
    trajectory_checker_.reset();
    context_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<TrajectoryChecker> trajectory_checker_;
  std::shared_ptr<PlanningValidatorContext> context_;

public:
  void set_trajectory(const Trajectory & trajectory)
  {
    context_->data->set_current_trajectory(std::make_shared<Trajectory>(trajectory));
  }

  void set_ego_pose(const geometry_msgs::msg::Pose & pose)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
    odom.pose.pose = pose;
    context_->data->current_kinematics = std::make_shared<nav_msgs::msg::Odometry>(odom);
  }

  void set_last_valid_trajectory(const Trajectory & trajectory)
  {
    context_->data->last_valid_trajectory = std::make_shared<Trajectory>(trajectory);
  }
};

TEST_F(TestTrajectoryChecker, checkValidFiniteValueFunction)
{
  // Valid Trajectory
  {
    set_trajectory(generateTrajectory(THRESHOLD_INTERVAL * 0.9));
    ASSERT_TRUE(trajectory_checker_->check_valid_finite_value());
  }

  // Nan Trajectory
  {
    set_trajectory(test_utils::generateNanTrajectory());
    ASSERT_FALSE(trajectory_checker_->check_valid_finite_value());
  }

  // Inf Trajectory
  {
    set_trajectory(test_utils::generateInfTrajectory());
    ASSERT_FALSE(trajectory_checker_->check_valid_finite_value());
  }
}

TEST_F(TestTrajectoryChecker, checkValidIntervalFunction)
{
  // Normal Trajectory
  {
    set_trajectory(generateTrajectory(THRESHOLD_INTERVAL * 0.9));
    ASSERT_TRUE(trajectory_checker_->check_valid_interval());
  }

  // Boundary Trajectory
  {
    // Note: too small value is not supported like numerical_limits::epsilon
    const auto ep = 1.0e-5;

    set_trajectory(generateTrajectory(THRESHOLD_INTERVAL - ep));
    ASSERT_TRUE(trajectory_checker_->check_valid_interval());

    set_trajectory(generateTrajectory(THRESHOLD_INTERVAL + ep));
    ASSERT_FALSE(trajectory_checker_->check_valid_interval());
  }

  // Long Interval Trajectory
  {
    set_trajectory(generateTrajectory(THRESHOLD_INTERVAL * 2.0));
    ASSERT_FALSE(trajectory_checker_->check_valid_interval());
  }
}

TEST_F(TestTrajectoryChecker, checkValidCurvatureFunction)
{
  // Normal Trajectory
  {
    set_trajectory(generateTrajectory(THRESHOLD_INTERVAL * 2.0));
    ASSERT_TRUE(trajectory_checker_->check_valid_curvature());
  }

  // Invalid curvature trajectory
  {
    // TODO(Horibe): write me
  }
}

TEST_F(TestTrajectoryChecker, checkValidRelativeAngleFunction)
{
  // valid case
  {
    /**
     * x: 0   1 2    3  4   5 6 7 8 9 10
     * y: 0 0.1 0 -0.1  0 0.2 0 0 0 0  0
     * max relative angle is about 0.197 radian (= 11 degree)
     **/
    Trajectory valid_traj = generateTrajectory(THRESHOLD_INTERVAL);
    valid_traj.points[1].pose.position.y = 0.1;
    valid_traj.points[3].pose.position.y = -0.1;
    valid_traj.points[5].pose.position.y = 0.2;
    set_trajectory(valid_traj);
    ASSERT_TRUE(trajectory_checker_->check_valid_relative_angle());
  }

  // invalid case
  {
    /**
     * x: 0 1 2 3  4 5 6 7 8 9 10
     * y: 0 0 0 0 10 0 0 0 0 0 0
     * the relative angle around index [4] is about 1.4 radian (= 84 degree)
     **/
    Trajectory invalid_traj = generateTrajectory(THRESHOLD_INTERVAL);
    invalid_traj.points[4].pose.position.x = 3;
    invalid_traj.points[4].pose.position.y = 10;
    // for (auto t : invalid_traj.points) {
    //   std::cout << "p: (x , y) = " << "( "<<t.pose.position.x <<
    // " , " << t.pose.position.y <<" )"<< std::endl;
    // }
    set_trajectory(invalid_traj);
    ASSERT_FALSE(trajectory_checker_->check_valid_relative_angle());
  }

  {
    /** <---inverted pattern-----
     * x: 0 -1 -2 -3  -4 -5 -6 -7 -8 -9 -10
     * y: 0  0  0  0  10  0  0  0  0  0   0
     **/
    constexpr auto interval = 1.0;
    Trajectory invalid_traj = generateTrajectory(interval);
    invalid_traj.points[4].pose.position.y = 10;
    for (auto t : invalid_traj.points) {
      t.pose.position.x *= -1;
    }
    set_trajectory(invalid_traj);
    ASSERT_FALSE(trajectory_checker_->check_valid_relative_angle());
  }

  {
    /** vertical pattern
     * x: 0 0 0 0 10 0 0 0 0 0  0
     * y: 0 1 2 3  4 5 6 7 8 9 10
     **/
    constexpr auto interval = 1.0;
    Trajectory invalid_traj = generateTrajectory(interval);
    for (size_t i = 0; i < invalid_traj.points.size(); i++) {
      auto & p = invalid_traj.points[i].pose.position;
      p.x = 0;
      p.y = i;
    }
    invalid_traj.points[4].pose.position.x = 10;
    set_trajectory(invalid_traj);
    ASSERT_FALSE(trajectory_checker_->check_valid_relative_angle());
  }
}

TEST_F(TestTrajectoryChecker, checkValidLateralJerkFunction)
{  // Test case 1: Valid trajectory with normal lateral jerk
  {
    set_trajectory(generateTrajectory(THRESHOLD_INTERVAL * 0.9));
    ASSERT_TRUE(trajectory_checker_->check_valid_lateral_jerk());
  }

  // Test case 2: Trajectory with straight line movement (valid lateral jerk)
  {
    std::vector<double> accel_values = {1.0, 2.0, 0.0, -1.0, -2.0};
    Trajectory zero_jerk_traj =
      test_utils::generateTrajectoryWithStepAcceleration(0.5, 5.0, 0.0, 20, accel_values, 4);
    set_trajectory(zero_jerk_traj);
    ASSERT_TRUE(trajectory_checker_->check_valid_lateral_jerk());
  }

  // Test case 3: Trajectory with sinusoidal longitudinal acceleration but straight path
  {
    Trajectory sinusoidal_accel_traj =
      test_utils::generateTrajectoryWithSinusoidalAcceleration(0.5, 8.0, 0.0, 30, 2.0, 10.0);
    set_trajectory(sinusoidal_accel_traj);
    ASSERT_TRUE(trajectory_checker_->check_valid_lateral_jerk());
  }

  // Test case 4: Trajectory with high lateral jerk (zigzag pattern)
  {
    // Generate trajectory with constant acceleration on a straight path
    Trajectory high_jerk_traj =
      test_utils::generateTrajectoryWithConstantAcceleration(2.0, 5.0, 0.0, 10, 1.0);

    // Create a sharp zigzag pattern by modifying Y positions
    for (size_t i = 2; i < high_jerk_traj.points.size(); i += 4) {
      if (i < high_jerk_traj.points.size()) {
        high_jerk_traj.points[i].pose.position.y += 2.0;
      }

      if (i + 2 < high_jerk_traj.points.size()) {
        high_jerk_traj.points[i + 2].pose.position.y -= 2.0;
      }
    }

    // Update orientations to match the path direction
    for (size_t i = 1; i < high_jerk_traj.points.size(); ++i) {
      const auto & p1 = high_jerk_traj.points[i - 1].pose.position;
      const auto & p2 = high_jerk_traj.points[i].pose.position;
      const double yaw = std::atan2(p2.y - p1.y, p2.x - p1.x);
      high_jerk_traj.points[i - 1].pose.orientation =
        autoware_utils::create_quaternion_from_yaw(yaw);
    }

    // Set high velocity to generate significant lateral jerk
    for (auto & point : high_jerk_traj.points) {
      point.longitudinal_velocity_mps = 10.0;
    }

    // This should fail due to high lateral jerk
    set_trajectory(high_jerk_traj);
    ASSERT_FALSE(trajectory_checker_->check_valid_lateral_jerk());
  }
}

TEST_F(TestTrajectoryChecker, checkCalcMaxLateralJerkFunction)
/**
 * Trajectory specification:
 * --------------------------
 * Index :               0    1    2    3    4    5    6    7    8    9
 * Velocity (m/s):       1    1    1    1    1    2    3    3    3    3
 * Acceleration (m/ss):  1    1    1    1    1    2    3    3    3    3
 * Curvature (1/m):      0    0    0    0.05 0.1  0.1  0.05 0    0    0
 * Lateral Jerk (m/sss): 0    0    0    0.15 0.3  2.4  4.05 0    0    0
 */
{
  {
    Trajectory custom_traj;
    custom_traj.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
    const double expected_max_lateral_jerk = 4.05;

    const size_t num_points = 10;
    const double point_spacing = 2.0;
    const double curve_radius = 10.0;

    // Create straight line section (indices 0-3)
    for (size_t i = 0; i < 4; ++i) {
      autoware_planning_msgs::msg::TrajectoryPoint p;
      p.pose.position.x = i * point_spacing;
      p.pose.position.y = 0.0;
      p.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
      p.longitudinal_velocity_mps = 1.0;
      p.acceleration_mps2 = 1.0;
      custom_traj.points.push_back(p);
    }

    // Create curve section (indices 4-6)
    for (size_t i = 4; i <= 6; ++i) {
      autoware_planning_msgs::msg::TrajectoryPoint p;
      const auto & last_pose = custom_traj.points[i - 1].pose;

      const double angle = (i - 3) * (point_spacing / curve_radius);
      const double last_x = last_pose.position.x;
      const double last_y = last_pose.position.y;
      const double prev_angle = (i - 4) * (point_spacing / curve_radius);

      p.pose.position.x = last_x + curve_radius * (std::sin(angle) - std::sin(prev_angle));
      p.pose.position.y =
        last_y + curve_radius * (1 - std::cos(angle) - (1 - std::cos(prev_angle)));
      p.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(angle);

      p.longitudinal_velocity_mps = 1.0 + (i - 4);  // 1.0, 2.0, 3.0
      p.acceleration_mps2 = 1.0 + (i - 4);          // 1.0, 2.0, 3.0

      custom_traj.points.push_back(p);
    }

    // Create final straight line section (indices 7-9)
    const auto & final_point = custom_traj.points[6];
    double final_roll, final_pitch, final_yaw;
    tf2::Quaternion final_q(
      final_point.pose.orientation.x, final_point.pose.orientation.y,
      final_point.pose.orientation.z, final_point.pose.orientation.w);
    tf2::Matrix3x3(final_q).getRPY(final_roll, final_pitch, final_yaw);

    const double start_x = final_point.pose.position.x;
    const double start_y = final_point.pose.position.y;

    for (size_t i = 7; i < num_points; ++i) {
      autoware_planning_msgs::msg::TrajectoryPoint p;

      const double dx = std::cos(final_yaw) * point_spacing * (i - 6);
      const double dy = std::sin(final_yaw) * point_spacing * (i - 6);

      p.pose.position.x = start_x + dx;
      p.pose.position.y = start_y + dy;
      p.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(final_yaw);

      p.longitudinal_velocity_mps = 3.0;
      p.acceleration_mps2 = 3.0;

      custom_traj.points.push_back(p);
    }

    // Calculate lateral jerk
    auto [max_lateral_jerk_value, max_lat_jerk_index] =
      autoware::planning_validator::trajectory_checker_utils::calc_max_lateral_jerk(custom_traj);
    const double tolerance = 0.01;  // 1% tolerance for lateral jerk values
    EXPECT_NEAR(expected_max_lateral_jerk, max_lateral_jerk_value, tolerance);
    EXPECT_EQ(max_lat_jerk_index, 6);  // Check the index of the maximum lateral jerk
  }
}

TEST_F(TestTrajectoryChecker, checkTrajectoryShiftFunction)
{
  using test_utils::generateShiftedTrajectory;
  /**
   * x: 0 1 2 3 4 5 6 7 8 9 10
   * y: 0 0 0 0 0 0 0 0 0 0  0
   **/
  constexpr auto interval = 1.0;
  Trajectory base_traj = generateTrajectory(interval);
  set_last_valid_trajectory(base_traj);

  const auto ego_pose =
    autoware_utils::calc_offset_pose(base_traj.points.front().pose, 2.5, 0.0, 0.0, 0.0);
  set_ego_pose(ego_pose);

  // valid case
  {
    set_trajectory(generateShiftedTrajectory(base_traj, 0.1, 1.0));
    ASSERT_TRUE(trajectory_checker_->check_trajectory_shift());
  }

  // invalid case (lateral shift)
  {
    set_trajectory(generateShiftedTrajectory(base_traj, 1.0));
    ASSERT_FALSE(trajectory_checker_->check_trajectory_shift());
  }

  // invalid case (backward shift)
  {
    set_trajectory(generateShiftedTrajectory(base_traj, 0.0, -1.0, 4));
    ASSERT_FALSE(trajectory_checker_->check_trajectory_shift());
  }

  // invalid case (forward shift)
  {
    set_trajectory(generateShiftedTrajectory(base_traj, 0.0, 4.0));
    ASSERT_FALSE(trajectory_checker_->check_trajectory_shift());
  }
}
}  // namespace autoware::planning_validator
