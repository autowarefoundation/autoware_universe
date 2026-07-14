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

#include "test_utils.hpp"

#include "autoware/trajectory_processor/trajectory_modifier_utils/utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace trajectory_optimizer_test_utils
{

rclcpp::NodeOptions make_optimizer_node_options()
{
  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("use_sim_time", true);
  options.append_parameter_override("trajectory_velocity_optimizer.smooth_velocities", true);

  const auto package_dir =
    ament_index_cpp::get_package_share_directory("autoware_trajectory_processor");
  const auto path_optimizer_dir =
    ament_index_cpp::get_package_share_directory("autoware_path_optimizer");
  const auto test_utils_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");

  options.arguments(
    {"--ros-args",
     "--params-file",
     package_dir + "/config/trajectory_optimizer.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_qp_smoother.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_point_fixer.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_velocity_optimizer.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_extender.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_spline_smoother.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_kinematic_feasibility_enforcer.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_mpt_optimizer.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_temporal_mpt_optimizer.param.yaml",
     "--params-file",
     package_dir + "/config/trajectory_smoothing/elastic_band_smoother.param.yaml",
     "--params-file",
     path_optimizer_dir + "/config/path_optimizer.param.yaml",
     "--params-file",
     test_utils_dir + "/config/test_vehicle_info.param.yaml"});

  options.append_parameter_override(
    "plugin_names",
    std::vector<std::string>{
      "autoware::trajectory_optimizer::plugin::TrajectoryPointFixer",
      "autoware::trajectory_optimizer::plugin::TrajectoryKinematicFeasibilityEnforcer",
      "autoware::trajectory_optimizer::plugin::TrajectoryQPSmoother",
      "autoware::trajectory_optimizer::plugin::TrajectoryEBSmootherOptimizer",
      "autoware::trajectory_optimizer::plugin::TrajectorySplineSmoother",
      "autoware::trajectory_optimizer::plugin::TrajectoryVelocityOptimizer",
      "autoware::trajectory_optimizer::plugin::TrajectoryMPTOptimizer",
      "autoware::trajectory_optimizer::plugin::TrajectoryTemporalMPTOptimizer",
      "autoware::trajectory_optimizer::plugin::TrajectoryExtender"});

  return options;
}

rclcpp::NodeOptions make_modifier_node_options()
{
  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("use_sim_time", true);

  const auto package_dir =
    ament_index_cpp::get_package_share_directory("autoware_trajectory_processor");
  const auto test_utils_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");

  options.arguments(
    {"--ros-args", "--params-file", package_dir + "/config/trajectory_modifier.param.yaml",
     "--params-file", test_utils_dir + "/config/test_vehicle_info.param.yaml"});

  return options;
}

bool is_plugin_list_parameter(const std::string & name)
{
  return name == "plugin_names";
}

bool is_read_only(const rcl_interfaces::msg::ParameterDescriptor & descriptor)
{
  return descriptor.read_only;
}

std::optional<rclcpp::Parameter> make_updated_parameter(
  const rclcpp::Parameter & current, const rcl_interfaces::msg::ParameterDescriptor & descriptor)
{
  const auto & name = current.get_name();
  switch (current.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return rclcpp::Parameter{name, !current.as_bool()};
    case rclcpp::ParameterType::PARAMETER_INTEGER: {
      const auto current_value = static_cast<int64_t>(current.as_int());
      if (!descriptor.integer_range.empty()) {
        const auto & range = descriptor.integer_range.front();
        const auto step = static_cast<int64_t>(range.step == 0 ? 1 : range.step);
        const auto from_value = static_cast<int64_t>(range.from_value);
        const auto to_value = static_cast<int64_t>(range.to_value);
        const auto next = current_value + step <= to_value ? current_value + step : from_value;
        if (next != current_value) {
          return rclcpp::Parameter{name, next};
        }
        return std::nullopt;
      }
      return rclcpp::Parameter{name, current_value == 0 ? 1 : current_value + 1};
    }
    case rclcpp::ParameterType::PARAMETER_DOUBLE: {
      const auto current_value = current.as_double();
      if (!descriptor.floating_point_range.empty()) {
        const auto & range = descriptor.floating_point_range.front();
        const auto mid = (range.from_value + range.to_value) * 0.5;
        if (mid != current_value) {
          return rclcpp::Parameter{name, mid};
        }
        const auto next = current_value + (range.step == 0.0 ? 1.0e-3 : range.step);
        if (next <= range.to_value && next != current_value) {
          return rclcpp::Parameter{name, next};
        }
        return std::nullopt;
      }
      const auto delta = current_value == 0.0 ? 1.0e-3 : current_value * 0.01;
      return rclcpp::Parameter{name, current_value + delta};
    }
    case rclcpp::ParameterType::PARAMETER_STRING:
      return rclcpp::Parameter{name, current.as_string() + "_updated_by_test"};
    default:
      return std::nullopt;
  }
}

std::vector<std::string> list_declared_parameters(
  rclcpp::AsyncParametersClient & client, rclcpp::executors::SingleThreadedExecutor & executor)
{
  auto future = client.list_parameters({}, 10);
  EXPECT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds{5}),
    rclcpp::FutureReturnCode::SUCCESS);
  return future.get().names;
}

void set_parameter_atomically(
  rclcpp::AsyncParametersClient & client, rclcpp::executors::SingleThreadedExecutor & executor,
  const rclcpp::Parameter & parameter)
{
  auto initial_future = client.get_parameters({parameter.get_name()});
  ASSERT_EQ(
    executor.spin_until_future_complete(initial_future, std::chrono::seconds{5}),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto & initial = initial_future.get();
  ASSERT_EQ(initial.size(), 1U);
  ASSERT_FALSE(initial.front() == parameter)
    << parameter.get_name() << " must be changed to test parameter propagation";

  auto future = client.set_parameters_atomically({parameter});
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds{5}),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto & result = future.get();
  EXPECT_TRUE(result.successful) << parameter.get_name() << ": " << result.reason;
}

void set_parameters_atomically(
  rclcpp::AsyncParametersClient & client, rclcpp::executors::SingleThreadedExecutor & executor,
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::vector<std::string> names;
  names.reserve(parameters.size());
  for (const auto & parameter : parameters) {
    names.push_back(parameter.get_name());
  }

  auto initial_future = client.get_parameters(names);
  ASSERT_EQ(
    executor.spin_until_future_complete(initial_future, std::chrono::seconds{5}),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto & initial = initial_future.get();
  ASSERT_EQ(initial.size(), parameters.size());
  for (size_t i = 0; i < parameters.size(); ++i) {
    ASSERT_FALSE(initial.at(i) == parameters.at(i))
      << parameters.at(i).get_name() << " must be changed to test parameter propagation";
  }

  auto future = client.set_parameters_atomically(parameters);
  ASSERT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds{5}),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto & result = future.get();
  ASSERT_TRUE(result.successful) << result.reason;
}

}  // namespace trajectory_optimizer_test_utils

using autoware::trajectory_modifier::utils::TrajectoryPoints;
using autoware_planning_msgs::msg::TrajectoryPoint;

class UtilsTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}

  static TrajectoryPoint create_trajectory_point(double x, double y, float velocity = 1.0)
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

  static geometry_msgs::msg::Pose create_pose(double x, double y)
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

  static geometry_msgs::msg::Twist create_twist(double vx, double vy = 0.0, double vz = 0.0)
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

  autoware::trajectory_modifier::utils::replace_trajectory_with_stop_point(
    trajectory, ego_pose, 0.1);

  EXPECT_EQ(trajectory.size(), 3);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.x, 5.0);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.y, 10.0);
  EXPECT_DOUBLE_EQ(trajectory[0].longitudinal_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].lateral_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].acceleration_mps2, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].heading_rate_rps, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].front_wheel_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(trajectory[0].rear_wheel_angle_rad, 0.0);
  EXPECT_NEAR(trajectory[1].pose.position.x, 5.0, 1e-2);
  EXPECT_NEAR(trajectory[1].pose.position.y, 10.0, 1e-2);
  EXPECT_DOUBLE_EQ(trajectory[1].longitudinal_velocity_mps, 0.0);
}

TEST_F(UtilsTest, ReplaceTrajectoryWithStopPointNonEmptyTrajectory)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(1.0, 1.0, 5.0));
  trajectory.push_back(create_trajectory_point(2.0, 2.0, 10.0));
  trajectory.push_back(create_trajectory_point(3.0, 3.0, 15.0));

  auto ego_pose = create_pose(7.0, 8.0);

  autoware::trajectory_modifier::utils::replace_trajectory_with_stop_point(
    trajectory, ego_pose, 0.1);

  EXPECT_EQ(trajectory.size(), 3);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.x, 7.0);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.y, 8.0);
  EXPECT_DOUBLE_EQ(trajectory[0].longitudinal_velocity_mps, 0.0);
  EXPECT_NEAR(trajectory[1].pose.position.x, 7.0, 1e-2);
  EXPECT_NEAR(trajectory[1].pose.position.y, 8.0, 1e-2);
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

  autoware::trajectory_modifier::utils::replace_trajectory_with_stop_point(
    trajectory, ego_pose, 0.1);

  EXPECT_EQ(trajectory.size(), 3);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.y, 3.0);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.orientation.y, 0.2);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.orientation.z, 0.3);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.orientation.w, 0.9);
  EXPECT_NEAR(trajectory[1].pose.position.x, 2.0, 5e-2);
  EXPECT_NEAR(trajectory[1].pose.position.y, 3.0, 5e-2);
  EXPECT_NEAR(trajectory[1].pose.orientation.x, 0.1, 5e-2);
  EXPECT_NEAR(trajectory[1].pose.orientation.y, 0.2, 5e-2);
  EXPECT_NEAR(trajectory[1].pose.orientation.z, 0.3, 5e-2);
  EXPECT_NEAR(trajectory[1].pose.orientation.w, 0.9, 5e-2);
}

TEST_F(UtilsTest, ReplaceTrajectoryWithStopPointNegativeCoordinates)
{
  TrajectoryPoints trajectory;
  trajectory.push_back(create_trajectory_point(10.0, 10.0));

  auto ego_pose = create_pose(-5.0, -7.5);

  autoware::trajectory_modifier::utils::replace_trajectory_with_stop_point(
    trajectory, ego_pose, 0.1);

  EXPECT_EQ(trajectory.size(), 3);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.x, -5.0);
  EXPECT_DOUBLE_EQ(trajectory[0].pose.position.y, -7.5);
  EXPECT_NEAR(trajectory[1].pose.position.x, -5.0, 1e-2);
  EXPECT_NEAR(trajectory[1].pose.position.y, -7.5, 1e-2);
}
