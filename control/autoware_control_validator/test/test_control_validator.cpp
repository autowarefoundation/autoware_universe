// Copyright 2024 TIER IV, Inc.
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

#include "autoware/control_validator/control_validator.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/node_options.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest-param-test.h>
#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <tuple>

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

Trajectory make_linear_trajectory(
  const TrajectoryPoint & start, const TrajectoryPoint & end, size_t num_points, double velocity)
{
  auto create_quaternion = [](double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
  };

  double yaw = std::atan2(
    end.pose.position.y - start.pose.position.y, end.pose.position.x - start.pose.position.x);
  yaw += (velocity < 0) ? M_PI : 0;

  Trajectory trajectory;
  trajectory.points.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    double ratio = static_cast<double>(i) / static_cast<double>(num_points - 1);

    TrajectoryPoint point;
    point.pose.position.x =
      start.pose.position.x + ratio * (end.pose.position.x - start.pose.position.x);
    point.pose.position.y =
      start.pose.position.y + ratio * (end.pose.position.y - start.pose.position.y);
    point.pose.orientation = create_quaternion(yaw);
    point.longitudinal_velocity_mps = static_cast<float>(velocity);
    point.lateral_velocity_mps = 0.0;

    trajectory.points.emplace_back(point);
  }

  return trajectory;
}

TrajectoryPoint make_trajectory_point(double x, double y)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  return point;
}

namespace autoware::control_validator
{
class TrajectoryValidatorTest
: public ::testing::TestWithParam<std::tuple<Trajectory, Trajectory, double, bool>>
{
public:
  void validate(
    ControlValidatorStatus & res, const Trajectory & predicted_trajectory,
    const Trajectory & reference_trajectory)
  {
    return trajectory_validator_->validate(res, predicted_trajectory, reference_trajectory);
  }

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_control_validator") +
         "/config/control_validator.param.yaml",
       "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_test_utils") +
         "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<ControlValidator>(options);
    trajectory_validator_ = std::make_shared<TrajectoryValidator>(*node_);
  }
  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<TrajectoryValidator> trajectory_validator_;
};

TEST_P(TrajectoryValidatorTest, test_calc_lateral_deviation_status)
{
  auto [reference_trajectory, predicted_trajectory, expected_deviation, expected_condition] =
    GetParam();
  ControlValidatorStatus res;
  validate(res, predicted_trajectory, reference_trajectory);

  EXPECT_EQ(res.is_valid_max_distance_deviation, expected_condition);
  EXPECT_NEAR(res.max_distance_deviation, expected_deviation, 1e-5);
}

INSTANTIATE_TEST_SUITE_P(
  TrajectoryDeviationTests, TrajectoryValidatorTest,
  ::testing::Values(

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0.99), 11, 1.0),
      0.99, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 1.0), 11, 1.0),
      1.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 1.01), 11, 1.0),
      1.01, false),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, -1.0),
      make_linear_trajectory(
        make_trajectory_point(0, 0), make_trajectory_point(10, 0.99), 11, -1.0),
      0.99, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, -1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 1.0), 11, -1.0),
      1.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, -1.0),
      make_linear_trajectory(
        make_trajectory_point(0, 0), make_trajectory_point(10, 1.01), 11, -1.0),
      1.01, false),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(11, 0), make_trajectory_point(20, 0.0), 11, 1.0),
      0.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(11, 0), make_trajectory_point(20, 0.0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      0.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(1, 0), make_trajectory_point(10, 1.0), 11, 1.0),
      1.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(-1, 0), make_trajectory_point(10, 1.0), 11, 1.0),
      1.0, true),

    std::make_tuple(
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(10, 0), 11, 1.0),
      make_linear_trajectory(make_trajectory_point(0, 0), make_trajectory_point(20, 2.0), 21, 1.0),
      1.0, true))

);

class AccelerationValidatorTest : public ::testing::TestWithParam<std::tuple<bool, double, double>>
{
public:
  bool is_in_error_range() { return acceleration_validator_->is_in_error_range(); }
  void set_desired(double x) { acceleration_validator_->desired_acc_lpf.reset(x); }
  void set_measured(double x) { acceleration_validator_->measured_acc_lpf.reset(x); }

protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_control_validator") +
         "/config/control_validator.param.yaml",
       "--params-file",
       ament_index_cpp::get_package_share_directory("autoware_test_utils") +
         "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<ControlValidator>(options);
    acceleration_validator_ = std::make_shared<AccelerationValidator>(*node_);
  }
  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<AccelerationValidator> acceleration_validator_;
};

TEST_P(AccelerationValidatorTest, test_is_in_error_range)
{
  auto [expected, des, mes] = GetParam();
  set_desired(des);
  set_measured(mes);

  ASSERT_EQ(expected, is_in_error_range());
};

INSTANTIATE_TEST_SUITE_P(
  AccelerationValidatorTests, AccelerationValidatorTest,
  ::testing::Values(
    std::make_tuple(true, 0.0, 0.0), std::make_tuple(false, 0.0, 5.0),
    std::make_tuple(false, 0.0, -5.0), std::make_tuple(true, 1.0, 1.0),
    std::make_tuple(false, 1.0, 5.0), std::make_tuple(false, 1.0, -5.0),
    std::make_tuple(true, -1.0, -1.0), std::make_tuple(false, -1.0, -5.0),
    std::make_tuple(false, -1.0, 5.0)));

}  // namespace autoware::control_validator
