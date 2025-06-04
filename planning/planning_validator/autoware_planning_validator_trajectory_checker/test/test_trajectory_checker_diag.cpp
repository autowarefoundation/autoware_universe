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

#include <autoware/planning_validator/node.hpp>
#include <autoware/planning_validator_test_utils/planning_validator_test_utils.hpp>
#include <autoware/planning_validator_test_utils/test_parameters.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::planning_validator
{
using autoware::planning_validator::PlanningValidatorNode;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

using test_utils::generateDefaultAcceleration;
using test_utils::generateDefaultOdometry;
using test_utils::generateTrajectory;
using test_utils::THRESHOLD_INTERVAL;
using test_utils::WHEELBASE;

constexpr double epsilon = 0.001;
constexpr double scale_margin = 1.1;

class PubSubManager : public rclcpp::Node
{
public:
  PubSubManager() : Node("test_pub_sub")
  {
    trajectory_pub_ = create_publisher<Trajectory>("/planning_validator_node/input/trajectory", 1);
    kinematics_pub_ = create_publisher<Odometry>("/planning_validator_node/input/kinematics", 1);
    acceleration_pub_ = create_publisher<AccelWithCovarianceStamped>(
      "/planning_validator_node/input/acceleration", 1);
    pointcloud_pub_ = create_publisher<PointCloud2>("/planning_validator_node/input/pointcloud", 1);
    diag_sub_ = create_subscription<DiagnosticArray>(
      "/diagnostics", 1,
      [this](const DiagnosticArray::ConstSharedPtr msg) { received_diags_.push_back(msg); });

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    qos.transient_local();
    map_pub_ =
      create_publisher<LaneletMapBin>("/planning_validator_node/input/lanelet_map_bin", qos);
    route_pub_ = create_publisher<LaneletRoute>("/planning_validator_node/input/route", qos);
  }

  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr kinematics_pub_;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr acceleration_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<LaneletMapBin>::SharedPtr map_pub_;
  rclcpp::Publisher<LaneletRoute>::SharedPtr route_pub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diag_sub_;

  std::vector<DiagnosticArray::ConstSharedPtr> received_diags_;
};

void spinSome(rclcpp::Node::SharedPtr node_ptr)
{
  for (int i = 0; i < 20; ++i) {
    rclcpp::spin_some(node_ptr);
    rclcpp::WallRate(100).sleep();
  }
}

bool isAllOK(const std::vector<DiagnosticArray::ConstSharedPtr> & diags)
{
  for (const auto & diag : diags) {
    for (const auto & status : diag->status) {
      if (status.level != DiagnosticStatus::OK) {
        return false;
      }
    }
  }
  return true;
}

bool hasError(const std::vector<DiagnosticArray::ConstSharedPtr> & diags)
{
  for (const auto & diag : diags) {
    for (const auto & status : diag->status) {
      if (status.level == DiagnosticStatus::ERROR) {
        return true;
      }
    }
  }
  return false;
}

bool hasError(const std::vector<DiagnosticArray::ConstSharedPtr> & diags, const std::string & name)
{
  for (const auto & diag : diags) {
    for (const auto & status : diag->status) {
      if (status.name == name) {
        return status.level == DiagnosticStatus::ERROR;
      }
    }
  }
  throw std::runtime_error(name + " is not contained in the diagnostic message.");
}

std::pair<std::shared_ptr<PlanningValidatorNode>, std::shared_ptr<PubSubManager>> prepareTest(
  const Trajectory & trajectory, const Odometry & ego_odom,
  const AccelWithCovarianceStamped & acceleration)
{
  const std::string plugin_name = "autoware::planning_validator::TrajectoryChecker";
  auto node_options = test_utils::getNodeOptionsWithDefaultParams();
  node_options.append_parameter_override("launch_modules", std::vector<std::string>{plugin_name});
  auto validator = std::make_shared<PlanningValidatorNode>(node_options);
  auto manager = std::make_shared<PubSubManager>();
  EXPECT_GE(manager->trajectory_pub_->get_subscription_count(), 1U) << "topic is not connected.";

  manager->trajectory_pub_->publish(trajectory);
  manager->kinematics_pub_->publish(ego_odom);
  manager->acceleration_pub_->publish(acceleration);
  manager->pointcloud_pub_->publish(sensor_msgs::msg::PointCloud2{}.set__header(
    std_msgs::msg::Header{}.set__frame_id("base_link")));
  manager->map_pub_->publish(autoware::test_utils::makeMapBinMsg());
  manager->route_pub_->publish(autoware::test_utils::makeBehaviorNormalRoute());
  spinSome(validator);
  spinSome(manager);

  return {validator, manager};
}

void runWithOKTrajectory(
  const Trajectory & trajectory, const Odometry & ego_odom,
  const AccelWithCovarianceStamped & acceleration)
{
  auto [validator, manager] = prepareTest(trajectory, ego_odom, acceleration);

  EXPECT_GE(manager->received_diags_.size(), 1U) << "diag has not received!";
  EXPECT_TRUE(isAllOK(manager->received_diags_));
}

void runWithOKTrajectory(
  const Trajectory & trajectory, const Odometry & ego_odom,
  const AccelWithCovarianceStamped & acceleration, const std::string & name)
{
  auto [validator, manager] = prepareTest(trajectory, ego_odom, acceleration);

  EXPECT_GE(manager->received_diags_.size(), 1U) << "diag has not received!";
  EXPECT_FALSE(hasError(manager->received_diags_, name));
}

void runWithBadTrajectory(
  const Trajectory & trajectory, const Odometry & ego_odom,
  const AccelWithCovarianceStamped & acceleration)
{
  auto [validator, manager] = prepareTest(trajectory, ego_odom, acceleration);

  EXPECT_GE(manager->received_diags_.size(), 1U) << "diag has not received!";
  EXPECT_TRUE(hasError(manager->received_diags_));
}

void runWithBadTrajectory(
  const Trajectory & trajectory, const Odometry & ego_odom,
  const AccelWithCovarianceStamped & acceleration, const std::string & name)
{
  auto [validator, manager] = prepareTest(trajectory, ego_odom, acceleration);

  EXPECT_GE(manager->received_diags_.size(), 1U) << "diag has not received!";
  EXPECT_TRUE(hasError(manager->received_diags_, name));
}

// =============================================================
//                    Overall tests
// =============================================================

// OK cases
TEST(TrajectoryCheckerModule, DiagCheckForNominalTrajectory)
{
  runWithOKTrajectory(
    generateTrajectory(THRESHOLD_INTERVAL * 0.5), generateDefaultOdometry(),
    generateDefaultAcceleration());
}

// Bad cases
TEST(TrajectoryCheckerModule, DiagCheckForNaNTrajectory)
{
  runWithBadTrajectory(
    test_utils::generateNanTrajectory(), generateDefaultOdometry(), generateDefaultAcceleration());
}
TEST(TrajectoryCheckerModule, DiagCheckForInfTrajectory)
{
  runWithBadTrajectory(
    test_utils::generateInfTrajectory(), generateDefaultOdometry(), generateDefaultAcceleration());
}
TEST(TrajectoryCheckerModule, DiagCheckForTooLongIntervalTrajectory)
{
  constexpr double ep = 0.001;
  runWithBadTrajectory(
    generateTrajectory(THRESHOLD_INTERVAL + ep), generateDefaultOdometry(),
    generateDefaultAcceleration());
}

// =============================================================
//                    Specific diag tests
// =============================================================

TEST(TrajectoryCheckerModule, DiagCheckSize)
{
  const auto diag_name = "planning_validator_node: trajectory_validation_size";
  const auto odom = generateDefaultOdometry();
  const auto accel = generateDefaultAcceleration();
  runWithBadTrajectory(generateTrajectory(1.0, 1.0, 0.0, 0), odom, accel, diag_name);
  runWithBadTrajectory(generateTrajectory(1.0, 1.0, 0.0, 1), odom, accel, diag_name);
  runWithOKTrajectory(generateTrajectory(1.0, 1.0, 0.0, 2), odom, accel);
  runWithOKTrajectory(generateTrajectory(1.0, 1.0, 0.0, 3), odom, accel);
}

TEST(TrajectoryCheckerModule, DiagCheckInterval)
{
  const auto diag_name = "planning_validator_node: trajectory_validation_interval";
  const auto odom = generateDefaultOdometry();
  const auto accel = generateDefaultAcceleration();

  // Larger interval than threshold -> must be NG
  {
    auto trajectory = generateTrajectory(1.0, 1.0, 0.0, 10);
    auto tp = trajectory.points.back();
    tp.pose.position.x += THRESHOLD_INTERVAL + 0.001;
    trajectory.points.push_back(tp);
    runWithBadTrajectory(trajectory, odom, accel, diag_name);
  }

  // Smaller interval than threshold -> must be OK
  {
    auto trajectory = generateTrajectory(1.0, 1.0, 0.0, 10);
    auto tp = trajectory.points.back();
    tp.pose.position.x += THRESHOLD_INTERVAL - 0.001;
    trajectory.points.push_back(tp);
    runWithOKTrajectory(trajectory, odom, accel, diag_name);
  }
}

TEST(TrajectoryCheckerModule, DiagCheckRelativeAngle)
{
  using test_utils::THRESHOLD_RELATIVE_ANGLE;
  const auto diag_name = "planning_validator_node: trajectory_validation_relative_angle";

  // TODO(Horibe): interval must be larger than min_interval used in planning_validator.cpp
  constexpr auto interval = 1.1;

  const auto odom = generateDefaultOdometry();
  const auto accel = generateDefaultAcceleration();

  // Larger Relative Angle than threshold -> must be NG
  {
    auto bad_trajectory = generateTrajectory(interval, 1.0, 0.0, 10);
    auto bad_tp = bad_trajectory.points.back();
    bad_tp.pose.position.x += interval * std::cos(THRESHOLD_RELATIVE_ANGLE + 0.01);
    bad_tp.pose.position.y += interval * std::sin(THRESHOLD_RELATIVE_ANGLE + 0.01);
    bad_trajectory.points.push_back(bad_tp);
    runWithBadTrajectory(bad_trajectory, odom, accel, diag_name);
  }

  // Smaller Relative Angle than threshold -> must be OK
  {
    auto ok_trajectory = generateTrajectory(interval, 1.0, 0.0, 10);
    auto ok_tp = ok_trajectory.points.back();
    ok_tp.pose.position.x += interval * std::cos(THRESHOLD_RELATIVE_ANGLE - 0.01);
    ok_tp.pose.position.y += interval * std::sin(THRESHOLD_RELATIVE_ANGLE - 0.01);
    ok_trajectory.points.push_back(ok_tp);
    runWithOKTrajectory(ok_trajectory, odom, accel, diag_name);
  }
}

TEST(TrajectoryCheckerModule, DiagCheckCurvature)
{
  const auto diag_name = "planning_validator_node: trajectory_validation_curvature";

  // TODO(Horibe): interval must be larger than min_interval used in planning_validator.cpp
  constexpr auto interval = 1.1;

  const auto odom = generateDefaultOdometry();
  const auto accel = generateDefaultAcceleration();

  // Large y at one point -> must be NG
  {
    auto bad_trajectory = generateTrajectory(interval, 1.0, 0.0, 10);
    bad_trajectory.points.at(5).pose.position.y += 5.0;
    runWithBadTrajectory(bad_trajectory, odom, accel, diag_name);
  }

  // Higher curvature than threshold -> must be NG
  {
    constexpr double curvature = test_utils::THRESHOLD_CURVATURE * scale_margin;
    auto bad_trajectory =
      test_utils::generateTrajectoryWithConstantCurvature(interval, 1.0, curvature, 10, WHEELBASE);
    runWithBadTrajectory(bad_trajectory, odom, accel, diag_name);
  }

  // Lower curvature than threshold -> must be OK
  {
    constexpr double curvature = test_utils::THRESHOLD_CURVATURE / scale_margin;
    auto ok_trajectory =
      test_utils::generateTrajectoryWithConstantCurvature(interval, 1.0, curvature, 10, WHEELBASE);
    runWithOKTrajectory(ok_trajectory, odom, accel, diag_name);
  }
}

TEST(TrajectoryCheckerModule, DiagCheckLateralAcceleration)
{
  const auto diag_name = "planning_validator_node: trajectory_validation_lateral_acceleration";
  constexpr double speed = 10.0;

  const auto odom = generateDefaultOdometry();
  const auto accel = generateDefaultAcceleration();

  // Note: lateral_acceleration is speed^2 * curvature;

  // Higher lateral acc than threshold -> must be NG
  {
    constexpr double curvature = test_utils::THRESHOLD_LATERAL_ACC / (speed * speed) * scale_margin;
    const auto bad_trajectory =
      test_utils::generateTrajectoryWithConstantCurvature(1.0, speed, curvature, 10, WHEELBASE);
    runWithBadTrajectory(bad_trajectory, odom, accel, diag_name);
  }

  // Smaller lateral acc than threshold -> must be OK
  {
    constexpr double curvature = test_utils::THRESHOLD_LATERAL_ACC / (speed * speed) / scale_margin;
    const auto ok_trajectory =
      test_utils::generateTrajectoryWithConstantCurvature(1.0, speed, curvature, 10, WHEELBASE);
    runWithOKTrajectory(ok_trajectory, odom, accel, diag_name);
  }
}

TEST(TrajectoryCheckerModule, DiagCheckLongitudinalMaxAcc)
{
  const auto diag_name = "planning_validator_node: trajectory_validation_acceleration";
  constexpr double speed = 1.0;

  const auto odom = generateDefaultOdometry();
  const auto accel = generateDefaultAcceleration();

  // Larger acceleration than threshold -> must be NG
  {
    constexpr double acceleration = test_utils::THRESHOLD_LONGITUDINAL_MAX_ACC + epsilon;
    auto bad_trajectory =
      test_utils::generateTrajectoryWithConstantAcceleration(1.0, speed, 0.0, 20, acceleration);
    runWithBadTrajectory(bad_trajectory, odom, accel, diag_name);
  }

  // Smaller acceleration than threshold -> must be OK
  {
    constexpr double acceleration = test_utils::THRESHOLD_LONGITUDINAL_MAX_ACC - epsilon;
    auto bad_trajectory =
      test_utils::generateTrajectoryWithConstantAcceleration(1.0, speed, 0.0, 20, acceleration);
    runWithOKTrajectory(bad_trajectory, odom, accel, diag_name);
  }
}

TEST(TrajectoryCheckerModule, DiagCheckLongitudinalMinAcc)
{
  const auto diag_name = "planning_validator_node: trajectory_validation_deceleration";
  constexpr double speed = 20.0;

  const auto odom = generateDefaultOdometry();
  const auto accel = generateDefaultAcceleration();

  const auto test = [&](const auto acceleration, const bool expect_ok) {
    auto trajectory =
      test_utils::generateTrajectoryWithConstantAcceleration(1.0, speed, 0.0, 10, acceleration);
    if (expect_ok) {
      runWithOKTrajectory(trajectory, odom, accel, diag_name);
    } else {
      runWithBadTrajectory(trajectory, odom, accel, diag_name);
    }
  };

  // Larger deceleration than threshold -> must be NG
  test(test_utils::THRESHOLD_LONGITUDINAL_MIN_ACC - epsilon, false);

  // Larger deceleration than threshold -> must be OK
  test(test_utils::THRESHOLD_LONGITUDINAL_MIN_ACC + epsilon, true);
}

TEST(TrajectoryCheckerModule, DiagCheckSteering)
{
  const auto diag_name = "planning_validator_node: trajectory_validation_steering";

  // TODO(Horibe): interval must be larger than min_interval used in planning_validator.cpp
  constexpr auto interval = 1.1;

  const auto test = [&](const auto steering, const bool expect_ok) {
    auto trajectory =
      test_utils::generateTrajectoryWithConstantSteering(interval, 1.0, steering, 10, WHEELBASE);
    const auto odom = generateDefaultOdometry();
    const auto accel = generateDefaultAcceleration();
    expect_ok ? runWithOKTrajectory(trajectory, odom, accel, diag_name)
              : runWithBadTrajectory(trajectory, odom, accel, diag_name);
  };

  // Larger steering than threshold -> must be NG
  test(test_utils::THRESHOLD_STEERING * scale_margin, false);
  test(-test_utils::THRESHOLD_STEERING * scale_margin, false);

  // Smaller steering than threshold -> must be OK
  test(test_utils::THRESHOLD_STEERING / scale_margin, true);
  test(-test_utils::THRESHOLD_STEERING / scale_margin, true);
}

TEST(TrajectoryCheckerModule, DiagCheckSteeringRate)
{
  const auto diag_name = "planning_validator_node: trajectory_validation_steering_rate";

  // TODO(Horibe): interval must be larger than min_interval used in planning_validator.cpp
  constexpr auto interval = 1.1;

  const auto test = [&](const auto steering_rate, const bool expect_ok) {
    auto trajectory = test_utils::generateTrajectoryWithConstantSteeringRate(
      interval, 1.0, steering_rate, 10, WHEELBASE);
    const auto odom = generateDefaultOdometry();
    const auto accel = generateDefaultAcceleration();
    expect_ok ? runWithOKTrajectory(trajectory, odom, accel, diag_name)
              : runWithBadTrajectory(trajectory, odom, accel, diag_name);
  };

  // Larger steering rate than threshold -> must be NG
  test(test_utils::THRESHOLD_STEERING_RATE * scale_margin, false);
  test(-test_utils::THRESHOLD_STEERING_RATE * scale_margin, false);

  // Smaller steering rate than threshold -> must be OK
  test(test_utils::THRESHOLD_STEERING_RATE / scale_margin, true);
  test(-test_utils::THRESHOLD_STEERING_RATE / scale_margin, true);
}

TEST(TrajectoryCheckerModule, DiagCheckVelocityDeviation)
{
  const auto diag_name = "planning_validator_node: trajectory_validation_velocity_deviation";
  const auto test = [&](const auto trajectory_speed, const auto ego_speed, const bool expect_ok) {
    const auto trajectory = generateTrajectory(1.0, trajectory_speed, 0.0, 10);
    const auto ego_odom = generateDefaultOdometry(0.0, 0.0, ego_speed);
    const auto accel = generateDefaultAcceleration();
    expect_ok ? runWithOKTrajectory(trajectory, ego_odom, accel, diag_name)
              : runWithBadTrajectory(trajectory, ego_odom, accel, diag_name);
  };

  // Larger velocity deviation than threshold -> must be NG
  test(1.0 + test_utils::THRESHOLD_VELOCITY_DEVIATION * scale_margin, 1.0, false);
  test(1.0, 1.0 + test_utils::THRESHOLD_VELOCITY_DEVIATION * scale_margin, false);

  // Larger velocity deviation than threshold -> must be OK
  test(1.0, 1.0 + test_utils::THRESHOLD_VELOCITY_DEVIATION / scale_margin, true);
}

TEST(TrajectoryCheckerModule, DiagCheckDistanceDeviation)
{
  const auto diag_name = "planning_validator_node: trajectory_validation_distance_deviation";
  const auto test = [&](const auto ego_x, const auto ego_y, const bool expect_ok) {
    const auto trajectory = generateTrajectory(1.0, 3.0, 0.0, 10);
    const auto last_p = trajectory.points.back().pose.position;
    const auto ego_odom = generateDefaultOdometry(last_p.x + ego_x, last_p.y + ego_y, 0.0);
    const auto accel = generateDefaultAcceleration();
    expect_ok ? runWithOKTrajectory(trajectory, ego_odom, accel, diag_name)
              : runWithBadTrajectory(trajectory, ego_odom, accel, diag_name);
  };

  // Larger distance deviation than threshold -> must be NG
  const auto error_distance = test_utils::THRESHOLD_DISTANCE_DEVIATION * scale_margin;
  test(error_distance, 0.0, false);
  test(0.0, error_distance, false);
  test(0.0, -error_distance, false);
  test(error_distance, error_distance, false);
  test(error_distance, -error_distance, false);

  // Smaller distance deviation than threshold -> must be OK
  const auto ok_distance = test_utils::THRESHOLD_DISTANCE_DEVIATION / scale_margin;
  test(ok_distance, 0.0, true);
  test(0.0, ok_distance, true);
  test(0.0, -ok_distance, true);
}

TEST(TrajectoryCheckerModule, DiagCheckLongitudinalDistanceDeviation)
{
  const auto diag_name =
    "planning_validator_node: trajectory_validation_longitudinal_distance_deviation";
  const auto trajectory = generateTrajectory(1.0, 3.0, 0.0, 10);
  const auto test = [&](const auto ego_x, const auto ego_y, const bool expect_ok) {
    const auto ego_odom = generateDefaultOdometry(ego_x, ego_y, 0.0);
    const auto accel = generateDefaultAcceleration();
    expect_ok ? runWithOKTrajectory(trajectory, ego_odom, accel, diag_name)
              : runWithBadTrajectory(trajectory, ego_odom, accel, diag_name);
  };

  const auto invalid_distance =
    test_utils::THRESHOLD_LONGITUDINAL_DISTANCE_DEVIATION * scale_margin;
  const auto valid_distance = test_utils::THRESHOLD_LONGITUDINAL_DISTANCE_DEVIATION / scale_margin;

  // behind from idx=0 -> must be NG
  test(-invalid_distance, 0.0, false);

  // ahead from idx=last -> must be NG
  test(trajectory.points.back().pose.position.x + invalid_distance, 0.0, false);

  // In between trajectory points -> must be OK
  const auto mid_point = trajectory.points.at(5).pose.position;
  test(mid_point.x, mid_point.y, true);

  // behind from idx=0, but close to 0 -> must be OK
  test(-valid_distance, 0.0, true);

  // ahead from idx=last, but close to last -> must be OK
  test(trajectory.points.back().pose.position.x + valid_distance, 0.0, true);
}

TEST(TrajectoryCheckerModule, DiagCheckForwardTrajectoryLength)
{
  const auto diag_name = "planning_validator_node: trajectory_validation_forward_trajectory_length";
  constexpr auto trajectory_v = 10.0;
  constexpr size_t trajectory_size = 10;
  constexpr auto ego_v = 10.0;
  constexpr auto ego_a = std::abs(test_utils::PARAMETER_FORWARD_TRAJECTORY_LENGTH_ACCELERATION);
  constexpr auto margin = test_utils::PARAMETER_FORWARD_TRAJECTORY_LENGTH_MARGIN;

  const auto accel = generateDefaultAcceleration();

  // Longer trajectory than threshold -> must be OK
  {
    constexpr auto ok_trajectory_length = ego_v * ego_v / (2.0 * ego_a);  // v1^2 - v0^2 = 2as
    std::cerr << "aaa: " << ok_trajectory_length << std::endl;
    constexpr auto trajectory_interval = ok_trajectory_length / trajectory_size;
    const auto ok_trajectory =
      generateTrajectory(trajectory_interval, trajectory_v, 0.0, trajectory_size);
    const auto ego_odom = generateDefaultOdometry(0.0, 0.0, ego_v);
    runWithOKTrajectory(ok_trajectory, ego_odom, accel, diag_name);
  }

  // Smaller trajectory than threshold -> must be NG
  {
    // shorter with 1.0m
    constexpr auto bad_trajectory_length = ego_v * ego_v / (2.0 * ego_a) - margin - 1.0;
    std::cerr << "bbb: " << bad_trajectory_length << std::endl;
    constexpr auto trajectory_interval = bad_trajectory_length / trajectory_size;
    const auto bad_trajectory =
      generateTrajectory(trajectory_interval, trajectory_v, 0.0, trajectory_size);
    const auto ego_odom = generateDefaultOdometry(0.0, 0.0, ego_v);
    runWithBadTrajectory(bad_trajectory, ego_odom, accel, diag_name);
  }

  // Longer trajectory than threshold, but smaller length from ego -> must be NG
  {
    constexpr auto bad_trajectory_length = ego_v * ego_v / (2.0 * ego_a);  // v1^2 - v0^2 = 2as
    std::cerr << "ccc: " << bad_trajectory_length << std::endl;
    constexpr auto trajectory_interval = bad_trajectory_length / trajectory_size;
    const auto bad_trajectory =
      generateTrajectory(trajectory_interval, trajectory_v, 0.0, trajectory_size);
    const auto & p = bad_trajectory.points.at(trajectory_size - 1).pose.position;
    const auto ego_odom = generateDefaultOdometry(p.x, p.y, ego_v);
    runWithBadTrajectory(bad_trajectory, ego_odom, accel, diag_name);
  }
}

TEST(TrajectoryCheckerModule, DiagCheckYawDeviation)
{
  const auto diag_name = "planning_validator_node: trajectory_validation_yaw_deviation";
  const auto straight_trajectory = generateTrajectory(1.0, 0.0, 0.0, 10);

  const auto accel = generateDefaultAcceleration();

  // Ego with yaw deviation smaller than threshold -> must be OK
  {
    auto ego_odom = generateDefaultOdometry(0.0, 0.0, 0.0);
    for (auto yaw = 0.0; yaw <= test_utils::THRESHOLD_YAW_DEVIATION; yaw += 0.1) {
      ego_odom.pose.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
      runWithOKTrajectory(straight_trajectory, ego_odom, accel, diag_name);
    }
  }
  // Ego with yaw deviation larger than threshold -> must be NG
  {
    auto ego_odom = generateDefaultOdometry(0.0, 0.0, 0.0);
    for (auto yaw = test_utils::THRESHOLD_YAW_DEVIATION + 1e-3; yaw < M_PI; yaw += 0.1) {
      ego_odom.pose.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
      runWithBadTrajectory(straight_trajectory, ego_odom, accel, diag_name);
      ego_odom.pose.pose.orientation = autoware_utils::create_quaternion_from_yaw(-yaw);
      runWithBadTrajectory(straight_trajectory, ego_odom, accel, diag_name);
    }
  }
}
}  // namespace autoware::planning_validator
