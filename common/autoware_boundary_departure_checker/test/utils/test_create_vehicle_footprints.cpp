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

#include "autoware/boundary_departure_checker/utils.hpp"
#include "test_plot_utils.hpp"

#include <Eigen/Core>

#include <gtest/gtest.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <string>
#include <vector>

using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::LinearRing2d;
using geometry_msgs::msg::PoseWithCovariance;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

namespace
{
PoseWithCovariance create_pose_with_covariance(
  const Eigen::Matrix2d & covariance_xy, const double yaw)
{
  PoseWithCovariance pose_with_covariance;
  pose_with_covariance.covariance[0 * 6 + 0] = covariance_xy(0, 0);
  pose_with_covariance.covariance[0 * 6 + 1] = covariance_xy(0, 1);
  pose_with_covariance.covariance[1 * 6 + 0] = covariance_xy(1, 0);
  pose_with_covariance.covariance[1 * 6 + 1] = covariance_xy(1, 1);
  pose_with_covariance.pose.orientation.z = std::sin(yaw / 2);
  pose_with_covariance.pose.orientation.w = std::cos(yaw / 2);
  return pose_with_covariance;
}

TrajectoryPoints create_trajectory_points(
  const std::vector<std::pair<Eigen::Vector2d, double>> & xy_yaws)
{
  TrajectoryPoints trajectory_points;
  for (const auto & [xy, yaw] : xy_yaws) {
    TrajectoryPoint p;
    p.pose.position.x = xy(0);
    p.pose.position.y = xy(1);
    p.pose.orientation.z = std::sin(yaw / 2);
    p.pose.orientation.w = std::cos(yaw / 2);
    trajectory_points.push_back(p);
  }
  return trajectory_points;
}

PathWithLaneId create_path(const std::vector<std::pair<Eigen::Vector2d, double>> & xy_yaws)
{
  PathWithLaneId path;
  for (const auto & [xy, yaw] : xy_yaws) {
    PathPointWithLaneId p;
    p.point.pose.position.x = xy(0);
    p.point.pose.position.y = xy(1);
    p.point.pose.orientation.z = std::sin(yaw / 2);
    p.point.pose.orientation.w = std::cos(yaw / 2);
    path.points.push_back(p);
  }
  return path;
}

// reference:
// https://github.com/autowarefoundation/autoware_core/blob/main/description/autoware_sample_vehicle_description/config/vehicle_info.param.yaml
constexpr double wheel_radius_m = 0.383;
constexpr double wheel_width_m = 0.235;
constexpr double wheel_base_m = 2.79;
constexpr double wheel_tread_m = 1.64;
constexpr double front_overhang_m = 1.0;
constexpr double rear_overhang_m = 1.1;
constexpr double left_overhang_m = 0.128;
constexpr double right_overhang_m = 0.128;
constexpr double vehicle_height_m = 2.5;
constexpr double max_steer_angle_rad = 0.70;
constexpr double wheel_base_to_front_overhang = front_overhang_m + wheel_base_m;
constexpr double half_wheel_base = wheel_base_m / 2.0;
constexpr double half_width_left = wheel_tread_m / 2.0 + left_overhang_m;
constexpr double half_width_right = wheel_tread_m / 2.0 + right_overhang_m;
}  // namespace

namespace autoware::boundary_departure_checker
{

struct CreateVehicleFootprintsAlongTrajectoryParam
{
  std::string description;
  Eigen::Matrix2d covariance_xy;
  double yaw;
  std::vector<std::pair<Eigen::Vector2d, double>> trajectory_points;
  double footprint_margin_scale;
  std::vector<LinearRing2d> expected_footprints;
};

std::ostream & operator<<(std::ostream & os, const CreateVehicleFootprintsAlongTrajectoryParam & p)
{
  return os << p.description;
}

struct CreateVehicleFootprintsAlongPathParam
{
  std::string description;
  std::vector<std::pair<Eigen::Vector2d, double>> path_points;
  double footprint_extra_margin;
  std::vector<LinearRing2d> expected_footprints;
};

std::ostream & operator<<(std::ostream & os, const CreateVehicleFootprintsAlongPathParam & p)
{
  return os << p.description;
}

class CreateVehicleFootprintsAlongTrajectoryTest
: public ::testing::TestWithParam<CreateVehicleFootprintsAlongTrajectoryParam>
{
protected:
  void SetUp() override
  {
    vehicle_info = autoware::vehicle_info_utils::createVehicleInfo(
      wheel_radius_m, wheel_width_m, wheel_base_m, wheel_tread_m, front_overhang_m, rear_overhang_m,
      left_overhang_m, right_overhang_m, vehicle_height_m, max_steer_angle_rad);
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
};

class CreateVehicleFootprintsAlongPathTest
: public ::testing::TestWithParam<CreateVehicleFootprintsAlongPathParam>
{
protected:
  void SetUp() override
  {
    vehicle_info = autoware::vehicle_info_utils::createVehicleInfo(
      wheel_radius_m, wheel_width_m, wheel_base_m, wheel_tread_m, front_overhang_m, rear_overhang_m,
      left_overhang_m, right_overhang_m, vehicle_height_m, max_steer_angle_rad);
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
};

TEST_P(CreateVehicleFootprintsAlongTrajectoryTest, test_create_vehicle_footprints)
{
  const auto p = GetParam();
  const auto pose_with_covariance = create_pose_with_covariance(p.covariance_xy, p.yaw);
  const auto trajectory_points = create_trajectory_points(p.trajectory_points);

  // Generate footprints
  const auto footprints = autoware::boundary_departure_checker::utils::createVehicleFootprints(
    pose_with_covariance, trajectory_points, vehicle_info, p.footprint_margin_scale);

  // 1. Integrated Visualization
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();

    // Plot the base path points
    std::vector<double> px, py;
    for (const auto & pt : trajectory_points) {
      px.push_back(pt.pose.position.x);
      py.push_back(pt.pose.position.y);
    }
    plt.plot(Args(px, py), Kwargs("color"_a = "gray", "linestyle"_a = "--", "label"_a = "Path"));

    // Plot generated footprints
    for (const auto & fp : footprints) {
      std::vector<double> fx, fy;
      for (const auto & p_fp : fp) {
        fx.push_back(p_fp.x());
        fy.push_back(p_fp.y());
      }
      plt.plot(Args(fx, fy), Kwargs("alpha"_a = 0.5));
    }

    plt.axis(Args("equal"));
    save_figure(plt, "test_create_vehicle_footprint");
  });

  // 2. Comprehensive Edge Assertions
  ASSERT_EQ(footprints.size(), p.expected_footprints.size());
  for (size_t i = 0; i < footprints.size(); ++i) {
    const auto & footprint = footprints.at(i);
    const auto & expected_footprint = p.expected_footprints.at(i);
    ASSERT_EQ(footprint.size(), expected_footprint.size());
    for (size_t j = 0; j < footprint.size(); ++j) {
      EXPECT_DOUBLE_EQ(footprint.at(j).x(), expected_footprint.at(j).x());
      EXPECT_DOUBLE_EQ(footprint.at(j).y(), expected_footprint.at(j).y());
    }
  }
}

TEST_P(CreateVehicleFootprintsAlongPathTest, test_create_vehicle_footprints)
{
  const auto p = GetParam();
  const auto path = create_path(p.path_points);
  const auto footprints = autoware::boundary_departure_checker::utils::createVehicleFootprints(
    path, vehicle_info, p.footprint_extra_margin);

  ASSERT_EQ(footprints.size(), p.expected_footprints.size());
  for (size_t i = 0; i < footprints.size(); ++i) {
    const auto & footprint = footprints.at(i);
    const auto & expected_footprint = p.expected_footprints.at(i);
    ASSERT_EQ(footprint.size(), expected_footprint.size());
    for (size_t j = 0; j < footprint.size(); ++j) {
      EXPECT_DOUBLE_EQ(footprint.at(j).x(), expected_footprint.at(j).x());
      EXPECT_DOUBLE_EQ(footprint.at(j).y(), expected_footprint.at(j).y());
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
  LaneDepartureCheckerTest, CreateVehicleFootprintsAlongTrajectoryTest,
  ::testing::Values(
    CreateVehicleFootprintsAlongTrajectoryParam{
      "EmptyTrajectory", Eigen::Matrix2d{{0.0, 0.0}, {0.0, 0.0}}, 0.0, {}, 0.0, {}},
    CreateVehicleFootprintsAlongTrajectoryParam{
      "SinglePointTrajectory",
      Eigen::Matrix2d{{0.0, 0.0}, {0.0, 0.0}},
      0.0,
      {{{0.0, 0.0}, 0.0}},
      0.0,
      {{{wheel_base_to_front_overhang, half_width_left},
        {wheel_base_to_front_overhang, -(half_width_right)},
        {half_wheel_base, -(half_width_right)},
        {-rear_overhang_m, -(half_width_right)},
        {-rear_overhang_m, half_width_left},
        {half_wheel_base, half_width_left},
        {wheel_base_to_front_overhang, half_width_left}}}},
    CreateVehicleFootprintsAlongTrajectoryParam{
      "NonZeroMargin",
      Eigen::Matrix2d{{0.1, 0.0}, {0.0, 0.2}},
      0.0,
      {{{0.0, 0.0}, 0.0}, {{1.0, 0.0}, 0.0}},
      1.0,
      {{{wheel_base_to_front_overhang + 0.1, half_width_left + 0.2},
        {wheel_base_to_front_overhang + 0.1, -(half_width_right + 0.2)},
        {half_wheel_base, -(half_width_right + 0.2)},
        {-(rear_overhang_m + 0.1), -(half_width_right + 0.2)},
        {-(rear_overhang_m + 0.1), half_width_left + 0.2},
        {half_wheel_base, half_width_left + 0.2},
        {wheel_base_to_front_overhang + 0.1, half_width_left + 0.2}},
       {{wheel_base_to_front_overhang + 0.1 + 1.0, half_width_left + 0.2},
        {wheel_base_to_front_overhang + 0.1 + 1.0, -(half_width_right + 0.2)},
        {half_wheel_base + 1.0, -(half_width_right + 0.2)},
        {-(rear_overhang_m + 0.1) + 1.0, -(half_width_right + 0.2)},
        {-(rear_overhang_m + 0.1) + 1.0, half_width_left + 0.2},
        {half_wheel_base + 1.0, half_width_left + 0.2},
        {wheel_base_to_front_overhang + 0.1 + 1.0, half_width_left + 0.2}}}},
    CreateVehicleFootprintsAlongTrajectoryParam{
      "NonZeroYaw",
      Eigen::Matrix2d{{0.2, 0.0}, {0.0, 0.1}},
      M_PI_2,
      {{{0.0, 0.0}, 0.0}, {{1.0, 0.0}, 0.0}},
      1.0,
      {{{wheel_base_to_front_overhang + 0.1, half_width_left + 0.2},
        {wheel_base_to_front_overhang + 0.1, -(half_width_right + 0.2)},
        {half_wheel_base, -(half_width_right + 0.2)},
        {-(rear_overhang_m + 0.1), -(half_width_right + 0.2)},
        {-(rear_overhang_m + 0.1), half_width_left + 0.2},
        {half_wheel_base, half_width_left + 0.2},
        {wheel_base_to_front_overhang + 0.1, half_width_left + 0.2}},
       {{wheel_base_to_front_overhang + 0.1 + 1.0, half_width_left + 0.2},
        {wheel_base_to_front_overhang + 0.1 + 1.0, -(half_width_right + 0.2)},
        {half_wheel_base + 1.0, -(half_width_right + 0.2)},
        {-(rear_overhang_m + 0.1) + 1.0, -(half_width_right + 0.2)},
        {-(rear_overhang_m + 0.1) + 1.0, half_width_left + 0.2},
        {half_wheel_base + 1.0, half_width_left + 0.2},
        {wheel_base_to_front_overhang + 0.1 + 1.0, half_width_left + 0.2}}}},
    CreateVehicleFootprintsAlongTrajectoryParam{
      "CurvedPathAtNinetyDeg",
      Eigen::Matrix2d::Zero(),  // No covariance/margin
      0.0,
      {{{10.0, 0.0}, M_PI_2}, {{0.0, 10.0}, M_PI}},  // Two-point trajectory
      0.0,                                           // Zero margin scale
      {
        // Expected Footprint 1: At (10.0, 0.0) facing +Y (Yaw = PI/2)
        // Formula: Global_X = Base_X - Local_Y | Global_Y = Base_Y + Local_X
        {{10.0 - half_width_left, wheel_base_to_front_overhang},  // Front-Left
         {10.0 + half_width_right,
          wheel_base_to_front_overhang},               // Front-Right (Local Y is -half_width_right)
         {10.0 + half_width_right, half_wheel_base},   // Mid-Right
         {10.0 + half_width_right, -rear_overhang_m},  // Rear-Right
         {10.0 - half_width_left, -rear_overhang_m},   // Rear-Left
         {10.0 - half_width_left, half_wheel_base},    // Mid-Left
         {10.0 - half_width_left, wheel_base_to_front_overhang}},  // Closing Front-Left

        // Expected Footprint 2: At (0.0, 10.0) facing -X (Yaw = PI)
        // Formula: Global_X = Base_X - Local_X | Global_Y = Base_Y - Local_Y
        {{-wheel_base_to_front_overhang, 10.0 - half_width_left},  // Front-Left
         {-wheel_base_to_front_overhang,
          10.0 + half_width_right},                    // Front-Right (Local Y is -half_width_right)
         {-half_wheel_base, 10.0 + half_width_right},  // Mid-Right
         {rear_overhang_m, 10.0 + half_width_right},   // Rear-Right (Local X is -rear_overhang_m)
         {rear_overhang_m, 10.0 - half_width_left},    // Rear-Left
         {-half_wheel_base, 10.0 - half_width_left},   // Mid-Left
         {-wheel_base_to_front_overhang, 10.0 - half_width_left}}  // Closing Front-Left
      }}),
  ::testing::PrintToStringParamName());

INSTANTIATE_TEST_SUITE_P(
  LaneDepartureCheckerTest, CreateVehicleFootprintsAlongPathTest,
  ::testing::Values(
    CreateVehicleFootprintsAlongPathParam{"EmptyTrajectory", {}, 0.0, {}},
    CreateVehicleFootprintsAlongPathParam{
      "SinglePointTrajectory",
      {{{0.0, 0.0}, 0.0}},
      0.0,
      {{{wheel_base_to_front_overhang, half_width_left},
        {wheel_base_to_front_overhang, -(half_width_right)},
        {half_wheel_base, -(half_width_right)},
        {-rear_overhang_m, -(half_width_right)},
        {-rear_overhang_m, half_width_left},
        {half_wheel_base, half_width_left},
        {wheel_base_to_front_overhang, half_width_left}}}},
    CreateVehicleFootprintsAlongPathParam{
      "NonZeroMargin",
      {{{0.0, 0.0}, 0.0}, {{1.0, 0.0}, 0.0}},
      0.1,
      {{{wheel_base_to_front_overhang + 0.1, half_width_left + 0.1},
        {wheel_base_to_front_overhang + 0.1, -(half_width_right + 0.1)},
        {half_wheel_base, -(half_width_right + 0.1)},
        {-(rear_overhang_m + 0.1), -(half_width_right + 0.1)},
        {-(rear_overhang_m + 0.1), half_width_left + 0.1},
        {half_wheel_base, half_width_left + 0.1},
        {wheel_base_to_front_overhang + 0.1, half_width_left + 0.1}},
       {{wheel_base_to_front_overhang + 0.1 + 1.0, half_width_left + 0.1},
        {wheel_base_to_front_overhang + 0.1 + 1.0, -(half_width_right + 0.1)},
        {half_wheel_base + 1.0, -(half_width_right + 0.1)},
        {-(rear_overhang_m + 0.1) + 1.0, -(half_width_right + 0.1)},
        {-(rear_overhang_m + 0.1) + 1.0, half_width_left + 0.1},
        {half_wheel_base + 1.0, half_width_left + 0.1},
        {wheel_base_to_front_overhang + 0.1 + 1.0, half_width_left + 0.1}}}}),
  ::testing::PrintToStringParamName());

TEST(CreateVehicleFootprintsTest, TestCreateFootprintsWithLongitudinalConfig)
{
  const auto vehicle_info = autoware::vehicle_info_utils::createVehicleInfo(
    wheel_radius_m, wheel_width_m, wheel_base_m, wheel_tread_m, front_overhang_m, rear_overhang_m,
    left_overhang_m, right_overhang_m, vehicle_height_m, max_steer_angle_rad);

  // 1. Setup Trajectory with Velocity (10 m/s)
  TrajectoryPoints trajectory;
  TrajectoryPoint p;
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  p.pose.orientation.w = 1.0;
  p.longitudinal_velocity_mps = 10.0;
  trajectory.push_back(p);

  // 2. Setup Config and Initial Margin
  const double lat_margin_init = 0.1;
  const double lon_margin_init = 0.2;
  const double velocity_scale = 0.5;
  const double extra_margin = 1.0;

  // FIX: Explicitly assign members to avoid order-based bugs
  FootprintMargin uncertainty_margin;
  uncertainty_margin.lon_m = lon_margin_init;
  uncertainty_margin.lat_m = lat_margin_init;

  LongitudinalConfig config;
  config.lon_tracking.scale = velocity_scale;
  config.lon_tracking.extra_margin_m = extra_margin;

  // 3. Compute Expected Boundaries using Variables
  const double total_lon_margin =
    lon_margin_init + (p.longitudinal_velocity_mps * velocity_scale) + extra_margin;
  const double total_lat_margin = lat_margin_init;

  const double expected_front = wheel_base_to_front_overhang + total_lon_margin;
  const double expected_rear = -(rear_overhang_m + total_lon_margin);
  const double expected_left = half_width_left + total_lat_margin;
  const double expected_right = -(half_width_right + total_lat_margin);

  const auto footprints =
    utils::create_vehicle_footprints(trajectory, vehicle_info, uncertainty_margin, config);

  ASSERT_EQ(footprints.size(), 1);
  const auto & fp = footprints[0];
  ASSERT_EQ(fp.size(), 7);

  // 4. Verify All Four Corners
  EXPECT_NEAR(fp[VehicleInfo::FrontLeftIndex].x(), expected_front, 1e-6);
  EXPECT_NEAR(fp[VehicleInfo::FrontLeftIndex].y(), expected_left, 1e-6);

  EXPECT_NEAR(fp[VehicleInfo::FrontRightIndex].x(), expected_front, 1e-6);
  EXPECT_NEAR(fp[VehicleInfo::FrontRightIndex].y(), expected_right, 1e-6);

  EXPECT_NEAR(fp[VehicleInfo::RearRightIndex].x(), expected_rear, 1e-6);
  EXPECT_NEAR(fp[VehicleInfo::RearRightIndex].y(), expected_right, 1e-6);

  EXPECT_NEAR(fp[VehicleInfo::RearLeftIndex].x(), expected_rear, 1e-6);
  EXPECT_NEAR(fp[VehicleInfo::RearLeftIndex].y(), expected_left, 1e-6);

  // 5. Plotting
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    std::vector<double> fx, fy;
    for (const auto & p_fp : fp) {
      fx.push_back(p_fp.x());
      fy.push_back(p_fp.y());
    }
    plt.plot(Args(fx, fy), Kwargs("color"_a = "blue", "label"_a = "Velocity Expanded Footprint"));
    plt.axis(Args("equal"));
    plt.legend();
    save_figure(plt, "test_create_vehicle_footprint");
  });
}

TEST(CreateVehicleFootprintsTest, TestGetFootprintSides)
{
  const auto vehicle_info = autoware::vehicle_info_utils::createVehicleInfo(
    wheel_radius_m, wheel_width_m, wheel_base_m, wheel_tread_m, front_overhang_m, rear_overhang_m,
    left_overhang_m, right_overhang_m, vehicle_height_m, max_steer_angle_rad);

  const auto footprint = vehicle_info.createFootprint(0.0, 0.0);

  // Case 1: Using Rear vertices (Default)
  {
    auto sides = utils::get_footprint_sides(footprint, false, false);

    // Right Side: Front-Right (1) -> Rear-Right (3)
    EXPECT_DOUBLE_EQ(sides.right.first.x(), wheel_base_to_front_overhang);
    EXPECT_DOUBLE_EQ(sides.right.second.x(), -rear_overhang_m);

    // Left Side: Front-Left (6) -> Rear-Left (4)
    EXPECT_DOUBLE_EQ(sides.left.first.x(), wheel_base_to_front_overhang);
    EXPECT_DOUBLE_EQ(sides.left.second.x(), -rear_overhang_m);
  }

  // Case 2: Using Center (Mid) vertices
  {
    auto sides = utils::get_footprint_sides(footprint, true, true);

    // Right Side: Front-Right (1) -> Mid-Right (2)
    EXPECT_DOUBLE_EQ(sides.right.second.x(), half_wheel_base);

    // Left Side: Front-Left (6) -> Mid-Left (5)
    EXPECT_DOUBLE_EQ(sides.left.second.x(), half_wheel_base);
  }

  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();
    auto sides = utils::get_footprint_sides(footprint, false, false);

    // Plot segments
    plt.plot(
      Args(
        std::vector<double>{sides.left.first.x(), sides.left.second.x()},
        std::vector<double>{sides.left.first.y(), sides.left.second.y()}),
      Kwargs("color"_a = "blue", "label"_a = "Left Side Segment"));

    plt.plot(
      Args(
        std::vector<double>{sides.right.first.x(), sides.right.second.x()},
        std::vector<double>{sides.right.first.y(), sides.right.second.y()}),
      Kwargs("color"_a = "red", "label"_a = "Right Side Segment"));

    plt.axis(Args("equal"));
    plt.legend();
    save_figure(plt, "test_create_vehicle_footprint");
  });
}
}  // namespace autoware::boundary_departure_checker
