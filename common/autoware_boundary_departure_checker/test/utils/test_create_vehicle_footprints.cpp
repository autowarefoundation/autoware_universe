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

#include "autoware/boundary_departure_checker/footprints_generator.hpp"
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
  std::vector<LinearRing2d> expected_footprints;
};

std::ostream & operator<<(std::ostream & os, const CreateVehicleFootprintsAlongTrajectoryParam & p)
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

TEST_P(CreateVehicleFootprintsAlongTrajectoryTest, test_create_vehicle_footprints)
{
  const auto & p = GetParam();
  const auto trajectory_points = create_trajectory_points(p.trajectory_points);
  const auto pose_with_covariance = create_pose_with_covariance(p.covariance_xy, p.yaw);

  // Notice: footprint_margin_scale is purposely omitted based on the new implementation
  const auto footprints =
    footprints::generate(trajectory_points, vehicle_info, pose_with_covariance);

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

INSTANTIATE_TEST_SUITE_P(
  LaneDepartureCheckerTest, CreateVehicleFootprintsAlongTrajectoryTest,
  ::testing::Values(
    CreateVehicleFootprintsAlongTrajectoryParam{
      "EmptyTrajectory", Eigen::Matrix2d{{0.0, 0.0}, {0.0, 0.0}}, 0.0, {}, {}},
    CreateVehicleFootprintsAlongTrajectoryParam{
      "SinglePointTrajectory",
      Eigen::Matrix2d{{0.0, 0.0}, {0.0, 0.0}},
      0.0,
      {{{0.0, 0.0}, 0.0}},
      {{{wheel_base_to_front_overhang, half_width_left},
        {wheel_base_to_front_overhang, -(half_width_right)},
        {half_wheel_base, -(half_width_right)},
        {-rear_overhang_m, -(half_width_right)},
        {-rear_overhang_m, half_width_left},
        {half_wheel_base, half_width_left},
        {wheel_base_to_front_overhang, half_width_left}}}},
    CreateVehicleFootprintsAlongTrajectoryParam{
      "CurvedPathAtNinetyDeg",
      Eigen::Matrix2d::Zero(),  // No covariance/margin
      0.0,
      {{{10.0, 0.0}, M_PI_2}, {{0.0, 10.0}, M_PI}},  // Two-point trajectory
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
}  // namespace autoware::boundary_departure_checker
