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

#include "autoware/boundary_departure_checker/type_alias.hpp"
#include "autoware/boundary_departure_checker/uncrossable_boundary_checker.hpp"
#include "test_plot_utils.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

namespace autoware::boundary_departure_checker
{

class UncrossableBoundaryCheckerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // 1. Setup Vehicle Info (Standard Sedan Size)
    vehicle_info_ = autoware::vehicle_info_utils::createVehicleInfo(
      0.383,  // wheel radius [m]
      0.235,  // wheel width [m]
      2.79,   // wheelbase [m]
      1.64,   // wheel tread [m]
      1.0,    // front overhang [m]
      1.1,    // rear overhang [m]
      0.128,  // left overhang [m]
      0.128,  // right overhang [m]
      2.5,    // vehicle height [m]
      0.70);  // max steer angle [rad]

    // 2. Setup Clock and Param
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    param_.lateral_margin_m = 0.01;
    param_.on_time_buffer_s = 0.15;   // Require 150ms of continuous violation to trigger CRITICAL
    param_.off_time_buffer_s = 0.15;  // Require 150ms of continuous safety to clear CRITICAL
    param_.max_deceleration_mps2 = -4.0;
    param_.max_jerk_mps3 = -5.0;
    param_.brake_delay_s = 1.0;
    param_.time_to_departure_cutoff_s = 3.0;
    param_.boundary_types_to_detect = {"road_border"};

    // 3. Create a LaneletMap with a straight road border at Y = 2.0
    map_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::Point3d p1(lanelet::utils::getId(), -100.0, 2.0, 0.0);  // road boundary start
    lanelet::Point3d p2(lanelet::utils::getId(), 100.0, 2.0, 0.0);   // road boundary end
    lanelet::LineString3d boundary_ls(lanelet::utils::getId(), {p1, p2});
    boundary_ls.attributes()[lanelet::AttributeName::Type] = "road_border";
    map_->add(boundary_ls);
  }

  void TearDown() override { rclcpp::shutdown(); }

  static TrajectoryPoints create_trajectory(
    double start_x, double start_y, double velocity, double yaw = 0.0)
  {
    TrajectoryPoints traj;
    for (int i = 0; i < 5; ++i) {
      TrajectoryPoint p;
      p.pose.position.x = start_x + i * 5.0 * std::cos(yaw);
      p.pose.position.y = start_y + i * 5.0 * std::sin(yaw);
      p.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
      p.time_from_start = rclcpp::Duration::from_seconds(i * 0.5);
      p.longitudinal_velocity_mps = static_cast<float>(velocity);
      traj.push_back(p);
    }
    return traj;
  }

  static EgoDynamicState create_ego_state(
    const TrajectoryPoints & traj, double velocity, double current_time_s = 0.0)
  {
    EgoDynamicState state;
    if (!traj.empty()) {
      state.pose_with_cov.pose = traj.front().pose;
    }
    // Set zero covariance to avoid dynamic margin inflation in tests
    for (auto & c : state.pose_with_cov.covariance) c = 0.0;
    state.velocity = velocity;
    state.acceleration = 0.0;
    state.current_time_s = current_time_s;
    return state;
  }

  UncrossableBoundaryChecker checker_;
  std::shared_ptr<rclcpp::Clock> clock_;
  UncrossableBoundaryDepartureParam param_;
  std::shared_ptr<lanelet::LaneletMap> map_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
};

// ==============================================================================
// 1. Initialization and Edge Cases
// ==============================================================================

// Evaluates initialization failure when the lanelet map is null.
TEST_F(UncrossableBoundaryCheckerTest, TestInitializationFailure)
{
  // Arrange:
  UncrossableBoundaryChecker bad_checker;
  bad_checker.set_lanelet_map(nullptr);

  // Act:
  auto result = bad_checker.initialize();

  // Assert:
  EXPECT_FALSE(result.has_value());
}

// Evaluates departure check behavior with an empty trajectory.
TEST_F(UncrossableBoundaryCheckerTest, TestCheckDepartureEmptyTrajectory)
{
  // Arrange:
  TrajectoryPoints empty_traj;
  auto ego_state = create_ego_state(empty_traj, 0.0, clock_->now().seconds());

  checker_.set_lanelet_map(map_);
  checker_.set_param(param_);
  ASSERT_TRUE(checker_.initialize().has_value());

  // Act:
  auto check = checker_.check_departure(empty_traj, vehicle_info_, ego_state);

  // Assert:
  EXPECT_TRUE(check.has_value());
}

// Evaluates departure check behavior when vehicle velocity is zero.
TEST_F(UncrossableBoundaryCheckerTest, TestCheckDepartureZeroVelocity)
{
  // Arrange:
  auto traj = create_trajectory(0.0, 0.0, 0.0);  // 0 velocity
  auto ego_state = create_ego_state(traj, 0.0, clock_->now().seconds());

  checker_.set_lanelet_map(map_);
  checker_.set_param(param_);
  ASSERT_TRUE(checker_.initialize().has_value());

  // Act:
  auto result = checker_.check_departure(traj, vehicle_info_, ego_state);

  // Assert:
  EXPECT_EQ(result->status, DepartureType::NONE);
}

// ==============================================================================
// 2. Hysteresis and Time Buffering Tests
// ==============================================================================

// Evaluates state transition hysteresis (ON/OFF buffers) during boundary departure.
TEST_F(UncrossableBoundaryCheckerTest, TestTimeBufferingHysteresis)
{
  // Arrange:
  checker_.set_lanelet_map(map_);
  checker_.set_param(param_);
  ASSERT_TRUE(checker_.initialize().has_value());

  double safe_y = 0.0;
  double danger_y = 1.0;
  double danger_yaw = 0.1;  // angled toward boundary
  double test_velocity = 10.0;

  // STEP 1: Safe Driving
  auto traj_safe = create_trajectory(0.0, safe_y, test_velocity, 0.0);
  auto state_safe = create_ego_state(traj_safe, test_velocity, clock_->now().seconds());

  // Act & Assert:
  auto res1 = checker_.check_departure(traj_safe, vehicle_info_, state_safe);
  ASSERT_TRUE(res1.has_value());
  EXPECT_EQ(res1->status, DepartureType::NONE);

  // STEP 2: Instantly teleport to Danger Zone
  auto traj_danger = create_trajectory(0.0, danger_y, test_velocity, danger_yaw);
  auto state_danger = create_ego_state(traj_danger, test_velocity, clock_->now().seconds());

  auto res2 = checker_.check_departure(traj_danger, vehicle_info_, state_danger);
  ASSERT_TRUE(res2.has_value());
  EXPECT_EQ(res2->status, DepartureType::NONE);  // suppressed by ON buffer

  // STEP 3: Wait for ON buffer to expire
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  auto res3 = checker_.check_departure(
    traj_danger, vehicle_info_,
    create_ego_state(traj_danger, test_velocity, clock_->now().seconds()));
  ASSERT_TRUE(res3.has_value());
  EXPECT_EQ(res3->status, DepartureType::CRITICAL);

  // STEP 4: Instantly teleport back to Safe Zone
  auto res4 = checker_.check_departure(
    traj_safe, vehicle_info_, create_ego_state(traj_safe, test_velocity, clock_->now().seconds()));
  ASSERT_TRUE(res4.has_value());
  EXPECT_EQ(res4->status, DepartureType::CRITICAL);  // held by OFF buffer

  // STEP 5: Wait for OFF buffer to expire
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  auto res5 = checker_.check_departure(
    traj_safe, vehicle_info_, create_ego_state(traj_safe, test_velocity, clock_->now().seconds()));
  ASSERT_TRUE(res5.has_value());
  EXPECT_EQ(res5->status, DepartureType::NONE);

  // Assert: (Plotting)
  plot_hysteresis_results(traj_safe, traj_danger, vehicle_info_);
}
}  // namespace autoware::boundary_departure_checker
=============================================

// Evaluates state transition hysteresis (ON/OFF buffers) during boundary departure.
TEST_F(UncrossableBoundaryCheckerTest, TestTimeBufferingHysteresis)
{
  // Arrange:
  checker_.set_lanelet_map(map_);
  checker_.set_param(param_);
  ASSERT_TRUE(checker_.initialize().has_value());

  double safe_y = 0.0;
  double danger_y = 1.0;
  double danger_yaw = 0.1;  // angled toward boundary
  double test_velocity = 10.0;

  // STEP 1: Safe Driving
  auto traj_safe = create_trajectory(0.0, safe_y, test_velocity, 0.0);
  auto state_safe = create_ego_state(traj_safe, test_velocity, clock_->now().seconds());

  // Act & Assert:
  auto res1 = checker_.check_departure(traj_safe, vehicle_info_, state_safe);
  ASSERT_TRUE(res1.has_value());
  EXPECT_EQ(res1->status, DepartureType::NONE);

  // STEP 2: Instantly teleport to Danger Zone
  auto traj_danger = create_trajectory(0.0, danger_y, test_velocity, danger_yaw);
  auto state_danger = create_ego_state(traj_danger, test_velocity, clock_->now().seconds());

  auto res2 = checker_.check_departure(traj_danger, vehicle_info_, state_danger);
  ASSERT_TRUE(res2.has_value());
  EXPECT_EQ(res2->status, DepartureType::NONE);  // suppressed by ON buffer

  // STEP 3: Wait for ON buffer to expire
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  auto res3 = checker_.check_departure(
    traj_danger, vehicle_info_,
    create_ego_state(traj_danger, test_velocity, clock_->now().seconds()));
  ASSERT_TRUE(res3.has_value());
  EXPECT_EQ(res3->status, DepartureType::CRITICAL);

  // STEP 4: Instantly teleport back to Safe Zone
  auto res4 = checker_.check_departure(
    traj_safe, vehicle_info_, create_ego_state(traj_safe, test_velocity, clock_->now().seconds()));
  ASSERT_TRUE(res4.has_value());
  EXPECT_EQ(res4->status, DepartureType::CRITICAL);  // held by OFF buffer

  // STEP 5: Wait for OFF buffer to expire
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  auto res5 = checker_.check_departure(
    traj_safe, vehicle_info_, create_ego_state(traj_safe, test_velocity, clock_->now().seconds()));
  ASSERT_TRUE(res5.has_value());
  EXPECT_EQ(res5->status, DepartureType::NONE);

  // ----------------------------------------------------------------------------
  // STEP 6: Visualization
  // ----------------------------------------------------------------------------
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();

    // Plot the Boundary (Y = 2.0)
    std::vector<double> bound_x = {-10.0, 30.0};
    std::vector<double> bound_y = {2.0, 2.0};
    plt.plot(
      Args(bound_x, bound_y),
      Kwargs("color"_a = "red", "linewidth"_a = 2.0, "label"_a = "Road Boundary"));

    // Plot the Safe Trajectory (Y = 0.0)
    std::vector<double> safe_x, safe_y;
    for (const auto & p : traj_safe) {
      safe_x.push_back(p.pose.position.x);
      safe_y.push_back(p.pose.position.y);
    }
    plt.plot(
      Args(safe_x, safe_y), Kwargs(
                              "color"_a = "green", "marker"_a = "o", "linestyle"_a = "--",
                              "label"_a = "Safe Trajectory (NONE)"));

    // Plot the Danger Trajectory (Angled)
    std::vector<double> danger_x, danger_y;
    for (const auto & p : traj_danger) {
      danger_x.push_back(p.pose.position.x);
      danger_y.push_back(p.pose.position.y);
    }
    plt.plot(
      Args(danger_x, danger_y), Kwargs(
                                  "color"_a = "orange", "marker"_a = "x", "linestyle"_a = "--",
                                  "label"_a = "Danger Trajectory (CRITICAL)"));

    // Draw the actual vehicle footprint on the Danger Trajectory
    auto local_fp = vehicle_info_.createFootprint(0.0, 0.0);
    auto transformed_fp = autoware_utils_geometry::transform_vector(
      local_fp, autoware_utils_geometry::pose2transform(traj_danger.front().pose));

    std::vector<double> box_x, box_y;
    for (const auto & pt : transformed_fp) {
      box_x.push_back(pt.x());
      box_y.push_back(pt.y());
    }
    plt.plot(
      Args(box_x, box_y),
      Kwargs("color"_a = "gray", "alpha"_a = 0.5, "label"_a = "Ego Vehicle Footprint"));

    plt.title(Args("Hysteresis Test: Top-Down View"));
    plt.xlabel(Args("X [m]"));
    plt.ylabel(Args("Y [m]"));
    plt.axis(Args("equal"));
    plt.legend(Kwargs("loc"_a = "lower right"));

    save_figure(plt, "test_uncrossable_boundary_checker");
  });
}
}  // namespace autoware::boundary_departure_checker
