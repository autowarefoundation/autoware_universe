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

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

namespace autoware::boundary_departure_checker
{
namespace
{
}  // namespace

class UncrossableBoundaryCheckerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 1. Setup Vehicle Info (Standard Sedan Size)
    vehicle_info_ = autoware::vehicle_info_utils::createVehicleInfo(
      0.383,  // front_overhang [m]
      0.235,  // rear_overhang [m]
      2.79,   // wheel_base [m]
      1.64,   // wheel_tread [m]
      1.0,    // left_overhang [m]
      1.1,    // right_overhang [m]
      2.5,    // vehicle_height [m]
      0.128,  // cg_to_rear [m]
      0.128,  // max_steer_angle [rad]
      0.70    // min_steer_angle [rad]
    );

    // 2. Param
    param_.lateral_margin_m = 0.01;   // [m]
    param_.on_time_buffer_s = 0.15;   // 150ms of continuous violation to trigger CRITICAL
    param_.off_time_buffer_s = 0.15;  // 150ms of continuous safety to clear CRITICAL
    param_.max_deceleration_mps2 = -4.0;
    param_.max_jerk_mps3 = -5.0;
    param_.brake_delay_s = 1.0;
    param_.time_to_departure_cutoff_s = 3.0;
    param_.boundary_types_to_detect = {"road_border"};

    // 3. Create a LaneletMap with a straight road border at Y = 2.0
    map_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::Point3d p1(lanelet::utils::getId(), -100.0, 2.0, 0.0);
    lanelet::Point3d p2(lanelet::utils::getId(), 100.0, 2.0, 0.0);
    lanelet::LineString3d boundary_ls(lanelet::utils::getId(), {p1, p2});
    boundary_ls.attributes()[lanelet::AttributeName::Type] = "road_border";
    map_->add(boundary_ls);
    checker_ptr_ = std::make_unique<UncrossableBoundaryChecker>(map_, param_, vehicle_info_);
  }

  static TrajectoryPoints create_trajectory(
    double start_x, double start_y, double velocity, double yaw = 0.0)
  {
    TrajectoryPoints traj;
    for (int i = 0; i < 5; ++i) {
      TrajectoryPoint p;
      p.pose.position.x = start_x + i * 5.0 * std::cos(yaw);
      p.pose.position.y = start_y + i * 5.0 * std::sin(yaw);
      p.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
      const double time = i * 0.5;
      p.time_from_start.sec = static_cast<int32_t>(time);
      p.time_from_start.nanosec = static_cast<uint32_t>((time - p.time_from_start.sec) * 1e9);
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
    for (auto & c : state.pose_with_cov.covariance) c = 0.0;
    state.velocity = velocity;
    state.acceleration = 0.0;
    state.current_time_s = current_time_s;
    return state;
  }

  double time_sec_toc(double time_increment_sec)
  {
    double y = time_increment_sec - sim_time_compensation_;
    double t = sim_time_sec_ + y;
    sim_time_compensation_ = (t - sim_time_sec_) - y;
    return sim_time_sec_ = t;
  }

  std::unique_ptr<UncrossableBoundaryChecker> checker_ptr_;
  UncrossableBoundaryDepartureParam param_;
  std::shared_ptr<lanelet::LaneletMap> map_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  double sim_time_sec_ = 0.0;
  double sim_time_compensation_ = 0.0;
};

// ==============================================================================
// 1. Initialization and Edge Cases
// ==============================================================================
TEST_F(UncrossableBoundaryCheckerTest, TestCheckDepartureEmptyTrajectory)
{
  // Arrange:
  TrajectoryPoints empty_traj;
  constexpr double init_time = 0.0;
  auto ego_state = create_ego_state(empty_traj, 0.0, init_time);

  // Act:
  auto result = checker_ptr_->update_departure_status(empty_traj, ego_state);

  // Assert:
  EXPECT_TRUE(result.status == DepartureType::NONE);
}

TEST_F(UncrossableBoundaryCheckerTest, TestCheckDepartureZeroVelocity)
{
  // Arrange:
  auto traj = create_trajectory(0.0, 0.0, 0.0);
  constexpr double init_time = 0.0;
  auto ego_state = create_ego_state(traj, 0.0, init_time);

  // Act:
  auto result = checker_ptr_->update_departure_status(traj, ego_state);

  // Assert:
  EXPECT_TRUE(result.status == DepartureType::NONE);
}

// ==============================================================================
// 2. Hysteresis and Time Buffering Tests
// ==============================================================================

// clang-format off
/**
 * TestTimeBufferingHysteresis:
 *
 *    Y ^
 *      |   Boundary (Uncrossable)
 *  2.0 +-------------------------------------------------
 *      |          / V_danger (Yaw=0.1, crosses at Y=2.0)
 *  1.0 |         V
 *      |        /   >>> Predicted Trajectory
 *  0.0 +-------V-----------------------------------------> X
 *              V_safe (Safe Y=0.0)
 *
 * Evaluates hysteresis logic: CRITICAL is held during OFF-buffer even if
 * the vehicle returns to Safe Zone, and delayed during ON-buffer.
 */
// clang-format on
TEST_F(UncrossableBoundaryCheckerTest, TestTimeBufferingHysteresis)
{
  // 1-line summary: Evaluates the hysteresis logic and time buffering (ON/OFF) for critical
  // departures.

  // Arrange:
  double safe_y = 0.0;          // safe lateral position [m]
  double danger_y = 1.0;        // danger lateral position [m]
  double danger_yaw = 0.1;      // yaw [rad] to force boundary crossing at Y=2.0
  double test_velocity = 10.0;  // velocity [m/s]

  // STEP 1: Safe Driving
  auto traj_safe = create_trajectory(0.0, safe_y, test_velocity, 0.0);
  auto state_safe = create_ego_state(traj_safe, test_velocity, time_sec_toc(0.0));

  // Act:
  auto res1 = checker_ptr_->update_departure_status(traj_safe, state_safe);

  // Assert:
  EXPECT_TRUE(res1.status == DepartureType::NONE) << "Should be NONE when far from boundary.";

  // STEP 2: Instantly teleport to Danger Zone. Time increase less than ON buffer.
  // Arrange:
  auto traj_danger = create_trajectory(0.0, danger_y, test_velocity, danger_yaw);
  auto state_danger = create_ego_state(traj_danger, test_velocity, time_sec_toc(0.1));

  // Act:
  auto res2 = checker_ptr_->update_departure_status(traj_danger, state_danger);

  // Assert:
  EXPECT_TRUE(res2.status == DepartureType::NONE) << "Should be NONE due to ON time buffer.";

  // STEP 3: Wait for ON buffer to expire. Time increase by 0.2 second (Danger is continuous)
  // Act:
  auto res3 = checker_ptr_->update_departure_status(
    traj_danger, create_ego_state(traj_danger, test_velocity, time_sec_toc(0.2)));

  // Assert:
  EXPECT_TRUE(res3.status == DepartureType::CRITICAL)
    << "Should trigger CRITICAL after buffer expires.";

  // STEP 4: Instantly teleport back to Safe Zone
  // Act:
  auto res4 = checker_ptr_->update_departure_status(
    traj_safe, create_ego_state(traj_safe, test_velocity, time_sec_toc(0.0)));

  // Assert:
  EXPECT_TRUE(res4.status == DepartureType::CRITICAL)
    << "Should hold CRITICAL due to OFF time buffer.";

  // STEP 5: Wait for OFF buffer to expire (Safety is continuous)
  // Act:
  auto res5 = checker_ptr_->update_departure_status(
    traj_safe, create_ego_state(traj_safe, test_velocity, time_sec_toc(0.2)));

  // Assert:
  EXPECT_TRUE(res5.status == DepartureType::NONE)
    << "Should return to NONE after OFF buffer expires.";

  // STEP 6: Visualization
}
}  // namespace autoware::boundary_departure_checker
