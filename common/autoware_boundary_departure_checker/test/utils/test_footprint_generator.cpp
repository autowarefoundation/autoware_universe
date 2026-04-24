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
#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace autoware::boundary_departure_checker
{
namespace
{
}  // namespace

class FootprintGeneratorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 1. Setup Vehicle Info
    vehicle_info_ = autoware::vehicle_info_utils::createVehicleInfo(
      0.383, 0.235, 2.79, 1.64, 1.0, 1.1, 2.5, 0.128, 0.128, 0.70);

    // 2. Setup Trajectory
    TrajectoryPoint p1;
    p1.pose.position.x = 0.0;
    p1.pose.position.y = 0.0;
    p1.pose.orientation.w = 1.0;
    p1.longitudinal_velocity_mps = 10.0;
    p1.time_from_start.sec = static_cast<int32_t>(0.0);
    p1.time_from_start.nanosec = static_cast<uint32_t>((0.0 - p1.time_from_start.sec) * 1e9);
    pred_traj_.push_back(p1);

    TrajectoryPoint p2 = p1;
    p2.pose.position.x = 1.0;
    p2.time_from_start.sec = static_cast<int32_t>(0.1);
    p2.time_from_start.nanosec = static_cast<uint32_t>((0.1 - p2.time_from_start.sec) * 1e9);
    pred_traj_.push_back(p2);

    // 3. Setup covariance
    pose_with_cov_.pose.orientation.w = 1.0;
    for (auto & c : pose_with_cov_.covariance) {
      c = 0.0;
    }
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  TrajectoryPoints pred_traj_;
  geometry_msgs::msg::PoseWithCovariance pose_with_cov_;
};

TEST_F(FootprintGeneratorTest, TestNormalFootprintGenerator)
{
  // 1-line summary: Verifies that footprints are correctly generated along a trajectory.

  // Act:
  const auto footprints = footprints::generate(pred_traj_, vehicle_info_, pose_with_cov_);

  // Assert:
  ASSERT_EQ(footprints.size(), pred_traj_.size());
  const double expected_x = vehicle_info_.wheel_base_m + vehicle_info_.front_overhang_m;
  EXPECT_DOUBLE_EQ(footprints[0][VehicleInfo::FrontLeftIndex].x(), expected_x);
}

TEST_F(FootprintGeneratorTest, TestSteeringFootprintGeneratorEmptyTrajectory)
{
  // Arrange:
  TrajectoryPoints empty_traj;

  // Act:
  const auto footprints = footprints::generate(empty_traj, vehicle_info_, pose_with_cov_);

  // Assert:
  EXPECT_TRUE(footprints.empty());
}

TEST_F(FootprintGeneratorTest, TestGetSidesFromFootprintsEmpty)
{
  // Arrange:
  footprints::Footprints empty_footprints;

  // Act:
  const auto sides_array = footprints::get_sides_from_footprints(empty_footprints);

  // Assert:
  EXPECT_TRUE(sides_array.empty());
}

TEST_F(FootprintGeneratorTest, TestGetSidesFromFootprints)
{
  // 1-line summary: Verifies extraction of left and right side segments from footprints.

  // Arrange:
  using autoware::vehicle_info_utils::VehicleInfo;
  const auto base_fp = vehicle_info_.createFootprint(0.0, 0.0);
  footprints::Footprints test_footprints = {base_fp};

  auto offset_fp = base_fp;
  for (auto & p : offset_fp) {
    p = autoware_utils_geometry::Point2d{p.x() + 5.0, p.y()};  // 5.0m offset
  }
  test_footprints.push_back(offset_fp);

  // Act:
  const auto sides_array = footprints::get_sides_from_footprints(test_footprints);

  // Assert:
  ASSERT_EQ(sides_array.size(), 2);

  EXPECT_DOUBLE_EQ(sides_array[0].left.first.x(), base_fp[VehicleInfo::FrontLeftIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[0].left.first.y(), base_fp[VehicleInfo::FrontLeftIndex].y());
  EXPECT_DOUBLE_EQ(sides_array[0].left.second.x(), base_fp[VehicleInfo::RearLeftIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[0].left.second.y(), base_fp[VehicleInfo::RearLeftIndex].y());

  EXPECT_DOUBLE_EQ(sides_array[0].right.first.x(), base_fp[VehicleInfo::FrontRightIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[0].right.first.y(), base_fp[VehicleInfo::FrontRightIndex].y());
  EXPECT_DOUBLE_EQ(sides_array[0].right.second.x(), base_fp[VehicleInfo::RearRightIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[0].right.second.y(), base_fp[VehicleInfo::RearRightIndex].y());

  EXPECT_DOUBLE_EQ(sides_array[1].left.first.x(), offset_fp[VehicleInfo::FrontLeftIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[1].left.first.y(), offset_fp[VehicleInfo::FrontLeftIndex].y());
  EXPECT_DOUBLE_EQ(sides_array[1].right.second.x(), offset_fp[VehicleInfo::RearRightIndex].x());
  EXPECT_DOUBLE_EQ(sides_array[1].right.second.y(), offset_fp[VehicleInfo::RearRightIndex].y());
}
}  // namespace autoware::boundary_departure_checker
