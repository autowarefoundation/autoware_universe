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

#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"
#include "autoware/trajectory_optimizer/utils.hpp"
#include "test_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

using autoware::trajectory_optimizer::utils::TrajectoryPoints;
using autoware_planning_msgs::msg::TrajectoryPoint;
using trajectory_optimizer_test_utils::create_sample_trajectory;

class TrajectoryOptimizerUtilsTest : public ::testing::Test
{
};

TEST_F(TrajectoryOptimizerUtilsTest, ValidatePoint)
{
  TrajectoryPoint valid_point;
  valid_point.longitudinal_velocity_mps = 1.0;
  valid_point.acceleration_mps2 = 0.1;
  valid_point.pose.position.x = 1.0;
  valid_point.pose.position.y = 1.0;
  valid_point.pose.position.z = 1.0;
  valid_point.pose.orientation.x = 0.0;
  valid_point.pose.orientation.y = 0.0;
  valid_point.pose.orientation.z = 0.0;
  valid_point.pose.orientation.w = 1.0;
  ASSERT_TRUE(autoware::trajectory_optimizer::utils::validate_point(valid_point));

  TrajectoryPoint invalid_point;
  invalid_point.pose.position.x = std::nan("");
  ASSERT_FALSE(autoware::trajectory_optimizer::utils::validate_point(invalid_point));
}

TEST_F(TrajectoryOptimizerUtilsTest, ApplySpline)
{
  TrajectoryPoints points = create_sample_trajectory();
  const double interpolation_resolution_m = 0.1;
  const double max_distance_discrepancy_m = 5.0;
  const bool preserve_original_orientation = true;
  autoware::trajectory_optimizer::utils::apply_spline(
    points, interpolation_resolution_m, max_distance_discrepancy_m, preserve_original_orientation);
  ASSERT_GE(points.size(), 2);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
