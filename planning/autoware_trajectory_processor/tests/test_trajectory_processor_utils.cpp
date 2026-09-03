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

#include "autoware/trajectory_processor/utils.hpp"

#include <gtest/gtest.h>

#include <cmath>

using autoware_planning_msgs::msg::TrajectoryPoint;

TEST(TrajectoryProcessorUtilsTest, ValidatesTrajectoryPoint)
{
  TrajectoryPoint valid_point;
  valid_point.longitudinal_velocity_mps = 1.0;
  valid_point.acceleration_mps2 = 0.1;
  valid_point.pose.position.x = 1.0;
  valid_point.pose.position.y = 1.0;
  valid_point.pose.position.z = 1.0;
  valid_point.pose.orientation.w = 1.0;
  EXPECT_TRUE(autoware::trajectory_processor::utils::validate_point(valid_point));

  TrajectoryPoint invalid_point;
  invalid_point.pose.position.x = std::nan("");
  EXPECT_FALSE(autoware::trajectory_processor::utils::validate_point(invalid_point));
}
