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

#include "autoware/trajectory_modifier/utils.hpp"

#include <gtest/gtest.h>

#include <vector>

class TrajectoryModifierTest : public ::testing::Test
{
protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(TrajectoryModifierTest, ValidateEmptyTrajectory)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> empty_trajectory;
  EXPECT_FALSE(autoware::trajectory_modifier::utils::validate_trajectory(empty_trajectory));
}

TEST_F(TrajectoryModifierTest, ValidateNonEmptyTrajectory)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory(1);
  EXPECT_TRUE(autoware::trajectory_modifier::utils::validate_trajectory(trajectory));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}