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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_temporal_mpt_optimizer_utils.hpp"
#include "test_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using autoware::trajectory_optimizer::plugin::trajectory_temporal_mpt_optimizer_utils::
  build_temporal_mpt_references;
using autoware::trajectory_optimizer::plugin::trajectory_temporal_mpt_optimizer_utils::
  compute_yaw_psi_bias;
using autoware::trajectory_optimizer::plugin::trajectory_temporal_mpt_optimizer_utils::
  find_closest_trajectory_index;
using trajectory_optimizer_test_utils::create_point;
using trajectory_optimizer_test_utils::create_point_with_yaw;

class TemporalMPTOptimizerUtilsTest : public ::testing::Test
{
};

TEST_F(TemporalMPTOptimizerUtilsTest, FindClosestTrajectoryIndex_AtFirstPoint)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;
  for (int i = 0; i < 5; ++i) {
    points.push_back(create_point(static_cast<double>(i), 0.0));
  }

  EXPECT_EQ(find_closest_trajectory_index(points, 0.0, 0.0), 0U);
}

TEST_F(TemporalMPTOptimizerUtilsTest, FindClosestTrajectoryIndex_OffsetFromStart)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;
  for (int i = 0; i < 5; ++i) {
    points.push_back(create_point(static_cast<double>(i), 0.0));
  }

  EXPECT_EQ(find_closest_trajectory_index(points, 2.4, 0.0), 2U);
  EXPECT_EQ(find_closest_trajectory_index(points, 3.6, 0.0), 4U);
}

TEST_F(TemporalMPTOptimizerUtilsTest, ComputeYawPsiBias_NoBranchDifference)
{
  EXPECT_NEAR(compute_yaw_psi_bias(0.1, 0.05), 0.0, 1e-9);
}

TEST_F(TemporalMPTOptimizerUtilsTest, ComputeYawPsiBias_TwoPiBranchDifference)
{
  constexpr double two_pi = 2.0 * M_PI;
  const double x0_yaw = two_pi + 0.2;
  const double yaw_at_start = 0.1;

  EXPECT_NEAR(compute_yaw_psi_bias(x0_yaw, yaw_at_start), two_pi, 1e-9);
}

TEST_F(TemporalMPTOptimizerUtilsTest, ComputeYawPsiBias_NegativeTwoPiBranchDifference)
{
  constexpr double two_pi = 2.0 * M_PI;
  const double x0_yaw = -two_pi + 0.1;
  const double yaw_at_start = 0.2;

  EXPECT_NEAR(compute_yaw_psi_bias(x0_yaw, yaw_at_start), -two_pi, 1e-9);
}

TEST_F(TemporalMPTOptimizerUtilsTest, BuildReferences_StageZeroIsEgoCentered)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;
  for (int i = 0; i < 10; ++i) {
    points.push_back(create_point(static_cast<double>(i), 0.0, 5.0f));
  }

  const std::array<double, temporal_mpt::NX> x0 = {0.0, 0.0, 0.0, 5.0};
  const auto refs = build_temporal_mpt_references(points, x0);

  EXPECT_EQ(refs.start_idx, 0U);
  EXPECT_NEAR(refs.yref_stage0[0], 0.0, 1e-9);
  EXPECT_NEAR(refs.yref_stage0[1], 0.0, 1e-9);
  EXPECT_NEAR(refs.yref_stage0[2], 0.0, 1e-9);
  EXPECT_NEAR(refs.yref_stage0[3], 5.0, 1e-9);
  EXPECT_NEAR(refs.x0_local[0], 0.0, 1e-9);
  EXPECT_NEAR(refs.x0_local[1], 0.0, 1e-9);
}

TEST_F(TemporalMPTOptimizerUtilsTest, BuildReferences_HorizonSamplesSequentialIndices)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;
  for (int i = 0; i < 10; ++i) {
    points.push_back(create_point(static_cast<double>(i), static_cast<double>(i), 2.0f));
  }

  const std::array<double, temporal_mpt::NX> x0 = {0.0, 0.0, 0.0, 2.0};
  const auto refs = build_temporal_mpt_references(points, x0);

  const auto & stage1 = refs.stage_yrefs[0];
  EXPECT_NEAR(stage1[0], 1.0, 1e-9);
  EXPECT_NEAR(stage1[1], 1.0, 1e-9);
  EXPECT_NEAR(stage1[3], 2.0, 1e-9);

  const auto & stage3 = refs.stage_yrefs[2];
  EXPECT_NEAR(stage3[0], 3.0, 1e-9);
  EXPECT_NEAR(stage3[1], 3.0, 1e-9);
}

TEST_F(TemporalMPTOptimizerUtilsTest, BuildReferences_ShortTrajectoryClampsToLastPoint)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0, 1.0f));
  points.push_back(create_point(1.0, 0.0, 2.0f));
  points.push_back(create_point(2.0, 5.0, 3.0f));

  const std::array<double, temporal_mpt::NX> x0 = {0.0, 0.0, 0.0, 1.0};
  const auto refs = build_temporal_mpt_references(points, x0);

  EXPECT_EQ(refs.terminal_idx, 2U);

  const auto & last_stage = refs.stage_yrefs[temporal_mpt::N - 2];
  EXPECT_NEAR(last_stage[0], 2.0, 1e-9);
  EXPECT_NEAR(last_stage[1], 5.0, 1e-9);
  EXPECT_NEAR(last_stage[3], 3.0, 1e-9);

  EXPECT_NEAR(refs.terminal_yref[0], 2.0, 1e-9);
  EXPECT_NEAR(refs.terminal_yref[1], 5.0, 1e-9);
  EXPECT_NEAR(refs.terminal_yref[3], 3.0, 1e-9);
}

TEST_F(TemporalMPTOptimizerUtilsTest, BuildReferences_AppliesYawBiasToHorizon)
{
  constexpr double two_pi = 2.0 * M_PI;
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;
  points.push_back(create_point_with_yaw(0.0, 0.0, two_pi + 0.1, 1.0f));
  points.push_back(create_point_with_yaw(1.0, 0.0, 0.2, 1.0f));

  const std::array<double, temporal_mpt::NX> x0 = {0.0, 0.0, two_pi + 0.1, 1.0};
  const auto refs = build_temporal_mpt_references(points, x0);

  EXPECT_NEAR(refs.psi_bias, two_pi, 1e-9);
  EXPECT_NEAR(refs.stage_yrefs[0][2], 0.2 + two_pi, 1e-9);
}

TEST_F(TemporalMPTOptimizerUtilsTest, BuildReferences_ClampNegativeVelocityToZero)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;
  points.push_back(create_point(0.0, 0.0, -1.0f));
  points.push_back(create_point(1.0, 0.0, -2.0f));

  const std::array<double, temporal_mpt::NX> x0 = {0.0, 0.0, 0.0, 0.0};
  const auto refs = build_temporal_mpt_references(points, x0);

  EXPECT_NEAR(refs.stage_yrefs[0][3], 0.0, 1e-9);
  EXPECT_NEAR(refs.terminal_yref[3], 0.0, 1e-9);
}

TEST_F(TemporalMPTOptimizerUtilsTest, BuildReferences_StartIdxFromClosestPoint)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;
  for (int i = 0; i < 5; ++i) {
    points.push_back(create_point(static_cast<double>(i * 10), 0.0, 1.0f));
  }

  const std::array<double, temporal_mpt::NX> x0 = {22.0, 0.0, 0.0, 1.0};
  const auto refs = build_temporal_mpt_references(points, x0);

  EXPECT_EQ(refs.start_idx, 2U);
  EXPECT_NEAR(refs.stage_yrefs[0][0], 30.0 - x0[0], 1e-9);
}
