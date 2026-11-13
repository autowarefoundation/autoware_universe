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

#include "stop_profile.hpp"
#include "trajectory_latcher.hpp"

#include <tier4_system_msgs/msg/in_lane_stop_trigger.hpp>

#include <gtest/gtest.h>

#include <optional>

namespace autoware::in_lane_mrm_planner
{
namespace
{
using tier4_system_msgs::msg::InLaneStopTrigger;

Trajectory make_trajectory(const float velocity)
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  TrajectoryPoint point;
  point.longitudinal_velocity_mps = velocity;
  trajectory.points.push_back(point);
  return trajectory;
}

}  // namespace

TEST(StopProfileTest, ConvertsTriggerProfile)
{
  EXPECT_EQ(from_trigger_profile(InLaneStopTrigger::PROFILE_MODERATE), StopProfile::MODERATE);
  EXPECT_EQ(from_trigger_profile(InLaneStopTrigger::PROFILE_EMERGENCY), StopProfile::EMERGENCY);
  EXPECT_FALSE(from_trigger_profile(InLaneStopTrigger::PROFILE_UNKNOWN).has_value());
  EXPECT_FALSE(from_trigger_profile(200).has_value());

  EXPECT_EQ(to_trigger_profile(StopProfile::MODERATE), InLaneStopTrigger::PROFILE_MODERATE);
  EXPECT_EQ(to_trigger_profile(StopProfile::EMERGENCY), InLaneStopTrigger::PROFILE_EMERGENCY);
}

TEST(StopProfileTest, DecidesLatchAction)
{
  const std::optional<StopProfile> none;
  EXPECT_EQ(decide_latch_action(false, StopProfile::MODERATE, none), LatchAction::KEEP);
  EXPECT_EQ(
    decide_latch_action(false, StopProfile::MODERATE, StopProfile::EMERGENCY),
    LatchAction::UNLATCH);
  EXPECT_EQ(decide_latch_action(true, StopProfile::EMERGENCY, none), LatchAction::LATCH);
  EXPECT_EQ(
    decide_latch_action(true, StopProfile::MODERATE, StopProfile::MODERATE), LatchAction::KEEP);
  EXPECT_EQ(
    decide_latch_action(true, StopProfile::EMERGENCY, StopProfile::MODERATE),
    LatchAction::RE_LATCH);
  EXPECT_EQ(
    decide_latch_action(true, StopProfile::MODERATE, StopProfile::EMERGENCY),
    LatchAction::RE_LATCH);
}

TEST(TrajectoryLatcherProfileTest, LatchesRequestedProfileCandidate)
{
  TrajectoryLatcher latcher;
  latcher.update_candidate(StopProfile::MODERATE, make_trajectory(3.0F));
  latcher.update_candidate(StopProfile::EMERGENCY, make_trajectory(6.0F));

  // Unlatched output is the standby (moderate) candidate.
  ASSERT_TRUE(latcher.output().has_value());
  EXPECT_FLOAT_EQ(latcher.output()->points.front().longitudinal_velocity_mps, 3.0F);

  ASSERT_TRUE(latcher.latch(StopProfile::EMERGENCY));
  EXPECT_EQ(latcher.latched_profile(), StopProfile::EMERGENCY);
  EXPECT_FLOAT_EQ(latcher.output()->points.front().longitudinal_velocity_mps, 6.0F);
}

TEST(TrajectoryLatcherProfileTest, ReLatchSwitchesToNewProfileCandidate)
{
  TrajectoryLatcher latcher;
  latcher.update_candidate(StopProfile::MODERATE, make_trajectory(3.0F));
  ASSERT_TRUE(latcher.latch(StopProfile::MODERATE));

  // Fresh candidates planned while latched do not change the latched output...
  latcher.update_candidate(StopProfile::MODERATE, make_trajectory(2.0F));
  latcher.update_candidate(StopProfile::EMERGENCY, make_trajectory(1.0F));
  EXPECT_FLOAT_EQ(latcher.output()->points.front().longitudinal_velocity_mps, 3.0F);

  // ...until the profile is re-latched.
  ASSERT_TRUE(latcher.latch(StopProfile::EMERGENCY));
  EXPECT_EQ(latcher.latched_profile(), StopProfile::EMERGENCY);
  EXPECT_FLOAT_EQ(latcher.output()->points.front().longitudinal_velocity_mps, 1.0F);

  latcher.unlatch();
  EXPECT_FALSE(latcher.latched_profile().has_value());
  EXPECT_FLOAT_EQ(latcher.output()->points.front().longitudinal_velocity_mps, 2.0F);
}

TEST(TrajectoryLatcherProfileTest, LatchFailsWithoutCandidateOfProfile)
{
  TrajectoryLatcher latcher;
  latcher.update_candidate(StopProfile::MODERATE, make_trajectory(3.0F));

  EXPECT_FALSE(latcher.latch(StopProfile::EMERGENCY));
  EXPECT_FALSE(latcher.is_latched());
  EXPECT_FALSE(latcher.has_candidate(StopProfile::EMERGENCY));

  // A failed re-latch keeps the current latch.
  ASSERT_TRUE(latcher.latch(StopProfile::MODERATE));
  EXPECT_FALSE(latcher.latch(StopProfile::EMERGENCY));
  EXPECT_EQ(latcher.latched_profile(), StopProfile::MODERATE);
  EXPECT_FLOAT_EQ(latcher.output()->points.front().longitudinal_velocity_mps, 3.0F);
}

}  // namespace autoware::in_lane_mrm_planner
