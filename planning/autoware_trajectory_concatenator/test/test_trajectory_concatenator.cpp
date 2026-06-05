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

#include "autoware/trajectory_concatenator/detail/trajectory_concatenator.hpp"

#include <builtin_interfaces/msg/time.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/generator_info.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <gtest/gtest.h>

#include <vector>

using autoware::trajectory_concatenator::TrajectoryConcatenator;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_internal_planning_msgs::msg::GeneratorInfo;
using builtin_interfaces::msg::Time;
using unique_identifier_msgs::msg::UUID;

namespace
{

UUID make_uuid(uint8_t first_byte)
{
  UUID uuid{};
  uuid.uuid[0] = first_byte;
  return uuid;
}

Time make_time(double seconds)
{
  Time t;
  t.sec = static_cast<int32_t>(seconds);
  t.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(t.sec)) * 1e9);
  return t;
}

CandidateTrajectories make_msg(UUID generator_id, std::vector<double> stamp_secs)
{
  std::vector<CandidateTrajectory> trajs;
  for (double s : stamp_secs) {
    CandidateTrajectory t;
    t.generator_id = generator_id;
    t.header.stamp = make_time(s);
    trajs.push_back(t);
  }

  GeneratorInfo info;
  info.generator_id = generator_id;

  CandidateTrajectories msg;
  msg.candidate_trajectories = trajs;
  msg.generator_info = {info};
  return msg;
}

concatenator::Params make_params(double duration_time)
{
  concatenator::Params p;
  p.duration_time = duration_time;
  return p;
}

}  // namespace

class TrajectoryConcatenatorTest : public ::testing::Test
{
protected:
  // 1-second staleness window for most tests
  TrajectoryConcatenator concatenator_{make_params(1.0)};
};

// ---------------------------------------------------------------------------
// Empty buffer
// ---------------------------------------------------------------------------

TEST_F(TrajectoryConcatenatorTest, EmptyBufferReturnsEmpty)
{
  auto result = concatenator_.get_concatenated(make_time(10.0));
  EXPECT_TRUE(result.candidate_trajectories.empty());
  EXPECT_TRUE(result.generator_info.empty());
}

// ---------------------------------------------------------------------------
// Single generator — fresh / stale / mixed
// ---------------------------------------------------------------------------

TEST_F(TrajectoryConcatenatorTest, FreshTrajectoryIsReturned)
{
  concatenator_.add_candidate(make_msg(make_uuid(1), {9.5}));  // age 0.5s < 1.0s window

  auto result = concatenator_.get_concatenated(make_time(10.0));
  EXPECT_EQ(result.candidate_trajectories.size(), 1u);
  EXPECT_EQ(result.generator_info.size(), 1u);
}

TEST_F(TrajectoryConcatenatorTest, StaleTrajectoryIsExcluded)
{
  concatenator_.add_candidate(make_msg(make_uuid(1), {5.0}));  // age 5.0s > 1.0s window

  auto result = concatenator_.get_concatenated(make_time(10.0));
  EXPECT_TRUE(result.candidate_trajectories.empty());
  EXPECT_TRUE(result.generator_info.empty());
}

TEST_F(TrajectoryConcatenatorTest, MixedFreshAndStale_OnlyFreshSurvives)
{
  // Two trajectories for same generator: one fresh (9.5s), one stale (5.0s)
  concatenator_.add_candidate(make_msg(make_uuid(1), {9.5, 5.0}));

  auto result = concatenator_.get_concatenated(make_time(10.0));
  ASSERT_EQ(result.candidate_trajectories.size(), 1u);
  EXPECT_EQ(result.candidate_trajectories[0].header.stamp.sec, 9);  // 9.5 -> sec=9
}

TEST_F(TrajectoryConcatenatorTest, AllTrajectoriesStale_GeneratorRemovedFromResult)
{
  concatenator_.add_candidate(make_msg(make_uuid(1), {4.0, 5.0}));  // both stale

  auto result = concatenator_.get_concatenated(make_time(10.0));
  EXPECT_TRUE(result.candidate_trajectories.empty());
  EXPECT_TRUE(result.generator_info.empty());
}

// ---------------------------------------------------------------------------
// Two generators
// ---------------------------------------------------------------------------

TEST_F(TrajectoryConcatenatorTest, TwoGeneratorsBothFresh)
{
  concatenator_.add_candidate(make_msg(make_uuid(1), {9.5}));
  concatenator_.add_candidate(make_msg(make_uuid(2), {9.8}));

  auto result = concatenator_.get_concatenated(make_time(10.0));
  EXPECT_EQ(result.candidate_trajectories.size(), 2u);
  EXPECT_EQ(result.generator_info.size(), 2u);
}

TEST_F(TrajectoryConcatenatorTest, TwoGeneratorsOneFreshOneStale)
{
  concatenator_.add_candidate(make_msg(make_uuid(1), {9.5}));  // fresh
  concatenator_.add_candidate(make_msg(make_uuid(2), {5.0}));  // stale

  auto result = concatenator_.get_concatenated(make_time(10.0));
  EXPECT_EQ(result.candidate_trajectories.size(), 1u);
  EXPECT_EQ(result.generator_info.size(), 1u);
}

TEST_F(TrajectoryConcatenatorTest, TwoGeneratorsBothStale)
{
  concatenator_.add_candidate(make_msg(make_uuid(1), {4.0}));
  concatenator_.add_candidate(make_msg(make_uuid(2), {5.0}));

  auto result = concatenator_.get_concatenated(make_time(10.0));
  EXPECT_TRUE(result.candidate_trajectories.empty());
  EXPECT_TRUE(result.generator_info.empty());
}

// ---------------------------------------------------------------------------
// State across calls
// ---------------------------------------------------------------------------

TEST_F(TrajectoryConcatenatorTest, AddCandidateTwice_OverwritesBufferForSameGenerator)
{
  auto uuid = make_uuid(1);

  concatenator_.add_candidate(make_msg(uuid, {9.0}));
  // Second call replaces the first for the same generator
  concatenator_.add_candidate(make_msg(uuid, {9.8}));

  auto result = concatenator_.get_concatenated(make_time(10.0));
  ASSERT_EQ(result.candidate_trajectories.size(), 1u);
  // Stamp 9.8 -> sec=9, nanosec≈800000000
  EXPECT_EQ(result.candidate_trajectories[0].header.stamp.sec, 9);
  EXPECT_GT(result.candidate_trajectories[0].header.stamp.nanosec, 700000000u);
}

TEST_F(TrajectoryConcatenatorTest, PrunedGeneratorAbsentOnSubsequentCall)
{
  concatenator_.add_candidate(make_msg(make_uuid(1), {5.0}));  // stale at t=10

  // First call prunes the entry
  [[maybe_unused]] auto pruned = concatenator_.get_concatenated(make_time(10.0));

  // Second call: buffer must still be empty (entry was removed, not just hidden)
  auto result = concatenator_.get_concatenated(make_time(10.5));
  EXPECT_TRUE(result.candidate_trajectories.empty());
}

TEST_F(TrajectoryConcatenatorTest, AlternatingFreshStaleFresh_StateUpdatesCorrectly)
{
  auto uuid = make_uuid(1);

  // Round 1: fresh at t=9.5, get at t=10.0 → 1 trajectory
  concatenator_.add_candidate(make_msg(uuid, {9.5}));
  auto r1 = concatenator_.get_concatenated(make_time(10.0));
  EXPECT_EQ(r1.candidate_trajectories.size(), 1u);

  // Round 2: same old stamp → stale at t=20.0 → 0 trajectories
  auto r2 = concatenator_.get_concatenated(make_time(20.0));
  EXPECT_TRUE(r2.candidate_trajectories.empty());

  // Round 3: add fresh, get at t=21.0 → 1 trajectory
  concatenator_.add_candidate(make_msg(uuid, {20.5}));
  auto r3 = concatenator_.get_concatenated(make_time(21.0));
  EXPECT_EQ(r3.candidate_trajectories.size(), 1u);
}

TEST_F(TrajectoryConcatenatorTest, TwoGenerators_OneBecomesStale_OtherSurvivesAcrossCalls)
{
  concatenator_.add_candidate(make_msg(make_uuid(1), {9.5}));  // stays fresh
  concatenator_.add_candidate(make_msg(make_uuid(2), {9.5}));  // also fresh initially

  auto r1 = concatenator_.get_concatenated(make_time(10.0));
  EXPECT_EQ(r1.candidate_trajectories.size(), 2u);

  // Advance time: uuid(2) was never refreshed, so it goes stale; uuid(1) gets a fresh update
  concatenator_.add_candidate(make_msg(make_uuid(1), {10.5}));

  auto r2 = concatenator_.get_concatenated(make_time(11.0));
  EXPECT_EQ(r2.candidate_trajectories.size(), 1u);
  EXPECT_EQ(r2.generator_info.size(), 1u);
}

// ---------------------------------------------------------------------------
// update_parameters
// ---------------------------------------------------------------------------

TEST_F(TrajectoryConcatenatorTest, UpdateParameters_ShorterWindow_MakesPreviouslyFreshTrajStale)
{
  concatenator_.add_candidate(make_msg(make_uuid(1), {9.5}));  // age 0.5s

  // Tighten window to 0.1s — trajectory is now stale
  concatenator_.update_parameters(make_params(0.1));

  auto result = concatenator_.get_concatenated(make_time(10.0));
  EXPECT_TRUE(result.candidate_trajectories.empty());
}

TEST_F(TrajectoryConcatenatorTest, UpdateParameters_LongerWindow_MakesPreviouslyStaleStillStale)
{
  // Age 5.0s, window 1.0s → stale
  concatenator_.add_candidate(make_msg(make_uuid(1), {5.0}));

  // Expanding window to 10.0s makes the trajectory fresh again
  concatenator_.update_parameters(make_params(10.0));

  auto result = concatenator_.get_concatenated(make_time(10.0));
  EXPECT_EQ(result.candidate_trajectories.size(), 1u);
}

// ---------------------------------------------------------------------------
// Edge cases
// ---------------------------------------------------------------------------

TEST_F(TrajectoryConcatenatorTest, EmptyGeneratorInfoProducesNoBufferEntry)
{
  CandidateTrajectories msg;
  msg.candidate_trajectories = {};
  msg.generator_info = {};  // no generators declared
  concatenator_.add_candidate(msg);

  auto result = concatenator_.get_concatenated(make_time(10.0));
  EXPECT_TRUE(result.candidate_trajectories.empty());
}

TEST_F(TrajectoryConcatenatorTest, TrajectoryWithMismatchedGeneratorIdIsIgnored)
{
  // generator_info declares uuid(1), but the trajectory carries uuid(2)
  CandidateTrajectory traj;
  traj.generator_id = make_uuid(2);
  traj.header.stamp = make_time(9.5);

  GeneratorInfo info;
  info.generator_id = make_uuid(1);  // different UUID

  CandidateTrajectories msg;
  msg.candidate_trajectories = {traj};
  msg.generator_info = {info};
  concatenator_.add_candidate(msg);

  // uuid(1) entry has no trajectories → pruned immediately
  auto result = concatenator_.get_concatenated(make_time(10.0));
  EXPECT_TRUE(result.candidate_trajectories.empty());
}

TEST_F(TrajectoryConcatenatorTest, ExactlyAtBoundary_TrajectoryIsStillFresh)
{
  // Staleness condition is strictly greater-than duration_time, so age == duration_time survives.
  concatenator_.add_candidate(make_msg(make_uuid(1), {9.0}));

  auto result = concatenator_.get_concatenated(make_time(10.0));  // diff = exactly 1.0
  EXPECT_EQ(result.candidate_trajectories.size(), 1u);
}
