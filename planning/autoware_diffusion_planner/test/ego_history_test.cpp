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

#include "autoware/diffusion_planner/preprocessing/ego_history.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"

#include <Eigen/Core>
#include <rclcpp/time.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <memory>
#include <vector>

namespace autoware::diffusion_planner::test
{
using nav_msgs::msg::Odometry;
using preprocess::EgoHistory;

namespace
{
// ROS-time stamp at t seconds. Message header stamps are read back as RCL_ROS_TIME, so all test
// times must use the same source or rclcpp::Time subtraction throws on a clock-source mismatch.
rclcpp::Time stamp(const double t)
{
  return rclcpp::Time(static_cast<int64_t>(std::llround(t * 1e9)), RCL_ROS_TIME);
}

// Odometry at time t seconds, positioned at x with identity heading.
std::shared_ptr<const Odometry> make_odom(const double t, const double x)
{
  auto odom = std::make_shared<Odometry>();
  odom->header.stamp = stamp(t);
  odom->pose.pose.position.x = x;
  odom->pose.pose.orientation.w = 1.0;
  return odom;
}
}  // namespace

class EgoHistoryTest : public ::testing::Test
{
protected:
  void SetUp() override {}
};

// --- Buffer management (the ordering invariant that consumers rely on) -------------------------

TEST_F(EgoHistoryTest, EmptyOnConstruction)
{
  EgoHistory history;
  EXPECT_TRUE(history.empty());
}

TEST_F(EgoHistoryTest, NullSamplesAreSkipped)
{
  EgoHistory history;
  history.update({nullptr, nullptr});
  EXPECT_TRUE(history.empty());
}

TEST_F(EgoHistoryTest, MonotonicAppendKeepsNewest)
{
  EgoHistory history;
  history.update({make_odom(0.0, 0.0), make_odom(0.1, 1.0), make_odom(0.2, 2.0)});
  EXPECT_FALSE(history.empty());
  EXPECT_NEAR(history.newest_stamp().seconds(), 0.2, 1e-6);
}

// A large backwards jump (bag loop / sim reset) must purge the stale future timeline.
TEST_F(EgoHistoryTest, BackwardsJumpPurgesStaleTimeline)
{
  EgoHistory history;
  history.update({make_odom(100.0, 100.0), make_odom(100.1, 101.0), make_odom(100.2, 102.0)});
  ASSERT_NEAR(history.newest_stamp().seconds(), 100.2, 1e-6);

  // Clock jumps back to ~0; every buffered sample is newer, so all are dropped.
  history.update({make_odom(0.0, 0.0), make_odom(0.1, 1.0), make_odom(0.2, 2.0)});
  EXPECT_NEAR(history.newest_stamp().seconds(), 0.2, 1e-6);

  // Sampling on the new timeline must not see the stale 100.x poses.
  const auto [state, diff] = history.select_state(stamp(0.1), true);
  EXPECT_NEAR(state.pose.pose.position.x, 1.0, 1e-3);
}

// A single out-of-order stamp trims only the offending tail, preserving older history.
TEST_F(EgoHistoryTest, OutOfOrderStampTrimsOnlyTail)
{
  EgoHistory history;
  history.update(
    {make_odom(0.0, 0.0), make_odom(0.1, 1.0), make_odom(0.2, 2.0), make_odom(0.3, 3.0)});

  // 0.15 drops 0.3 and 0.2 (both >= 0.15) but keeps 0.0 and 0.1.
  history.update({make_odom(0.15, 99.0)});
  EXPECT_NEAR(history.newest_stamp().seconds(), 0.15, 1e-6);

  // Older 0.0/0.1 survive: interpolating at 0.1 still yields x ~= 1.0, not the 99 sample.
  const auto [state, diff] = history.select_state(stamp(0.1), true);
  EXPECT_NEAR(state.pose.pose.position.x, 1.0, 1e-3);
}

// An identical stamp replaces the previous sample rather than creating a zero-dt duplicate.
TEST_F(EgoHistoryTest, DuplicateStampReplaces)
{
  EgoHistory history;
  history.update({make_odom(0.0, 0.0), make_odom(0.1, 1.0), make_odom(0.2, 2.0)});
  history.update({make_odom(0.2, 99.0)});

  const auto [state, diff] = history.select_state(stamp(0.2), false);
  EXPECT_NEAR(state.pose.pose.position.x, 99.0, 1e-3);
}

// Samples older than the time window are pruned from the front.
TEST_F(EgoHistoryTest, PrunesToTimeWindow)
{
  EgoHistory history(0.25);  // 0.25 s window
  for (int i = 0; i <= 5; ++i) {
    history.update({make_odom(0.1 * i, static_cast<double>(i))});  // 0.0 .. 0.5
  }
  // Newest is 0.5; only stamps >= 0.25 survive -> oldest kept is 0.3 (x = 3).
  const auto [state, diff] = history.select_state(stamp(0.0), false);
  EXPECT_NEAR(state.pose.pose.position.x, 3.0, 1e-3);
  EXPECT_GT(diff, 0.25);  // nearest kept sample is >= 0.3 s from t=0
}

// --- State selection ---------------------------------------------------------------------------

TEST_F(EgoHistoryTest, SelectStateInterpolatesWithinRange)
{
  EgoHistory history;
  history.update({make_odom(0.0, 0.0), make_odom(0.2, 2.0)});
  const auto [state, diff] = history.select_state(stamp(0.1), true);
  EXPECT_NEAR(state.pose.pose.position.x, 1.0, 1e-3);
  EXPECT_NEAR(diff, 0.0, 1e-9);
}

TEST_F(EgoHistoryTest, SelectStateFallsBackToNearestOutsideRange)
{
  EgoHistory history;
  history.update({make_odom(0.0, 0.0), make_odom(0.2, 2.0)});
  const auto [state, diff] = history.select_state(stamp(0.5), true);
  EXPECT_NEAR(state.pose.pose.position.x, 2.0, 1e-3);  // clamped to newest
  EXPECT_NEAR(diff, 0.3, 1e-3);
}

// --- Past-trajectory export --------------------------------------------------------------------

// Migrated from preprocessing_utils_edge_case_test: resampling with exact time alignment.
TEST_F(EgoHistoryTest, CreateEgoAgentPastTimeInterpolation)
{
  std::deque<Odometry> odom_msgs;
  for (int i = 0; i < 3; ++i) {
    Odometry odom;
    odom.header.stamp.sec = 0;
    odom.header.stamp.nanosec = static_cast<uint32_t>(i) * 100000000u;  // 0.0, 0.1, 0.2
    odom.pose.pose.position.x = static_cast<double>(i);                 // 0, 1, 2
    odom.pose.pose.orientation.w = 1.0;
    odom_msgs.push_back(odom);
  }

  const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
  const rclcpp::Time ref_time(0, 200000000u);  // 0.2s
  const size_t num_timesteps = 3;

  const auto result =
    EgoHistory::create_ego_agent_past(odom_msgs, num_timesteps, identity, ref_time);

  ASSERT_EQ(result.size(), num_timesteps * 4);
  for (size_t t = 0; t < num_timesteps; ++t) {
    const size_t base = t * 4;
    EXPECT_NEAR(result[base + EGO_AGENT_PAST_IDX_X], static_cast<float>(t), 1e-3f)
      << "timestep " << t;
    EXPECT_NEAR(result[base + EGO_AGENT_PAST_IDX_Y], 0.0f, 1e-3f);
  }
}

// to_agent_past applies the base_link-to-center shift when requested.
TEST_F(EgoHistoryTest, ToAgentPastAppliesShift)
{
  EgoHistory history;
  history.update({make_odom(0.0, 0.0)});
  const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
  const rclcpp::Time ref_time(0, 0);

  const auto without_shift = history.to_agent_past(identity, ref_time, true, false, 1.0);
  const auto with_shift = history.to_agent_past(identity, ref_time, true, true, 1.0);

  ASSERT_EQ(without_shift.size(), static_cast<size_t>(EGO_HISTORY_SHAPE[1]) * 4);
  const size_t last = static_cast<size_t>(EGO_HISTORY_SHAPE[1] - 1) * 4;
  EXPECT_NEAR(without_shift[last + EGO_AGENT_PAST_IDX_X], 0.0f, 1e-3f);
  EXPECT_NEAR(with_shift[last + EGO_AGENT_PAST_IDX_X], 1.0f, 1e-3f);  // shifted +1 along +x heading
}

}  // namespace autoware::diffusion_planner::test
