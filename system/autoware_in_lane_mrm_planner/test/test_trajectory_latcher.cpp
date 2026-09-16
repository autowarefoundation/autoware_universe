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

#include "predicted_objects_latcher.hpp"
#include "trajectory_latcher.hpp"
#include "trajectory_selector_stub.hpp"
#include "trigger_edge_detector.hpp"

#include <gtest/gtest.h>

namespace autoware::in_lane_mrm_planner
{
namespace
{

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

TEST(TrajectoryLatcherTest, OutputFollowsLatestCandidateBeforeLatch)
{
  TrajectoryLatcher latcher;
  const auto traj_a = make_trajectory(5.0F);
  const auto traj_b = make_trajectory(3.0F);

  latcher.update_candidate(traj_a);
  ASSERT_TRUE(latcher.output().has_value());
  EXPECT_FLOAT_EQ(latcher.output()->points.front().longitudinal_velocity_mps, 5.0F);

  latcher.update_candidate(traj_b);
  ASSERT_TRUE(latcher.output().has_value());
  EXPECT_FLOAT_EQ(latcher.output()->points.front().longitudinal_velocity_mps, 3.0F);
}

TEST(TrajectoryLatcherTest, LatchFreezesOutputDespiteNewCandidates)
{
  TrajectoryLatcher latcher;
  latcher.update_candidate(make_trajectory(4.0F));
  latcher.latch();

  ASSERT_TRUE(latcher.is_latched());
  ASSERT_TRUE(latcher.output().has_value());
  EXPECT_FLOAT_EQ(latcher.output()->points.front().longitudinal_velocity_mps, 4.0F);

  latcher.update_candidate(make_trajectory(1.0F));
  ASSERT_TRUE(latcher.output().has_value());
  EXPECT_FLOAT_EQ(latcher.output()->points.front().longitudinal_velocity_mps, 4.0F);
}

TEST(TrajectoryLatcherTest, UnlatchResumesLatestCandidate)
{
  TrajectoryLatcher latcher;
  latcher.update_candidate(make_trajectory(4.0F));
  latcher.latch();
  latcher.update_candidate(make_trajectory(1.0F));
  latcher.unlatch();

  EXPECT_FALSE(latcher.is_latched());
  ASSERT_TRUE(latcher.output().has_value());
  EXPECT_FLOAT_EQ(latcher.output()->points.front().longitudinal_velocity_mps, 1.0F);
}

TEST(TrajectoryLatcherTest, IgnoresEmptyCandidate)
{
  TrajectoryLatcher latcher;
  latcher.update_candidate(make_trajectory(2.0F));

  Trajectory empty;
  latcher.update_candidate(empty);
  ASSERT_TRUE(latcher.output().has_value());
  EXPECT_FLOAT_EQ(latcher.output()->points.front().longitudinal_velocity_mps, 2.0F);
}

TEST(TrajectorySelectorStubTest, PassesThroughTrajectory)
{
  const TrajectorySelectorStub selector;
  const auto input = make_trajectory(6.0F);
  const auto output = selector.select(input);
  EXPECT_FLOAT_EQ(output.points.front().longitudinal_velocity_mps, 6.0F);
}

TEST(TriggerEdgeDetectorTest, DetectsRisingAndFallingEdges)
{
  TriggerEdgeDetector detector;

  auto edges = detector.update(false);
  EXPECT_FALSE(edges.rising);
  EXPECT_FALSE(edges.falling);

  edges = detector.update(true);
  EXPECT_TRUE(edges.rising);
  EXPECT_FALSE(edges.falling);

  edges = detector.update(true);
  EXPECT_FALSE(edges.rising);
  EXPECT_FALSE(edges.falling);

  edges = detector.update(false);
  EXPECT_FALSE(edges.rising);
  EXPECT_TRUE(edges.falling);
}

TEST(PredictedObjectsLatcherTest, LatchOnRisingAndReleaseOnFalling)
{
  PredictedObjectsLatcher latcher;
  PredictedObjects live;
  live.objects.resize(1);
  live.objects.front().object_id.uuid = {1};

  PredictedObjects updated;
  updated.objects.resize(1);
  updated.objects.front().object_id.uuid = {2};

  latcher.latch(live, true);
  EXPECT_TRUE(latcher.is_latched());
  EXPECT_EQ(latcher.objects_for_planning(updated).objects.front().object_id.uuid.at(0), 1);

  latcher.unlatch();
  EXPECT_FALSE(latcher.is_latched());
  EXPECT_EQ(latcher.objects_for_planning(updated).objects.front().object_id.uuid.at(0), 2);
}

TEST(PredictedObjectsLatcherTest, SkipsLatchWhenDisabled)
{
  PredictedObjectsLatcher latcher;
  PredictedObjects live;
  live.objects.resize(1);

  latcher.latch(live, false);
  EXPECT_FALSE(latcher.is_latched());
  EXPECT_EQ(&latcher.objects_for_planning(live), &live);
}

}  // namespace autoware::in_lane_mrm_planner
