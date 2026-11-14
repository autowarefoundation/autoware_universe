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

#include "autoware/candidate_bank/candidate/trajectory_candidate_manager.hpp"
#include "autoware/candidate_bank/exact_pair_queue.hpp"

#include <rclcpp/duration.hpp>

#include <gtest/gtest.h>

#include <memory>
using autoware::candidate_bank::CandidateBankConfig;
using autoware::candidate_bank::CandidateSource;
using autoware::candidate_bank::ExactPairQueue;
using autoware::candidate_bank::TrajectoryCandidateManager;
using Trajectory = autoware_planning_msgs::msg::Trajectory;
TEST(CandidateBankConfig, DefaultsToPolygonGeometry)
{
  CandidateBankConfig cfg;
  EXPECT_EQ(cfg.collision_geometry_mode, "polygon");
}
Trajectory make_trajectory(double y, const rclcpp::Time & t)
{
  Trajectory tr;
  tr.header.frame_id = "map";
  tr.header.stamp = t;
  for (int i = 0; i <= 20; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x = i;
    p.pose.position.y = y;
    p.pose.orientation.w = 1;
    p.longitudinal_velocity_mps = 2;
    p.time_from_start = rclcpp::Duration::from_seconds(i * 0.5);
    tr.points.push_back(p);
  }
  return tr;
}
TEST(CandidateBank, RejectStaticCollisionAndRetainAlternative)
{
  CandidateBankConfig cfg;
  cfg.mode = "select";
  cfg.static_safety_enabled = true;
  cfg.static_hard_reject_enabled = true;
  cfg.collision_geometry_mode = "polygon";
  cfg.max_projection_error_m = 10;
  TrajectoryCandidateManager bank(cfg);
  rclcpp::Time t(10, 0, RCL_ROS_TIME);
  autoware_perception_msgs::msg::PredictedObjects predictions;
  predictions.header.frame_id = "map";
  auto tracked = std::make_shared<autoware_perception_msgs::msg::TrackedObjects>();
  tracked->header.frame_id = "map";
  autoware_perception_msgs::msg::TrackedObject object;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 2;
  object.shape.dimensions.y = 2;
  object.shape.dimensions.z = 2;
  object.kinematics.pose_with_covariance.pose.position.x = 8;
  object.kinematics.pose_with_covariance.pose.orientation.w = 1;
  tracked->objects.push_back(object);
  bank.add(make_trajectory(0, t), CandidateSource::LEGACY_OUTPUT, predictions, 0, std::nullopt, t);
  bank.add(make_trajectory(5, t), CandidateSource::LEGACY_OUTPUT, predictions, 0, std::nullopt, t);
  nav_msgs::msg::Odometry ego;
  ego.pose.pose.orientation.w = 1;
  bank.evaluate(ego, tracked, predictions, std::nullopt, {}, {}, t);
  ASSERT_EQ(bank.candidates().size(), 2u);
  EXPECT_FALSE(bank.candidates()[0].safe);
  EXPECT_TRUE(bank.candidates()[1].safe);
  auto chosen = bank.select(t);
  ASSERT_TRUE(chosen.has_value());
  EXPECT_NEAR(chosen->points.front().pose.position.y, 5, 1e-6);
}
TEST(CandidateBank, GenerationAgeAndReset)
{
  CandidateBankConfig cfg;
  cfg.mode = "observe";
  cfg.max_generation_age_sec = 1;
  TrajectoryCandidateManager bank(cfg);
  rclcpp::Time t(10, 0, RCL_ROS_TIME);
  autoware_perception_msgs::msg::PredictedObjects predictions;
  bank.add(make_trajectory(0, t), CandidateSource::LEGACY_OUTPUT, predictions, 0, std::nullopt, t);
  nav_msgs::msg::Odometry ego;
  ego.pose.pose.orientation.w = 1;
  bank.purge(ego, t + rclcpp::Duration::from_seconds(2));
  EXPECT_TRUE(bank.candidates().empty());
  bank.add(make_trajectory(0, t), CandidateSource::LEGACY_OUTPUT, predictions, 0, std::nullopt, t);
  bank.clear();
  EXPECT_TRUE(bank.candidates().empty());
  EXPECT_FALSE(bank.incumbent().has_value());
}
TEST(PairQueue, ExactAndOnce)
{
  ExactPairQueue<int, int> q(2);
  q.first(10, 1);
  q.second(11, 2);
  EXPECT_FALSE(q.pop());
  q.second(10, 3);
  auto pair = q.pop();
  ASSERT_TRUE(pair);
  EXPECT_EQ(pair->second, 3);
  q.first(10, 1);
  q.second(10, 3);
  EXPECT_FALSE(q.pop());
  q.clear();
  q.first(1, 4);
  q.second(1, 5);
  EXPECT_TRUE(q.pop());
}

TEST(CandidateBank, TemporalAlignmentUsesEvaluationTimeBeforeSpatialProjection)
{
  CandidateBankConfig cfg;
  cfg.mode = "observe";
  cfg.max_generation_age_sec = 10;
  cfg.max_projection_error_m = 100;
  cfg.min_remaining_horizon_sec = 0;
  cfg.temporal_alignment_enabled = true;
  cfg.temporal_alignment_search_window_sec = 0.01;
  TrajectoryCandidateManager bank(cfg);
  rclcpp::Time t(10, 0, RCL_ROS_TIME);
  Trajectory tr;
  tr.header.frame_id = "map";
  tr.header.stamp = t;
  const double xs[] = {0.0, 1.0, 2.0, 1.0, 0.0, 1.0};
  for (int i = 0; i < 6; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint p;
    p.pose.position.x = xs[i];
    p.pose.orientation.w = 1;
    p.longitudinal_velocity_mps = 1;
    p.time_from_start = rclcpp::Duration::from_seconds(i);
    tr.points.push_back(p);
  }
  autoware_perception_msgs::msg::PredictedObjects predictions;
  bank.add(tr, CandidateSource::LEGACY_OUTPUT, predictions, 0, std::nullopt, t);
  nav_msgs::msg::Odometry ego;
  ego.header.frame_id = "map";
  ego.pose.pose.position.x = 0;
  ego.pose.pose.orientation.w = 1;
  bank.purge(ego, t + rclcpp::Duration::from_seconds(4.0));
  ASSERT_EQ(bank.candidates().size(), 1u);
  EXPECT_EQ(bank.candidates().front().live_start_index, 4u);
}
