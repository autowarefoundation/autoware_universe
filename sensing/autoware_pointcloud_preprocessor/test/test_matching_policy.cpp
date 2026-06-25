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

#include "autoware/pointcloud_preprocessor/concatenate_data/matching_policy.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace
{
using autoware::pointcloud_preprocessor::AdvancedMatchingPolicy;
using autoware::pointcloud_preprocessor::CandidateCollectorState;
using autoware::pointcloud_preprocessor::IncomingCloudInfo;
using autoware::pointcloud_preprocessor::NaiveMatchingPolicy;

IncomingCloudInfo incoming_cloud_info(const std::string & topic, double timestamp, double arrival)
{
  return IncomingCloudInfo{topic, timestamp, arrival};
}
}  // namespace

TEST(NaiveMatchingPolicy, ReferenceIsArrivalTime)
{
  NaiveMatchingPolicy strategy;
  const auto reference = strategy.reference_for(incoming_cloud_info("lidar_top", 10.0, 12.5));
  EXPECT_DOUBLE_EQ(reference.reference_time, 12.5);
  EXPECT_DOUBLE_EQ(reference.noise_window, 0.0);
}

TEST(NaiveMatchingPolicy, MatchesClosestArrivalTime)
{
  NaiveMatchingPolicy strategy;
  std::vector<CandidateCollectorState> collectors = {
    {10.0, 0.0, false},
    {10.5, 0.0, false},
  };
  // Arrival 10.45 is closer to the second collector (|0.05| < |0.45|).
  const auto matched = strategy.match(collectors, incoming_cloud_info("lidar_left", 0.0, 10.45));
  ASSERT_TRUE(matched.has_value());
  EXPECT_EQ(*matched, 1u);
}

TEST(NaiveMatchingPolicy, SkipsCollectorsThatAlreadyHaveTheTopic)
{
  NaiveMatchingPolicy strategy;
  std::vector<CandidateCollectorState> collectors = {
    {10.0, 0.0, false},
    {10.5, 0.0, true},  // closest by time, but already holds the topic
  };
  const auto matched = strategy.match(collectors, incoming_cloud_info("lidar_left", 0.0, 10.45));
  ASSERT_TRUE(matched.has_value());
  EXPECT_EQ(*matched, 0u);
}

TEST(NaiveMatchingPolicy, NoCollectorsReturnsNullopt)
{
  NaiveMatchingPolicy strategy;
  EXPECT_FALSE(strategy.match({}, incoming_cloud_info("lidar_left", 0.0, 10.0)).has_value());
}

TEST(AdvancedMatchingPolicy, ConstructorValidatesSizes)
{
  const std::vector<std::string> topics = {"a", "b"};
  EXPECT_THROW(AdvancedMatchingPolicy(topics, {0.0}, {0.0, 0.0}), std::runtime_error);
  EXPECT_THROW(AdvancedMatchingPolicy(topics, {0.0, 0.0}, {0.0}), std::runtime_error);
  EXPECT_NO_THROW(AdvancedMatchingPolicy(topics, {0.0, 0.04}, {0.01, 0.01}));
}

TEST(AdvancedMatchingPolicy, ReferenceSubtractsOffset)
{
  AdvancedMatchingPolicy strategy(
    {"lidar_top", "lidar_left", "lidar_right"}, {0.0, 0.04, 0.08}, {0.01, 0.01, 0.02});
  const auto reference = strategy.reference_for(incoming_cloud_info("lidar_left", 10.04, 0.0));
  EXPECT_DOUBLE_EQ(reference.reference_time, 10.0);  // 10.04 - 0.04
  EXPECT_DOUBLE_EQ(reference.noise_window, 0.01);
}

TEST(AdvancedMatchingPolicy, MatchesInsideWindowAndRejectsOutside)
{
  AdvancedMatchingPolicy strategy(
    {"lidar_top", "lidar_left", "lidar_right"}, {0.0, 0.04, 0.08}, {0.01, 0.01, 0.01});
  std::vector<CandidateCollectorState> collectors = {{10.0, 0.01, false}};

  // lidar_left @ 10.04 -> corrected 10.00, inside [9.99, 10.01] (+- topic noise).
  EXPECT_EQ(strategy.match(collectors, incoming_cloud_info("lidar_left", 10.04, 0.0)).value(), 0u);
  // lidar_left @ 10.20 -> corrected 10.16, far outside the window.
  EXPECT_FALSE(
    strategy.match(collectors, incoming_cloud_info("lidar_left", 10.20, 0.0)).has_value());
}

TEST(AdvancedMatchingPolicy, DoesNotSkipCollectorsThatHaveTheTopic)
{
  // The advanced strategy matches purely on the timestamp window (unlike naive, it does not look at
  // whether the collector already holds the topic).
  AdvancedMatchingPolicy strategy({"lidar_top"}, {0.0}, {0.01});
  std::vector<CandidateCollectorState> collectors = {{10.0, 0.01, true}};
  EXPECT_EQ(strategy.match(collectors, incoming_cloud_info("lidar_top", 10.0, 0.0)).value(), 0u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
