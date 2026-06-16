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

// Tests for TrajectoryConcatenatorWrapper.
// Scope: wrapper-specific responsibilities only — thread safety and node-clock usage.
// Concatenation correctness is covered separately by test_trajectory_concatenator.

#include "autoware/trajectory_concatenator/trajectory_concatenator_wrapper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/generator_info.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <thread>
#include <vector>

using autoware::trajectory_concatenator::TrajectoryConcatenatorWrapper;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_internal_planning_msgs::msg::GeneratorInfo;
using unique_identifier_msgs::msg::UUID;

namespace
{

UUID make_uuid(uint8_t first_byte)
{
  UUID uuid{};
  uuid.uuid[0] = first_byte;
  return uuid;
}

// Build a single-generator message with one trajectory stamped at `stamp`.
CandidateTrajectories make_msg(UUID generator_id, builtin_interfaces::msg::Time stamp)
{
  CandidateTrajectory traj;
  traj.generator_id = generator_id;
  traj.header.stamp = stamp;

  GeneratorInfo info;
  info.generator_id = generator_id;

  CandidateTrajectories msg;
  msg.candidate_trajectories = {traj};
  msg.generator_info = {info};
  return msg;
}

}  // namespace

class TrajectoryConcatenatorWrapperTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_wrapper_node", rclcpp::NodeOptions{});
    wrapper_ = std::make_unique<TrajectoryConcatenatorWrapper>(
      *node_, node_->get_node_parameters_interface());
  }

  void TearDown() override
  {
    wrapper_.reset();
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<TrajectoryConcatenatorWrapper> wrapper_;
};

// ---------------------------------------------------------------------------
// Node-clock usage
// ---------------------------------------------------------------------------

TEST_F(TrajectoryConcatenatorWrapperTest, GetConcatenated_UsesNodeClock_FreshTrajectoryIsReturned)
{
  // Stamp a trajectory at the node's current time.  get_concatenated() must use the same
  // node clock, so the age is ~0 and the trajectory survives the default 0.2-s window.
  auto now = node_->get_clock()->now();
  wrapper_->add_candidate(make_msg(make_uuid(1), now));

  auto result = wrapper_->get_concatenated();
  EXPECT_FALSE(result.candidate_trajectories.empty());
}

TEST_F(TrajectoryConcatenatorWrapperTest, GetConcatenated_UsesNodeClock_OldTrajectoryIsDropped)
{
  // Stamp far in the past — always stale regardless of when we call get_concatenated().
  builtin_interfaces::msg::Time past;
  past.sec = 0;
  past.nanosec = 0;
  wrapper_->add_candidate(make_msg(make_uuid(1), past));

  auto result = wrapper_->get_concatenated();
  EXPECT_TRUE(result.candidate_trajectories.empty());
}

// ---------------------------------------------------------------------------
// Thread safety
// ---------------------------------------------------------------------------

TEST_F(TrajectoryConcatenatorWrapperTest, ConcurrentAddCandidate_DoesNotDeadlock)
{
  constexpr int num_threads = 4;
  constexpr int iterations = 50;

  std::vector<std::thread> threads;
  threads.reserve(num_threads);
  for (int i = 0; i < num_threads; ++i) {
    threads.emplace_back([&, i]() {
      auto uuid = make_uuid(static_cast<uint8_t>(i + 1));
      for (int j = 0; j < iterations; ++j) {
        auto stamp = node_->get_clock()->now();
        wrapper_->add_candidate(make_msg(uuid, stamp));
      }
    });
  }
  for (auto & t : threads) {
    t.join();
  }
  SUCCEED();  // reaching here means no deadlock
}

TEST_F(TrajectoryConcatenatorWrapperTest, ConcurrentAddAndGet_DoesNotDeadlock)
{
  constexpr int iterations = 50;
  auto uuid = make_uuid(1);

  std::thread adder([&]() {
    for (int i = 0; i < iterations; ++i) {
      auto stamp = node_->get_clock()->now();
      wrapper_->add_candidate(make_msg(uuid, stamp));
    }
  });

  std::thread getter([&]() {
    for (int i = 0; i < iterations; ++i) {
      [[maybe_unused]] auto r = wrapper_->get_concatenated();
    }
  });

  adder.join();
  getter.join();
  SUCCEED();
}

// ---------------------------------------------------------------------------
// add_candidate → get_concatenated round-trip (smoke test for delegation)
// ---------------------------------------------------------------------------

TEST_F(TrajectoryConcatenatorWrapperTest, AddThenGet_ReturnsSameGeneratorCount)
{
  auto now = node_->get_clock()->now();
  wrapper_->add_candidate(make_msg(make_uuid(1), now));
  wrapper_->add_candidate(make_msg(make_uuid(2), now));

  auto result = wrapper_->get_concatenated();
  EXPECT_EQ(result.generator_info.size(), 2u);
}

// ---------------------------------------------------------------------------

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
