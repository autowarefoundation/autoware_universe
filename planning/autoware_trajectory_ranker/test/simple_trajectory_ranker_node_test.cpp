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

#include "autoware/trajectory_ranker/simple_trajectory_ranker_node.hpp"

#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectories.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace
{
// Helper function to create a CandidateTrajectory message
autoware_internal_planning_msgs::msg::CandidateTrajectory create_candidate_trajectory(
  const unique_identifier_msgs::msg::UUID & generator_id)
{
  autoware_internal_planning_msgs::msg::CandidateTrajectory trajectory;
  trajectory.generator_id = generator_id;
  return trajectory;
}

// Helper to map string IDs to consistent UUIDs
std::map<std::string, unique_identifier_msgs::msg::UUID> uuid_map;
unique_identifier_msgs::msg::UUID get_uuid_from_string_id(const std::string & id_str)
{
  if (uuid_map.count(id_str) == 0) {
    uuid_map[id_str] = autoware_utils_uuid::generate_uuid();
  }
  return uuid_map[id_str];
}

// Test fixture for SimpleTrajectoryRanker
class SimpleTrajectoryRankerTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }

  static autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories::SharedPtr
  publish_and_receive(
    const std::shared_ptr<autoware::trajectory_ranker::SimpleTrajectoryRanker> & node,
    const autoware_internal_planning_msgs::msg::CandidateTrajectories & input_msg)
  {
    auto pub = node->create_publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>(
      "~/input/candidate_trajectories", 1);
    autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories::SharedPtr output_msg;
    auto sub =
      node->create_subscription<autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories>(
        "~/output/scored_trajectories", 1,
        [&output_msg](
          const autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories::SharedPtr msg) {
          output_msg = msg;
        });

    pub->publish(input_msg);

    rclcpp::WallRate rate(10);
    int count = 0;
    while (!output_msg && count < 100) {  // Spin for a limited time to avoid infinite loop
      rclcpp::spin_some(node);
      rate.sleep();
      count++;
    }
    return output_msg;
  }
};
}  // namespace

TEST_F(SimpleTrajectoryRankerTest, BasicRankingTest)
{
  // Node options with ranked_generator_name_prefixes parameter {"a", "b"}
  rclcpp::NodeOptions options;
  std::vector<std::string> ranked_generator_name_prefixes = {"a", "b"};
  options.append_parameter_override(
    "ranked_generator_name_prefixes", ranked_generator_name_prefixes);

  // Instantiate the node
  auto node = std::make_shared<autoware::trajectory_ranker::SimpleTrajectoryRanker>(options);

  // Publish a message with trajectories from "c", "b", "a"
  autoware_internal_planning_msgs::msg::CandidateTrajectories input_msg;
  input_msg.candidate_trajectories.push_back(
    create_candidate_trajectory(get_uuid_from_string_id("c")));
  input_msg.candidate_trajectories.push_back(
    create_candidate_trajectory(get_uuid_from_string_id("b")));
  input_msg.candidate_trajectories.push_back(
    create_candidate_trajectory(get_uuid_from_string_id("a")));

  // Populate generator_info
  for (const auto & name : {"a", "b", "c"}) {
    autoware_internal_planning_msgs::msg::GeneratorInfo info;
    info.generator_id = get_uuid_from_string_id(name);
    info.generator_name.data = name;
    input_msg.generator_info.push_back(info);
  }

  // Receive the scored trajectories
  auto output_msg = publish_and_receive(node, input_msg);

  // Check the received message
  ASSERT_TRUE(output_msg);
  ASSERT_EQ(output_msg->scored_candidate_trajectories.size(), 3UL);

  // Expected order: "a", "b", "c"
  EXPECT_EQ(
    autoware_utils_uuid::to_hex_string(
      output_msg->scored_candidate_trajectories[0].candidate_trajectory.generator_id),
    autoware_utils_uuid::to_hex_string(get_uuid_from_string_id("a")));
  EXPECT_DOUBLE_EQ(output_msg->scored_candidate_trajectories[0].score, 2.0);
  EXPECT_EQ(
    autoware_utils_uuid::to_hex_string(
      output_msg->scored_candidate_trajectories[1].candidate_trajectory.generator_id),
    autoware_utils_uuid::to_hex_string(get_uuid_from_string_id("b")));
  EXPECT_DOUBLE_EQ(output_msg->scored_candidate_trajectories[1].score, 1.0);
  EXPECT_EQ(
    autoware_utils_uuid::to_hex_string(
      output_msg->scored_candidate_trajectories[2].candidate_trajectory.generator_id),
    autoware_utils_uuid::to_hex_string(get_uuid_from_string_id("c")));
  EXPECT_DOUBLE_EQ(output_msg->scored_candidate_trajectories[2].score, 0.0);
}

TEST_F(SimpleTrajectoryRankerTest, PrefixRankingTest)
{
  // Node options with ranked_generator_name_prefixes parameter {"PrefixA", "PrefixB"}
  rclcpp::NodeOptions options;
  std::vector<std::string> ranked_generator_name_prefixes = {"PrefixA", "PrefixB"};
  options.append_parameter_override(
    "ranked_generator_name_prefixes", ranked_generator_name_prefixes);

  // Instantiate the node
  auto node = std::make_shared<autoware::trajectory_ranker::SimpleTrajectoryRanker>(options);

  // Publish a message with trajectories from "Unranked", "PrefixB_1", "PrefixA_1"
  autoware_internal_planning_msgs::msg::CandidateTrajectories input_msg;
  input_msg.candidate_trajectories.push_back(
    create_candidate_trajectory(get_uuid_from_string_id("Unranked")));
  input_msg.candidate_trajectories.push_back(
    create_candidate_trajectory(get_uuid_from_string_id("PrefixB_1")));
  input_msg.candidate_trajectories.push_back(
    create_candidate_trajectory(get_uuid_from_string_id("PrefixA_1")));

  // Populate generator_info
  for (const auto & name : {"PrefixA_1", "PrefixB_1", "Unranked"}) {
    autoware_internal_planning_msgs::msg::GeneratorInfo info;
    info.generator_id = get_uuid_from_string_id(name);
    info.generator_name.data = name;
    input_msg.generator_info.push_back(info);
  }

  // Receive the scored trajectories
  auto output_msg = publish_and_receive(node, input_msg);

  // Check the received message
  ASSERT_TRUE(output_msg);
  ASSERT_EQ(output_msg->scored_candidate_trajectories.size(), 3UL);

  // Expected order: "PrefixA_1", "PrefixB_1", "Unranked"
  EXPECT_EQ(
    autoware_utils_uuid::to_hex_string(
      output_msg->scored_candidate_trajectories[0].candidate_trajectory.generator_id),
    autoware_utils_uuid::to_hex_string(get_uuid_from_string_id("PrefixA_1")));
  EXPECT_DOUBLE_EQ(output_msg->scored_candidate_trajectories[0].score, 2.0);
  EXPECT_EQ(
    autoware_utils_uuid::to_hex_string(
      output_msg->scored_candidate_trajectories[1].candidate_trajectory.generator_id),
    autoware_utils_uuid::to_hex_string(get_uuid_from_string_id("PrefixB_1")));
  EXPECT_DOUBLE_EQ(output_msg->scored_candidate_trajectories[1].score, 1.0);
  EXPECT_EQ(
    autoware_utils_uuid::to_hex_string(
      output_msg->scored_candidate_trajectories[2].candidate_trajectory.generator_id),
    autoware_utils_uuid::to_hex_string(get_uuid_from_string_id("Unranked")));
  EXPECT_DOUBLE_EQ(output_msg->scored_candidate_trajectories[2].score, 0.0);
}

TEST_F(SimpleTrajectoryRankerTest, EmptyInputTest)
{
  // Node options with ranked_generator_name_prefixes parameter {"a", "b"}
  rclcpp::NodeOptions options;
  std::vector<std::string> ranked_generator_name_prefixes = {"a", "b"};
  options.append_parameter_override(
    "ranked_generator_name_prefixes", ranked_generator_name_prefixes);

  // Instantiate the node
  auto node = std::make_shared<autoware::trajectory_ranker::SimpleTrajectoryRanker>(options);

  // Publish an empty message
  autoware_internal_planning_msgs::msg::CandidateTrajectories input_msg;
  auto output_msg = publish_and_receive(node, input_msg);

  // Check the received message
  ASSERT_TRUE(output_msg);
  ASSERT_EQ(output_msg->scored_candidate_trajectories.size(), 0UL);
}

TEST_F(SimpleTrajectoryRankerTest, NoRankedGeneratorsTest)
{
  // Node options with empty ranked_generator_name_prefixes parameter
  rclcpp::NodeOptions options;
  options.append_parameter_override("ranked_generator_name_prefixes", std::vector<std::string>{});

  // Instantiate the node
  auto node = std::make_shared<autoware::trajectory_ranker::SimpleTrajectoryRanker>(options);

  // Publish a message with trajectories from "c", "b", "a"
  autoware_internal_planning_msgs::msg::CandidateTrajectories input_msg;
  input_msg.candidate_trajectories.push_back(
    create_candidate_trajectory(get_uuid_from_string_id("c")));
  input_msg.candidate_trajectories.push_back(
    create_candidate_trajectory(get_uuid_from_string_id("b")));
  input_msg.candidate_trajectories.push_back(
    create_candidate_trajectory(get_uuid_from_string_id("a")));

  // Populate generator_info
  for (const auto & name : {"a", "b", "c"}) {
    autoware_internal_planning_msgs::msg::GeneratorInfo info;
    info.generator_id = get_uuid_from_string_id(name);
    info.generator_name.data = name;
    input_msg.generator_info.push_back(info);
  }

  auto output_msg = publish_and_receive(node, input_msg);

  // Check the received message
  ASSERT_TRUE(output_msg);
  ASSERT_EQ(output_msg->scored_candidate_trajectories.size(), 3UL);

  // Expected order: "c", "b", "a" (same as input)
  EXPECT_EQ(
    autoware_utils_uuid::to_hex_string(
      output_msg->scored_candidate_trajectories[0].candidate_trajectory.generator_id),
    autoware_utils_uuid::to_hex_string(get_uuid_from_string_id("c")));
  EXPECT_EQ(
    autoware_utils_uuid::to_hex_string(
      output_msg->scored_candidate_trajectories[1].candidate_trajectory.generator_id),
    autoware_utils_uuid::to_hex_string(get_uuid_from_string_id("b")));
  EXPECT_EQ(
    autoware_utils_uuid::to_hex_string(
      output_msg->scored_candidate_trajectories[2].candidate_trajectory.generator_id),
    autoware_utils_uuid::to_hex_string(get_uuid_from_string_id("a")));
}

TEST_F(SimpleTrajectoryRankerTest, AllUnrankedGeneratorsTest)
{
  // Node options with ranked_generator_name_prefixes parameter {"x", "y"}
  rclcpp::NodeOptions options;
  std::vector<std::string> ranked_generator_name_prefixes = {"x", "y"};
  options.append_parameter_override(
    "ranked_generator_name_prefixes", ranked_generator_name_prefixes);

  // Instantiate the node
  auto node = std::make_shared<autoware::trajectory_ranker::SimpleTrajectoryRanker>(options);

  // Publish a message with trajectories from "c", "b", "a" (all unranked)
  autoware_internal_planning_msgs::msg::CandidateTrajectories input_msg;
  input_msg.candidate_trajectories.push_back(
    create_candidate_trajectory(get_uuid_from_string_id("c")));
  input_msg.candidate_trajectories.push_back(
    create_candidate_trajectory(get_uuid_from_string_id("b")));
  input_msg.candidate_trajectories.push_back(
    create_candidate_trajectory(get_uuid_from_string_id("a")));

  // Populate generator_info
  for (const auto & name : {"a", "b", "c"}) {
    autoware_internal_planning_msgs::msg::GeneratorInfo info;
    info.generator_id = get_uuid_from_string_id(name);
    info.generator_name.data = name;
    input_msg.generator_info.push_back(info);
  }

  auto output_msg = publish_and_receive(node, input_msg);

  // Check the received message
  ASSERT_TRUE(output_msg);
  ASSERT_EQ(output_msg->scored_candidate_trajectories.size(), 3UL);

  // Expected order: "c", "b", "a" (same as input)
  EXPECT_EQ(
    autoware_utils_uuid::to_hex_string(
      output_msg->scored_candidate_trajectories[0].candidate_trajectory.generator_id),
    autoware_utils_uuid::to_hex_string(get_uuid_from_string_id("c")));
  EXPECT_EQ(
    autoware_utils_uuid::to_hex_string(
      output_msg->scored_candidate_trajectories[1].candidate_trajectory.generator_id),
    autoware_utils_uuid::to_hex_string(get_uuid_from_string_id("b")));
  EXPECT_EQ(
    autoware_utils_uuid::to_hex_string(
      output_msg->scored_candidate_trajectories[2].candidate_trajectory.generator_id),
    autoware_utils_uuid::to_hex_string(get_uuid_from_string_id("a")));
}
