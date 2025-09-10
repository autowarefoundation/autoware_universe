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

#include <autoware/behavior_path_goal_planner_module/lane_change.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <tuple>
#include <variant>

namespace autoware
{

using Started = behavior_path_planner::LaneChangeContext::Started;
using Executing = behavior_path_planner::LaneChangeContext::Executing;
using Aborted = behavior_path_planner::LaneChangeContext::Aborted;
using Completed = behavior_path_planner::LaneChangeContext::Completed;
using NotLaneChanging = behavior_path_planner::LaneChangeContext::NotLaneChanging;
using State = behavior_path_planner::LaneChangeContext::State;

class TestLaneChangeContextTransition : public ::testing::Test
{
protected:
  static constexpr lanelet::Id target_lane_id = 1;

  std::shared_ptr<rclcpp::Node> ros_node_;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    ros_node_ = rclcpp::Node::make_shared("test");
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::tuple<Started, Executing, Completed, Aborted, NotLaneChanging> generate_latest_states() const
  {
    const auto now = ros_node_->get_clock()->now();

    const auto start = Started{target_lane_id, now};
    const auto executing = Executing{start};
    const auto completed = Completed{executing};
    const auto aborted = Aborted{};
    const auto not_lane_changing = NotLaneChanging{};
    return {start, executing, completed, aborted, not_lane_changing};
  }

  Started generate_latest_start_state() const
  {
    auto ros_node = rclcpp::Node::make_shared("test");
    const auto now = ros_node->get_clock()->now();

    return Started{target_lane_id, now};
  }

  Executing generate_latest_executing_state() const
  {
    return Executing{generate_latest_start_state()};
  }

  Completed generate_latest_completed_state() const
  {
    return Completed{generate_latest_executing_state()};
  }
};

/**
 * transition is stale if
 * if current state changed to `Started` or `Aborted`
 * if current state changed to `Executing` from
 * - `Aborted` or `NotLaneChanging`
 * - `Started` or `Executing` with different complete_lane id or different start_time
 * if current_state changed to `NotLaneChanging` from except for `NotLaneChanging`
 */
TEST_F(TestLaneChangeContextTransition, started_or_aborted_is_not_consistent_anyway)
{
  const auto [started, executing, completed, aborted, not_lane_changing] = generate_latest_states();

  {
    const auto now_started = generate_latest_start_state();
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(started, now_started),
      true);
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(
        executing, now_started),
      true);
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(
        completed, now_started),
      true);
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(aborted, now_started),
      true);
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(
        not_lane_changing, now_started),
      true);
  }
  {
    const auto now_aborted = Aborted{};
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(started, now_aborted),
      true);
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(
        executing, now_aborted),
      true);
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(
        completed, now_aborted),
      true);
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(aborted, now_aborted),
      true);
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(
        not_lane_changing, now_aborted),
      true);
  }
}

TEST_F(TestLaneChangeContextTransition, test_consistency_for_executing)
{
  const auto [started, executing, completed, aborted, not_lane_changing] = generate_latest_states();
  {
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(
        not_lane_changing, executing),
      true);
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(aborted, executing),
      true);
  }
  {
    // started/executing(same time) -> executing(same time) is consistent
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(started, executing),
      false);
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(executing, executing),
      false);

    // but started/executing(past time) -> executing is not consistent
    const auto latest_executing = generate_latest_executing_state();
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(
        started, latest_executing),
      true);
    EXPECT_EQ(
      behavior_path_planner::LaneChangeContext::is_not_consistent_transition(
        executing, latest_executing),
      true);
  }
  EXPECT_EQ(
    behavior_path_planner::LaneChangeContext::is_not_consistent_transition(
      not_lane_changing, executing),
    true);
}

TEST_F(TestLaneChangeContextTransition, test_consistency_for_not_lane_changing)
{
  const auto [started, executing, completed, aborted, not_lane_changing] = generate_latest_states();
  EXPECT_EQ(
    behavior_path_planner::LaneChangeContext::is_not_consistent_transition(
      started, not_lane_changing),
    true);
  EXPECT_EQ(
    behavior_path_planner::LaneChangeContext::is_not_consistent_transition(
      executing, not_lane_changing),
    true);
  EXPECT_EQ(
    behavior_path_planner::LaneChangeContext::is_not_consistent_transition(
      aborted, not_lane_changing),
    true);

  // this is consistent
  EXPECT_EQ(
    behavior_path_planner::LaneChangeContext::is_not_consistent_transition(
      not_lane_changing, not_lane_changing),
    false);
}

}  // namespace autoware
