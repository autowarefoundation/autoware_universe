// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_planner/behavior_path_planner_node.hpp"
#include "autoware/behavior_path_planner/test_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

using autoware::behavior_path_planner::generateNode;
using autoware::behavior_path_planner::generateTestManager;
using autoware::behavior_path_planner::publishMandatoryTopics;

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionRoute)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode({}, {});

  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_route_topic = "behavior_path_planner/input/route";

  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithBehaviorNormalRoute(test_target_node, input_route_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with empty route
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithAbnormalRoute(test_target_node, input_route_topic));
  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, NodeTestWithOffTrackEgoPose)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto test_target_node = generateNode({}, {});
  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_route_topic = "behavior_path_planner/input/route";
  const std::string input_odometry_topic = "behavior_path_planner/input/odometry";

  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithBehaviorNormalRoute(test_target_node, input_route_topic));

  // make sure behavior_path_planner is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithOffTrackOdometry(test_target_node, input_odometry_topic));

  rclcpp::shutdown();
}
