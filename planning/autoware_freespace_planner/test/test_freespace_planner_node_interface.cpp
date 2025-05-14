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

#include "autoware/freespace_planner/freespace_planner_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::freespace_planner::FreespacePlannerNode;
using autoware::planning_test_manager::PlanningInterfaceTestManager;

std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();
  test_manager->subscribeOutput<autoware_planning_msgs::msg::Trajectory>(
    "freespace_planner/output/trajectory");
  return test_manager;
}

std::shared_ptr<FreespacePlannerNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto freespace_planner_dir =
    ament_index_cpp::get_package_share_directory("autoware_freespace_planner");
  node_options.arguments(
    {"--ros-args", "--params-file",
     autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml", "--params-file",
     freespace_planner_dir + "/config/freespace_planner.param.yaml"});
  return std::make_shared<FreespacePlannerNode>(node_options);
}

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  rclcpp::Node::SharedPtr test_target_node)
{
  // publish necessary topics from test_manager
  test_manager->publishInput(
    test_target_node, "/tf", autoware::test_utils::makeTFMsg(test_target_node, "base_link", "map"));
  test_manager->publishInput(
    test_target_node, "freespace_planner/input/odometry", autoware::test_utils::makeOdometry());
  test_manager->publishInput(
    test_target_node, "freespace_planner/input/occupancy_grid",
    autoware::test_utils::makeCostMapMsg());
  test_manager->publishInput(
    test_target_node, "freespace_planner/input/scenario",
    autoware::test_utils::makeScenarioMsg(autoware_internal_planning_msgs::msg::Scenario::PARKING));
}

// the following tests are disable because they randomly fail
TEST(PlanningModuleInterfaceTest, DISABLED_testPlanningInterfaceWithVariousTrajectoryInput)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();
  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_route_topic = "freespace_planner/input/route";

  // test with normal route
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithBehaviorNormalRoute(test_target_node, input_route_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with empty route
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithAbnormalRoute(test_target_node, input_route_topic));
  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, DISABLED_NodeTestWithOffTrackEgoPose)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_route_topic = "freespace_planner/input/route";
  const std::string input_odometry_topic = "freespace_planner/input/odometry";

  // test for normal route
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithBehaviorNormalRoute(test_target_node, input_route_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithOffTrackOdometry(test_target_node, input_odometry_topic));

  rclcpp::shutdown();
}
