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

#include "autoware/extra_scenario_selector/node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::scenario_selector
{

using autoware::planning_test_manager::PlanningInterfaceTestManager;

static std::pair<rclcpp::Node::SharedPtr, std::unique_ptr<ExtraScenarioSelector>>
generateHostNodeAndPlugin()
{
  auto node_options = rclcpp::NodeOptions{};
  node_options.append_parameter_override("update_rate", 10.0);
  node_options.append_parameter_override("th_max_message_delay_sec", INFINITY);
  node_options.append_parameter_override("th_arrived_distance_m", 1.0);
  node_options.append_parameter_override("th_stopped_time_sec", 1.0);
  node_options.append_parameter_override("th_stopped_velocity_mps", 0.01);
  node_options.append_parameter_override("enable_mode_switching", true);

  auto host = std::make_shared<rclcpp::Node>("extra_scenario_selector_test_host", node_options);

  auto plugin = std::make_unique<ExtraScenarioSelector>();
  plugin->initialize(host.get());

  return {host, std::move(plugin)};
}

static std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();
  test_manager->subscribeOutput<autoware_internal_planning_msgs::msg::Scenario>("output/scenario");
  return test_manager;
}

static void publishMandatoryTopics(
  const std::shared_ptr<PlanningInterfaceTestManager> & test_manager,
  const rclcpp::Node::SharedPtr & host_node)
{
  test_manager->publishInput(
    host_node, "input/odometry", autoware::test_utils::makeOdometry());

  test_manager->publishInput(host_node, "is_parking_completed", std_msgs::msg::Bool{});

  test_manager->publishInput(
    host_node, "input/parking/trajectory", autoware_planning_msgs::msg::Trajectory{});

  test_manager->publishInput(
    host_node, "input/lanelet_map", autoware::test_utils::makeMapBinMsg());

  test_manager->publishInput(
    host_node, "input/route", autoware::test_utils::makeNormalRoute());

  test_manager->publishInput(
    host_node, "input/operation_mode_state",
    autoware_adapi_v1_msgs::msg::OperationModeState{});
}

TEST(PlanningModuleInterfaceTest, PluginTestWithExceptionTrajectoryLaneDrivingMode)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto [host_node, plugin] = generateHostNodeAndPlugin();

  publishMandatoryTopics(test_manager, host_node);

  const std::string input_trajectory_topic = "input/lane_driving/trajectory";

  ASSERT_NO_THROW(test_manager->testWithNormalTrajectory(host_node, input_trajectory_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW(test_manager->testWithAbnormalTrajectory(host_node, input_trajectory_topic));

  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, PluginTestWithExceptionTrajectoryParkingMode)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto [host_node, plugin] = generateHostNodeAndPlugin();

  publishMandatoryTopics(test_manager, host_node);

  const std::string input_trajectory_topic = "input/parking/trajectory";

  ASSERT_NO_THROW(test_manager->testWithNormalTrajectory(host_node, input_trajectory_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW(test_manager->testWithAbnormalTrajectory(host_node, input_trajectory_topic));

  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, PluginTestWithOffTrackEgoPose)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto [host_node, plugin] = generateHostNodeAndPlugin();

  publishMandatoryTopics(test_manager, host_node);

  const std::string input_trajectory_topic = "input/lane_driving/trajectory";
  const std::string input_odometry_topic = "input/odometry";

  ASSERT_NO_THROW(test_manager->testWithNormalTrajectory(host_node, input_trajectory_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW(test_manager->testWithOffTrackOdometry(host_node, input_odometry_topic));

  rclcpp::shutdown();
}

}  // namespace autoware::scenario_selector
