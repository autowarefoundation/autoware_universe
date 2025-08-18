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

#include "autoware/default_scenario_selector/node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware::scenario_selector
{
using autoware::planning_test_manager::PlanningInterfaceTestManager;

// 讓 plugin 在整個測試期間存活
static std::shared_ptr<DefaultScenarioSelector> g_plugin;

std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();
  // 監看 plugin/node 會發布的 topic
  test_manager->subscribeOutput<autoware_internal_planning_msgs::msg::Scenario>("output/scenario");
  return test_manager;
}

// 產生一個乾淨的 node，並以該 node 初始化 plugin
std::shared_ptr<rclcpp::Node> generateNodeAndInitPlugin()
{
  auto node_options = rclcpp::NodeOptions{};
  node_options.append_parameter_override("update_rate", 10.0);
  node_options.append_parameter_override("th_max_message_delay_sec", INFINITY);
  node_options.append_parameter_override("th_arrived_distance_m", 1.0);
  node_options.append_parameter_override("th_stopped_time_sec", 1.0);
  node_options.append_parameter_override("th_stopped_velocity_mps", 0.01);
  node_options.append_parameter_override("enable_mode_switching", true);

  // 名稱仍用 default_scenario_selector，與原先一致，方便 YAML/命名空間
  auto node = std::make_shared<rclcpp::Node>("default_scenario_selector", node_options);

  // 建立並初始化 plugin（用此 node 的 context 建立 pub/sub/timer/params）
  g_plugin = std::make_shared<DefaultScenarioSelector>();
  g_plugin->initialize(node.get());

  return node;
}

void publishMandatoryTopics(
  const std::shared_ptr<PlanningInterfaceTestManager> & test_manager,
  const std::shared_ptr<rclcpp::Node> & node)
{
  // 發佈 plugin 所需的最基本輸入
  test_manager->publishInput(node, "input/odometry", autoware::test_utils::makeOdometry());
  test_manager->publishInput(node, "is_parking_completed", std_msgs::msg::Bool{});
  test_manager->publishInput(node, "input/parking/trajectory", autoware_planning_msgs::msg::Trajectory{});
  test_manager->publishInput(node, "input/lanelet_map", autoware::test_utils::makeMapBinMsg());
  test_manager->publishInput(node, "input/route", autoware::test_utils::makeNormalRoute());
  test_manager->publishInput(node, "input/operation_mode_state", autoware_adapi_v1_msgs::msg::OperationModeState{});
}

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectoryLaneDrivingMode)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto node = generateNodeAndInitPlugin();

  publishMandatoryTopics(test_manager, node);

  const std::string input_trajectory_topic = "input/lane_driving/trajectory";

  // 正常軌跡
  ASSERT_NO_THROW(test_manager->testWithNormalTrajectory(node, input_trajectory_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // 空/單點/重疊點等異常軌跡
  ASSERT_NO_THROW(test_manager->testWithAbnormalTrajectory(node, input_trajectory_topic));

  rclcpp::shutdown();
  g_plugin.reset();
}

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectoryParkingMode)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto node = generateNodeAndInitPlugin();

  publishMandatoryTopics(test_manager, node);

  const std::string input_trajectory_topic = "input/parking/trajectory";

  // 正常軌跡
  ASSERT_NO_THROW(test_manager->testWithNormalTrajectory(node, input_trajectory_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // 空/單點/重疊點等異常軌跡
  ASSERT_NO_THROW(test_manager->testWithAbnormalTrajectory(node, input_trajectory_topic));

  rclcpp::shutdown();
  g_plugin.reset();
}

TEST(PlanningModuleInterfaceTest, NodeTestWithOffTrackEgoPose)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto node = generateNodeAndInitPlugin();

  publishMandatoryTopics(test_manager, node);

  const std::string input_trajectory_topic = "input/lane_driving/trajectory";
  const std::string input_odometry_topic = "input/odometry";

  // 正常軌跡
  ASSERT_NO_THROW(test_manager->testWithNormalTrajectory(node, input_trajectory_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // 逸離車道的 odom 測試
  ASSERT_NO_THROW(test_manager->testWithOffTrackOdometry(node, input_odometry_topic));

  rclcpp::shutdown();
  g_plugin.reset();
}

}  // namespace autoware::scenario_selector
