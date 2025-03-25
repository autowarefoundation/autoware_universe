// Copyright 2024 Tier IV, Inc.
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

#include "node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::motion_velocity_planner::MotionVelocityPlannerNode;
using autoware::planning_test_manager::PlanningInterfaceTestManager;

std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();

  // set subscriber with topic name: motion_velocity_planner → test_node_
  test_manager->subscribeOutput<autoware_planning_msgs::msg::Trajectory>(
    "motion_velocity_planner_node/output/trajectory");

  return test_manager;
}

std::shared_ptr<MotionVelocityPlannerNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};

  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto motion_velocity_planner_dir =
    ament_index_cpp::get_package_share_directory("autoware_motion_velocity_planner_node_universe");
  const auto velocity_smoother_dir =
    ament_index_cpp::get_package_share_directory("autoware_velocity_smoother");

  const auto get_motion_velocity_module_config = [](const std::string & module) {
    const auto package_name = "autoware_motion_velocity_" + module + "_module";
    const auto package_path = ament_index_cpp::get_package_share_directory(package_name);
    return package_path + "/config/" + module + ".param.yaml";
  };

  std::vector<std::string> module_names;
  module_names.emplace_back("autoware::motion_velocity_planner::OutOfLaneModule");
  module_names.emplace_back("autoware::motion_velocity_planner::ObstacleVelocityLimiterModule");
  module_names.emplace_back("autoware::motion_velocity_planner::DynamicObstacleStopModule");

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("launch_modules", module_names);
  params.emplace_back("is_simulation", false);
  node_options.parameter_overrides(params);

  autoware::test_utils::updateNodeOptions(
    node_options, {autoware_test_utils_dir + "/config/test_common.param.yaml",
                   autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
                   autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
                   velocity_smoother_dir + "/config/default_velocity_smoother.param.yaml",
                   velocity_smoother_dir + "/config/Analytical.param.yaml",
                   motion_velocity_planner_dir + "/config/motion_velocity_planner.param.yaml",
                   get_motion_velocity_module_config("out_of_lane"),
                   get_motion_velocity_module_config("obstacle_velocity_limiter"),
                   get_motion_velocity_module_config("dynamic_obstacle_stop")});

  return std::make_shared<MotionVelocityPlannerNode>(node_options);
}

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<MotionVelocityPlannerNode> test_target_node)
{
  // publish necessary topics from test_manager
  test_manager->publishInput(
    test_target_node, "/tf", autoware::test_utils::makeTFMsg(test_target_node, "base_link", "map"));
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner_node/input/accel",
    geometry_msgs::msg::AccelWithCovarianceStamped{});
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner_node/input/dynamic_objects",
    autoware_perception_msgs::msg::PredictedObjects{});
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner_node/input/no_ground_pointcloud",
    sensor_msgs::msg::PointCloud2{}.set__header(
      std_msgs::msg::Header{}.set__frame_id("base_link")));
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner_node/input/vehicle_odometry",
    autoware::test_utils::makeOdometry());
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner_node/input/vector_map",
    autoware::test_utils::makeMapBinMsg());
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner_node/input/traffic_signals",
    autoware_perception_msgs::msg::TrafficLightGroupArray{});
  test_manager->publishInput(
    test_target_node, "motion_velocity_planner_node/input/occupancy_grid",
    autoware::test_utils::makeCostMapMsg());
}

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_trajectory_topic = "motion_velocity_planner_node/input/trajectory";

  // test with nominal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithNormalTrajectory(test_target_node, input_trajectory_topic));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with empty trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithAbnormalTrajectory(test_target_node, input_trajectory_topic));
  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, NodeTestWithOffTrackEgoPose)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();
  publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_trajectory_topic = "motion_velocity_planner_node/input/trajectory";
  const std::string input_odometry_topic = "motion_velocity_planner_node/input/vehicle_odometry";

  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithNormalTrajectory(test_target_node, input_trajectory_topic));

  // make sure motion_velocity_planner is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW_WITH_ERROR_MSG(
    test_manager->testWithOffTrackOdometry(test_target_node, input_odometry_topic));

  rclcpp::shutdown();
}
