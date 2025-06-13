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

#include <autoware/behavior_velocity_planner/test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_planning_msgs/msg/path.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware::test_utils
{
std::string get_absolute_path_to_test_config(
  const std::string & package_name, const std::string & config_filename)
{
  const auto dir = ament_index_cpp::get_package_share_directory(package_name);
  return dir + "/test_config/" + config_filename;
}

}  // namespace autoware::test_utils

namespace autoware::behavior_velocity_planner
{

template <class T>
T loadMessageInYaml(
  const std::string & yaml_file, std::vector<std::string> corrupted_check_list = {})
{
  const auto yaml_path = autoware::test_utils::get_absolute_path_to_test_config(
    "autoware_behavior_velocity_crosswalk_module", yaml_file);

  YAML::Node node = YAML::LoadFile(yaml_path);
  for (auto & word : corrupted_check_list) {
    if (node[word].IsNull()) {
      throw std::runtime_error(
        "Failed to parse YAML file: " + yaml_path + ". The file might be corrupted.");
    }
  }

  return autoware::test_utils::parse<T>(node);
}

PathWithLaneId loadPathWithLaneIdInYaml()
{
  const auto yaml_path = autoware::test_utils::get_absolute_path_to_test_config(
    "autoware_behavior_velocity_crosswalk_module", "path_with_lane_id_data.yaml");

  if (const auto path = autoware::test_utils::parse<std::optional<PathWithLaneId>>(yaml_path)) {
    return *path;
  }

  throw std::runtime_error(
    "Failed to parse YAML file: " + yaml_path + ". The file might be corrupted.");
}

nav_msgs::msg::Odometry loadOdometryInYaml()
{
  return loadMessageInYaml<nav_msgs::msg::Odometry>("vehicle_odometry_data.yaml", {"pose"});
}

PredictedObjects loadPathObjectsInYaml()
{
  return loadMessageInYaml<PredictedObjects>("dynamic_objects_data.yaml", {"objects"});
}

TEST(PlanningFactorTest, NodeTestWithPredictedObjects)
{
  rclcpp::init(0, nullptr);

  const auto plugin_info_vec = {autoware::behavior_velocity_planner::PluginInfo{
    "crosswalk", "autoware::behavior_velocity_planner::CrosswalkModulePlugin"}};

  auto test_manager = autoware::behavior_velocity_planner::generateTestManager();
  auto test_target_node = autoware::behavior_velocity_planner::generateNode(plugin_info_vec);
  autoware::behavior_velocity_planner::publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_path_with_lane_id_topic =
    "behavior_velocity_planner_node/input/path_with_lane_id";
  const std::string input_odometry_topic = "behavior_velocity_planner_node/input/vehicle_odometry";
  const std::string input_dynamic_objects_topic =
    "behavior_velocity_planner_node/input/dynamic_objects";
  const std::string output_planning_factors_topic = "planning/planning_factors/crosswalk";

  const rclcpp::Node::SharedPtr test_node = test_manager->getTestNode();
  RCLCPP_INFO(rclcpp::get_logger("test_node"), "test");

  autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr planning_factor_msg;
  const auto test_sub =
    test_node->create_subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>(
      output_planning_factors_topic, rclcpp::QoS{1},
      [&planning_factor_msg](
        autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr msg) {
        planning_factor_msg = msg;
        RCLCPP_INFO(
          rclcpp::get_logger("test_node"), "Received PlanningFactorArray with %zu factors",
          msg->factors.size());

        RCLCPP_INFO_STREAM(rclcpp::get_logger("test_node"), "Planning factors:");
        for (const auto & factor : msg->factors) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("test_node"), "  Module: " << factor.module);
          RCLCPP_INFO_STREAM(
            rclcpp::get_logger("test_node"),
            "    Is driving forward: " << factor.is_driving_forward);
          RCLCPP_INFO_STREAM(rclcpp::get_logger("test_node"), "    Behavior: " << factor.behavior);
          RCLCPP_INFO_STREAM(rclcpp::get_logger("test_node"), "    Detail: " << factor.detail);
          RCLCPP_INFO_STREAM(
            rclcpp::get_logger("test_node"),
            "    Safety factors: " << factor.safety_factors.factors.size());
        }
        if (msg->factors.empty()) {
          RCLCPP_WARN(rclcpp::get_logger("test_node"), "No planning factors received.");
        }
      });

  const auto objects = loadPathObjectsInYaml();
  const auto odometry = loadOdometryInYaml();
  const auto path = loadPathWithLaneIdInYaml();
  test_manager->publishInput(test_target_node, input_dynamic_objects_topic, objects, 5);
  test_manager->publishInput(test_target_node, input_odometry_topic, odometry, 5);
  test_manager->publishInput(test_target_node, input_path_with_lane_id_topic, path, 5);

  // spin once
  rclcpp::spin_some(test_target_node);
  rclcpp::spin_some(test_manager->getTestNode());

  // make sure behavior_path_planner is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // NOTE: In this test, the objects variable contains only one pedestrian.
  const auto expected_object_id = objects.objects.front().object_id;
  const auto expected_object_type = autoware_internal_planning_msgs::msg::SafetyFactor::OBJECT;

  // make sure planning_factor_msg is received
  EXPECT_NE(planning_factor_msg, nullptr);

  // make sure planning_factor_msg is not empty
  EXPECT_EQ(planning_factor_msg->factors.size(), 1);

  const auto & factor = planning_factor_msg->factors.front();
  EXPECT_FALSE(factor.safety_factors.is_safe);

  // make sure safety_factors is not empty
  EXPECT_EQ(factor.safety_factors.factors.size(), 1);

  const auto & safety_factor = factor.safety_factors.factors.front();
  EXPECT_EQ(safety_factor.type, expected_object_type);
  EXPECT_FALSE(safety_factor.is_safe);
  EXPECT_EQ(safety_factor.points.size(), 1);
  EXPECT_EQ(safety_factor.object_id, expected_object_id);

  rclcpp::shutdown();
}
}  // namespace autoware::behavior_velocity_planner
