// Copyright 2024 TIER IV, Inc.
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
  const auto yaml_path = autoware::test_utils::get_absolute_path_to_test_config(
    "autoware_behavior_velocity_crosswalk_module", "vehicle_odometry_data.yaml");

  YAML::Node node = YAML::LoadFile(yaml_path);
  if (!node["pose"]) {
    throw std::runtime_error(
      "Failed to parse YAML file: " + yaml_path + ". The file might be corrupted.");
  }

  return autoware::test_utils::parse<nav_msgs::msg::Odometry>(node);
}

PredictedObjects loadPathObjectsInYaml()
{
  const auto yaml_path = autoware::test_utils::get_absolute_path_to_test_config(
    "autoware_behavior_velocity_crosswalk_module", "dynamic_objects_data.yaml");

  YAML::Node node = YAML::LoadFile(yaml_path);
  if (!node["objects"]) {
    throw std::runtime_error(
      "Failed to parse YAML file: " + yaml_path + ". The file might be corrupted.");
  }
  return autoware::test_utils::parse<PredictedObjects>(node);
}

TEST(PlanningModuleInterfaceTest, PlanningFactorTest)
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

  const auto test_sub =
    test_node->create_subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>(
      output_planning_factors_topic, rclcpp::QoS{1},
      [](const autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr msg) {
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

  // const auto path_sub = test_node->create_subscription<autoware_planning_msgs::msg::Path>(
  //   "/behavior_velocity_planner_node/output/path", rclcpp::QoS{1},
  //   [](const autoware_planning_msgs::msg::Path::SharedPtr msg) {
  //     RCLCPP_INFO(
  //       rclcpp::get_logger("test_node"), "Received Path with %zu points", msg->points.size());
  //     auto distance = 0.0;
  //     for (size_t i = 1; i < msg->points.size(); ++i) {
  //       const auto & p1 = msg->points[i - 1];
  //       const auto & p2 = msg->points[i];
  //       distance += std::hypot(
  //         p2.pose.position.x - p1.pose.position.x, p2.pose.position.y - p1.pose.position.y);
  //     }
  //     RCLCPP_INFO(rclcpp::get_logger("test_node"), "Total distance of path: %.2f meters",
  //     distance);

  //     // start point
  //     RCLCPP_INFO(
  //       rclcpp::get_logger("test_node"), "Start point: (%.2f, %.2f)",
  //       msg->points.front().pose.position.x, msg->points.front().pose.position.y);
  //     // end point
  //     RCLCPP_INFO(
  //       rclcpp::get_logger("test_node"), "End point: (%.2f, %.2f)",
  //       msg->points.back().pose.position.x, msg->points.back().pose.position.y);
  //   });

  // std::map<std::string, std::vector<std::string>> topic_list =
  //   test_node->get_topic_names_and_types();

  // for (const auto & topic : topic_list) {
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("test_node"), "Topic: %s, Types: %zu", topic.first.c_str(),
  //     topic.second.size());
  //   for (const auto & type : topic.second) {
  //     RCLCPP_INFO(rclcpp::get_logger("test_node"), "  Type: %s", type.c_str());
  //   }
  // }

  auto objects = loadPathObjectsInYaml();
  auto odometry = loadOdometryInYaml();
  auto path = loadPathWithLaneIdInYaml();

  const size_t retry_count = 5;
  for (size_t i = 0; i < retry_count; ++i) {
    test_manager->publishInput(test_target_node, input_dynamic_objects_topic, objects, 1);
    test_manager->publishInput(test_target_node, input_odometry_topic, odometry, 1);
    test_manager->publishInput(test_target_node, input_path_with_lane_id_topic, path, 2);
  }

  // make sure behavior_path_planner is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  rclcpp::shutdown();
}
}  // namespace autoware::behavior_velocity_planner
