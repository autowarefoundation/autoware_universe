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
std::string get_absolute_path_to_test_data(
  const std::string & package_name, const std::string & config_filename)
{
  const auto dir = ament_index_cpp::get_package_share_directory(package_name);
  return dir + "/test_data/" + config_filename;
}

// TODO(odashima): move this functions to autoware_test_utils
template <>
nav_msgs::msg::MapMetaData parse(const YAML::Node & node)
{
  nav_msgs::msg::MapMetaData msg;
  msg.map_load_time = parse<builtin_interfaces::msg::Time>(node["map_load_time"]);
  msg.resolution = node["resolution"].as<float>();
  msg.width = node["width"].as<uint32_t>();
  msg.height = node["height"].as<uint32_t>();
  msg.origin = parse<geometry_msgs::msg::Pose>(node["origin"]);
  return msg;
}

// TODO(odashima): move this functions to autoware_test_utils
template <>
nav_msgs::msg::OccupancyGrid parse(const YAML::Node & node)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header = parse<std_msgs::msg::Header>(node["header"]);
  msg.info = parse<nav_msgs::msg::MapMetaData>(node["info"]);
  msg.data = node["data"].as<std::vector<int8_t>>();
  return msg;
}

}  // namespace autoware::test_utils

namespace autoware::behavior_velocity_planner
{

template <class T>
T loadMessageInYaml(
  const std::string & yaml_file, std::vector<std::string> corrupted_check_list = {})
{
  const auto yaml_path = autoware::test_utils::get_absolute_path_to_test_data(
    "autoware_behavior_velocity_intersection_module", yaml_file);

  YAML::Node node = YAML::LoadFile(yaml_path);
  for (auto & word : corrupted_check_list) {
    if (node[word].IsNull()) {
      throw std::runtime_error(
        "Failed to parse YAML file: " + yaml_path + ". The file might be corrupted.");
    }
  }

  return autoware::test_utils::parse<T>(node);
}

PathWithLaneId loadPathWithLaneIdInYaml(
  const std::string & yaml_file = "path_with_lane_id_data.yaml")
{
  const auto yaml_path = autoware::test_utils::get_absolute_path_to_test_data(
    "autoware_behavior_velocity_intersection_module", yaml_file);

  if (const auto path = autoware::test_utils::parse<std::optional<PathWithLaneId>>(yaml_path)) {
    return *path;
  }

  throw std::runtime_error(
    "Failed to parse YAML file: " + yaml_path + ". The file might be corrupted.");
}

nav_msgs::msg::Odometry loadOdometryInYaml(
  const std::string & yaml_file = "vehicle_odometry_data.yaml")
{
  return loadMessageInYaml<nav_msgs::msg::Odometry>(yaml_file, {"pose"});
}

PredictedObjects loadPathObjectsInYaml(const std::string & yaml_file = "dynamic_objects_data.yaml")
{
  return loadMessageInYaml<PredictedObjects>(yaml_file, {"objects"});
}

autoware_perception_msgs::msg::TrafficLightGroupArray loadTrafficLightGroupArrayInYaml(
  const std::string & yaml_file = "traffic_light_group_array_data.yaml")
{
  return loadMessageInYaml<autoware_perception_msgs::msg::TrafficLightGroupArray>(
    yaml_file, {"traffic_light_groups"});
}

// load occupancy grid from yaml file
nav_msgs::msg::OccupancyGrid loadOccupancyGridInYaml(
  const std::string & yaml_file = "occupancy_grid_data.yaml")
{
  return loadMessageInYaml<nav_msgs::msg::OccupancyGrid>(yaml_file, {"data", "info"});
}

void publishMandatoryTopicsForOcclusion(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<BehaviorVelocityPlannerNode> test_target_node)
{
  // publish necessary topics from test_manager
  test_manager->publishInput(
    test_target_node, "/tf", autoware::test_utils::makeTFMsg(test_target_node, "base_link", "map"));
  test_manager->publishInput(
    test_target_node, "behavior_velocity_planner_node/input/accel",
    geometry_msgs::msg::AccelWithCovarianceStamped{});
  test_manager->publishInput(
    test_target_node, "behavior_velocity_planner_node/input/dynamic_objects",
    autoware_perception_msgs::msg::PredictedObjects{});
  test_manager->publishInput(
    test_target_node, "behavior_velocity_planner_node/input/no_ground_pointcloud",
    sensor_msgs::msg::PointCloud2{}.set__header(
      std_msgs::msg::Header{}.set__frame_id("base_link")));
  test_manager->publishInput(
    test_target_node, "behavior_velocity_planner_node/input/vehicle_odometry",
    autoware::test_utils::makeOdometry());
  test_manager->publishInput(
    test_target_node, "behavior_velocity_planner_node/input/accel",
    geometry_msgs::msg::AccelWithCovarianceStamped{});
  test_manager->publishInput(
    test_target_node, "behavior_velocity_planner_node/input/vector_map",
    autoware::test_utils::makeMapBinMsg("autoware_test_utils", "intersection/lanelet2_map.osm"));
  test_manager->publishInput(
    test_target_node, "behavior_velocity_planner_node/input/traffic_signals",
    autoware_perception_msgs::msg::TrafficLightGroupArray{});
  test_manager->publishInput(
    test_target_node, "behavior_velocity_planner_node/input/external_velocity_limit_mps",
    autoware_internal_planning_msgs::msg::VelocityLimit{});
}

TEST(PlanningFactorTest, NodeTestWithPredictedObjects)
{
  rclcpp::init(0, nullptr);

  const auto plugin_info_vec = {autoware::behavior_velocity_planner::PluginInfo{
    "intersection", "autoware::behavior_velocity_planner::IntersectionModulePlugin"}};

  auto test_manager = autoware::behavior_velocity_planner::generateTestManager();
  auto test_target_node = autoware::behavior_velocity_planner::generateNode(plugin_info_vec);
  autoware::behavior_velocity_planner::publishMandatoryTopics(test_manager, test_target_node);

  const std::string input_path_with_lane_id_topic =
    "behavior_velocity_planner_node/input/path_with_lane_id";
  const std::string input_odometry_topic = "behavior_velocity_planner_node/input/vehicle_odometry";
  const std::string input_dynamic_objects_topic =
    "behavior_velocity_planner_node/input/dynamic_objects";
  const std::string output_planning_factors_topic = "planning/planning_factors/intersection";

  const rclcpp::Node::SharedPtr test_node = test_manager->getTestNode();

  autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr planning_factor_msg;
  const auto test_sub =
    test_node->create_subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>(
      output_planning_factors_topic, rclcpp::QoS{1},
      [&planning_factor_msg](
        autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr msg) {
        planning_factor_msg = msg;
      });

  const auto objects = loadPathObjectsInYaml();
  const auto odometry = loadOdometryInYaml();
  const auto path = loadPathWithLaneIdInYaml();

  test_manager->publishInput(test_target_node, input_dynamic_objects_topic, objects);
  test_manager->publishInput(test_target_node, input_odometry_topic, odometry);
  test_manager->publishInput(test_target_node, input_path_with_lane_id_topic, path);

  // make sure behavior_path_planner is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // make sure planning_factor_msg is received
  EXPECT_NE(planning_factor_msg, nullptr);

  // make sure planning_factor_msg is not empty
  EXPECT_EQ(planning_factor_msg->factors.size(), 1);

  const auto & factor = planning_factor_msg->factors.front();
  EXPECT_FALSE(factor.safety_factors.is_safe);

  // make sure control_points is not empty
  EXPECT_EQ(factor.control_points.size(), 1);
  EXPECT_LT(factor.control_points.front().distance, 30.0);

  // make sure safety_factors is not empty
  EXPECT_EQ(factor.safety_factors.factors.size(), 1);

  const auto & safety_factor = factor.safety_factors.factors.front();
  const auto expected_object_type = autoware_internal_planning_msgs::msg::SafetyFactor::OBJECT;
  EXPECT_EQ(safety_factor.type, expected_object_type);
  EXPECT_FALSE(safety_factor.is_safe);
  EXPECT_EQ(safety_factor.points.size(), 1);
  const bool is_object_id_included = std::any_of(
    objects.objects.begin(), objects.objects.end(),
    [&](const auto & object) { return object.object_id == safety_factor.object_id; });
  EXPECT_TRUE(is_object_id_included);

  rclcpp::shutdown();
}

TEST(PlanningFactorTest, NodeTestWithPredictedObjectsForOcclusion)
{
  rclcpp::init(0, nullptr);

  const auto plugin_info_vec = {autoware::behavior_velocity_planner::PluginInfo{
    "intersection", "autoware::behavior_velocity_planner::IntersectionModulePlugin"}};

  auto test_manager = autoware::behavior_velocity_planner::generateTestManager();
  auto test_target_node = autoware::behavior_velocity_planner::generateNode(plugin_info_vec);
  publishMandatoryTopicsForOcclusion(test_manager, test_target_node);

  const std::string input_path_with_lane_id_topic =
    "behavior_velocity_planner_node/input/path_with_lane_id";
  const std::string input_odometry_topic = "behavior_velocity_planner_node/input/vehicle_odometry";
  const std::string input_dynamic_objects_topic =
    "behavior_velocity_planner_node/input/dynamic_objects";
  const std::string input_occupancy_grid_topic =
    "behavior_velocity_planner_node/input/occupancy_grid";

  const std::string output_planning_factors_topic =
    "planning/planning_factors/intersection_occlusion";

  const rclcpp::Node::SharedPtr test_node = test_manager->getTestNode();

  autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr planning_factor_msg;
  const auto test_sub =
    test_node->create_subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>(
      output_planning_factors_topic, rclcpp::QoS{1},
      [&planning_factor_msg](
        autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr msg) {
        planning_factor_msg = msg;
      });

  const auto objects = loadPathObjectsInYaml("dynamic_objects_for_occlusion_data.yaml");
  const auto odometry = loadOdometryInYaml("vehicle_odometry_for_occlusion_data.yaml");
  const auto path = loadPathWithLaneIdInYaml("path_with_lane_id_for_occlusion_data.yaml");
  const auto occupancy_grid = loadOccupancyGridInYaml("occupancy_grid_for_occlusion_data.yaml");

  for (size_t i = 0; i < 3; i++) {
    test_manager->publishInput(test_target_node, input_odometry_topic, odometry);
    test_manager->publishInput(test_target_node, input_dynamic_objects_topic, objects);
    test_manager->publishInput(test_target_node, input_path_with_lane_id_topic, path);
    test_manager->publishInput(test_target_node, input_occupancy_grid_topic, occupancy_grid);
  }

  // make sure behavior_path_planner is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // make sure planning_factor_msg is received
  EXPECT_NE(planning_factor_msg, nullptr);

  // make sure planning_factor_msg is not empty
  EXPECT_EQ(planning_factor_msg->factors.size(), 1);

  const auto & factor = planning_factor_msg->factors.front();
  EXPECT_FALSE(factor.safety_factors.is_safe);

  // make sure control_points is not empty
  EXPECT_EQ(factor.control_points.size(), 1);
  EXPECT_LT(factor.control_points.front().distance, 30.0);

  // NOTE: intersection_occlusion does not include safety factors
  EXPECT_EQ(factor.safety_factors.factors.size(), 0);

  rclcpp::shutdown();
}

}  // namespace autoware::behavior_velocity_planner
