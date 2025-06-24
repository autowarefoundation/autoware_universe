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

#include "autoware/behavior_path_planner/test_utils.hpp"

#include <autoware_test_utils/mock_data_parser.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_planning_msgs/msg/path.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

using autoware::behavior_path_planner::generateNode;
using autoware::behavior_path_planner::generateTestManager;
using autoware::behavior_path_planner::publishMandatoryTopics;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using nav_msgs::msg::MapMetaData;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;

namespace autoware::test_utils
{
std::string get_absolute_path_to_test_data(
  const std::string & package_name, const std::string & config_filename)
{
  const auto dir = ament_index_cpp::get_package_share_directory(package_name);
  return dir + "/test_data/" + config_filename;
}

template <>
MapMetaData parse(const YAML::Node & node)
{
  MapMetaData msg;
  msg.map_load_time = parse<builtin_interfaces::msg::Time>(node["map_load_time"]);
  msg.resolution = node["resolution"].as<float>();
  msg.width = node["width"].as<uint32_t>();
  msg.height = node["height"].as<uint32_t>();
  msg.origin = parse<geometry_msgs::msg::Pose>(node["origin"]);
  return msg;
}

template <>
OccupancyGrid parse(const YAML::Node & node)
{
  OccupancyGrid msg;
  msg.header = parse<std_msgs::msg::Header>(node["header"]);
  msg.info = parse<MapMetaData>(node["info"]);
  msg.data = node["data"].as<std::vector<int8_t>>();
  return msg;
}

}  // namespace autoware::test_utils

namespace autoware::behavior_path_planner
{

template <class T>
T loadMessageInYaml(
  const std::string & yaml_file, std::vector<std::string> corrupted_check_list = {})
{
  const auto yaml_path = autoware::test_utils::get_absolute_path_to_test_data(
    "autoware_behavior_path_lane_change_module", yaml_file);

  YAML::Node node = YAML::LoadFile(yaml_path);
  for (auto & word : corrupted_check_list) {
    if (node[word].IsNull()) {
      throw std::runtime_error(
        "Failed to parse YAML file: " + yaml_path + ". The file might be corrupted.");
    }
  }

  return autoware::test_utils::parse<T>(node);
}

LaneletRoute loadRouteInYaml(const std::string & yaml_file = "route_data.yaml")
{
  const auto route = loadMessageInYaml<LaneletRoute>(yaml_file, {"start_pose", "goal_pose"});
  if (route.segments.empty()) {
    throw std::runtime_error(
      "Failed to parse YAML file: " + yaml_file + ". The file might be corrupted.");
  }
  return route;
}

Odometry loadOdometryInYaml(const std::string & yaml_file = "vehicle_odometry_data.yaml")
{
  return loadMessageInYaml<Odometry>(yaml_file, {"pose"});
}

PredictedObjects loadPathObjectsInYaml(const std::string & yaml_file = "dynamic_objects_data.yaml")
{
  return loadMessageInYaml<PredictedObjects>(yaml_file, {"objects"});
}

OccupancyGrid loadOccupancyGridInYaml(const std::string & yaml_file = "occupancy_grid_data.yaml")
{
  return loadMessageInYaml<OccupancyGrid>(yaml_file, {"data", "info"});
}

class PlanningFactorTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    initModule();

    total_safety_factors_ = 0;
    total_object_id_hits_ = 0;
  }

  void initModule()
  {
    test_manager_ = generateTestManager();
    test_node_ = test_manager_->getTestNode();
    test_target_node_ = generateNode(
      {"lane_change"}, {"autoware::behavior_path_planner::LaneChangeRightModuleManager",
                        "autoware::behavior_path_planner::LaneChangeLeftModuleManager"});
    publishMandatoryTopics(test_manager_, test_target_node_);

    test_manager_->publishInput(
      test_target_node_, "behavior_path_planner/input/vector_map",
      autoware::test_utils::makeMapBinMsg("autoware_test_utils", "intersection/lanelet2_map.osm"));
  }

  const std::string input_route_topic = "behavior_path_planner/input/route";
  const std::string input_odometry_topic = "behavior_path_planner/input/odometry";
  const std::string input_dynamic_objects_topic = "behavior_path_planner/input/perception";
  const std::string input_occupancy_grid_topic = "behavior_path_planner/input/occupancy_grid_map";

  void setupForLaneChange(
    const std::string output_planning_factors_topic,
    const std::string input_dynamic_objects_data_file,
    const std::string input_occupancy_grid_data_file, const std::string input_odometry_data_file,
    const std::string input_route_data_file)
  {
    sub_planning_factor_ =
      test_node_->create_subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>(
        output_planning_factors_topic, rclcpp::QoS{1},
        [this](autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr msg) {
          planning_factor_msg_ = msg;
        });

    objects_ = loadPathObjectsInYaml(input_dynamic_objects_data_file);
    occupancy_grid_ = loadOccupancyGridInYaml(input_occupancy_grid_data_file);
    odometry_ = loadOdometryInYaml(input_odometry_data_file);
    route_ = loadRouteInYaml(input_route_data_file);

    test_manager_->publishInput(test_target_node_, input_dynamic_objects_topic, objects_);
    test_manager_->publishInput(test_target_node_, input_occupancy_grid_topic, occupancy_grid_);
    test_manager_->publishInput(test_target_node_, input_odometry_topic, odometry_);
    test_manager_->publishInput(test_target_node_, input_route_topic, route_);
  }

  void validatePlanningFactor()
  {
    // make sure behavior_path_planner is running
    EXPECT_GE(test_manager_->getReceivedTopicNum(), 1);

    // make sure planning_factor_msg_ is received
    EXPECT_NE(planning_factor_msg_, nullptr);

    // make sure planning_factor_msg_ is not empty
    EXPECT_GE(planning_factor_msg_->factors.size(), 1);

    for (const auto & factor : planning_factor_msg_->factors) {
      EXPECT_FALSE(factor.safety_factors.is_safe);

      // make sure control_points is not empty
      EXPECT_GE(factor.control_points.size(), 1);
      for (auto & control_point : factor.control_points) {
        EXPECT_LT(control_point.distance, 200.0);
      }

      validateSafetyFactors(factor.safety_factors.factors);
    }

    // make sure safety_factors is not empty
    EXPECT_GE(total_safety_factors_, 1);

    // make sure object_id is included in safety_factors
    EXPECT_GE(total_object_id_hits_, 1);
  }

  void validateSafetyFactors(
    const std::vector<autoware_internal_planning_msgs::msg::SafetyFactor> & safety_factors)
  {
    for (auto & safety_factor : safety_factors) {
      ++total_safety_factors_;

      const auto expected_object_type = autoware_internal_planning_msgs::msg::SafetyFactor::OBJECT;
      EXPECT_EQ(safety_factor.type, expected_object_type);
      EXPECT_FALSE(safety_factor.is_safe);
      EXPECT_EQ(safety_factor.points.size(), 1);

      const bool is_object_id_included = std::any_of(
        objects_.objects.begin(), objects_.objects.end(),
        [&](const auto & object) { return object.object_id == safety_factor.object_id; });
      if (is_object_id_included) {
        ++total_object_id_hits_;
      }
    }
  }

  void TearDown() override
  {
    test_manager_ = nullptr;
    test_target_node_ = nullptr;
    sub_planning_factor_ = nullptr;
    planning_factor_msg_ = nullptr;
    rclcpp::shutdown();
  }

  std::shared_ptr<PlanningInterfaceTestManager> test_manager_;
  std::shared_ptr<BehaviorPathPlannerNode> test_target_node_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>::SharedPtr
    sub_planning_factor_;
  autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr planning_factor_msg_;
  LaneletRoute route_;
  Odometry odometry_;
  PredictedObjects objects_;
  OccupancyGrid occupancy_grid_;
  size_t total_safety_factors_ = 0;
  size_t total_object_id_hits_ = 0;
};

TEST_F(PlanningFactorTest, LaneChangeRight)
{
  const std::string output_planning_factors_topic = "planning/planning_factors/lane_change_right";
  const std::string input_dynamic_objects_data_file = "dynamic_objects_data_lane_change_right.yaml";
  const std::string input_occupancy_grid_data_file = "occupancy_grid_data_lane_change_right.yaml";
  const std::string input_odometry_data_file = "vehicle_odometry_data_lane_change_right.yaml";
  const std::string input_route_data_file = "route_data_lane_change_right.yaml";

  setupForLaneChange(
    output_planning_factors_topic, input_dynamic_objects_data_file, input_occupancy_grid_data_file,
    input_odometry_data_file, input_route_data_file);
  validatePlanningFactor();
}

TEST_F(PlanningFactorTest, LaneChangeLeft)
{
  const std::string output_planning_factors_topic = "planning/planning_factors/lane_change_left";
  const std::string input_dynamic_objects_data_file = "dynamic_objects_data_lane_change_left.yaml";
  const std::string input_occupancy_grid_data_file = "occupancy_grid_data_lane_change_left.yaml";
  const std::string input_odometry_data_file = "vehicle_odometry_data_lane_change_left.yaml";
  const std::string input_route_data_file = "route_data_lane_change_left.yaml";
  setupForLaneChange(
    output_planning_factors_topic, input_dynamic_objects_data_file, input_occupancy_grid_data_file,
    input_odometry_data_file, input_route_data_file);
  validatePlanningFactor();
}

}  // namespace autoware::behavior_path_planner
