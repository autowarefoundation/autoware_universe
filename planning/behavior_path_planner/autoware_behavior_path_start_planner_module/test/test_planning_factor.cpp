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
using nav_msgs::msg::Odometry;

namespace autoware::test_utils
{
std::string get_absolute_path_to_test_data(
  const std::string & package_name, const std::string & config_filename)
{
  const auto dir = ament_index_cpp::get_package_share_directory(package_name);
  return dir + "/test_data/" + config_filename;
}

}  // namespace autoware::test_utils

namespace autoware::behavior_path_planner
{

template <class T>
T loadMessageInYaml(
  const std::string & yaml_file, std::vector<std::string> corrupted_check_list = {})
{
  const auto yaml_path = autoware::test_utils::get_absolute_path_to_test_data(
    "autoware_behavior_path_start_planner_module", yaml_file);

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

class PlanningFactorTest : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    initModule();
  }

  void initModule()
  {
    test_manager_ = generateTestManager();
    test_node_ = test_manager_->getTestNode();
    test_target_node_ = generateNode(
      {"start_planner"}, {"autoware::behavior_path_planner::StartPlannerModuleManager"});
  }

  const std::string input_route_topic = "behavior_path_planner/input/route";
  const std::string input_odometry_topic = "behavior_path_planner/input/odometry";
  const std::string input_dynamic_objects_topic = "behavior_path_planner/input/perception";

  void setupForStartBlocked()
  {
    publishMandatoryTopics(test_manager_, test_target_node_);

    test_manager_->publishInput(
      test_target_node_, "behavior_path_planner/input/vector_map",
      autoware::test_utils::makeMapBinMsg("autoware_test_utils", "intersection/lanelet2_map.osm"));

    const std::string output_planning_factors_topic = "planning/planning_factors/start_planner";
    sub_planning_factor_ =
      test_node_->create_subscription<autoware_internal_planning_msgs::msg::PlanningFactorArray>(
        output_planning_factors_topic, rclcpp::QoS{1},
        [this](autoware_internal_planning_msgs::msg::PlanningFactorArray::SharedPtr msg) {
          planning_factor_msg_ = msg;
        });

    route_ = loadRouteInYaml();
    odometry_ = loadOdometryInYaml();
    objects_ = loadPathObjectsInYaml();

    test_manager_->publishInput(test_target_node_, input_dynamic_objects_topic, objects_);
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

    size_t total_safety_factors = 0;
    size_t total_object_id_hits = 0;
    for (const auto & factor : planning_factor_msg_->factors) {
      EXPECT_FALSE(factor.safety_factors.is_safe);

      // make sure control_points is not empty
      EXPECT_GE(factor.control_points.size(), 1);
      for (auto & control_point : factor.control_points) {
        EXPECT_LT(control_point.distance, 200.0);
      }

      for (auto & safety_factor : factor.safety_factors.factors) {
        ++total_safety_factors;

        const auto expected_object_type =
          autoware_internal_planning_msgs::msg::SafetyFactor::OBJECT;
        EXPECT_EQ(safety_factor.type, expected_object_type);
        EXPECT_FALSE(safety_factor.is_safe);
        EXPECT_EQ(safety_factor.points.size(), 1);

        const bool is_object_id_included = std::any_of(
          objects_.objects.begin(), objects_.objects.end(),
          [&](const auto & object) { return object.object_id == safety_factor.object_id; });
        if (is_object_id_included) {
          ++total_object_id_hits;
        }
      }
    }

    // make sure safety_factors is not empty
    EXPECT_GE(total_safety_factors, 1);

    // make sure object_id is included in safety_factors
    EXPECT_GE(total_object_id_hits, 1);
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
};

TEST_F(PlanningFactorTest, StartPrevention)
{
  setupForStartBlocked();
  validatePlanningFactor();
}

}  // namespace autoware::behavior_path_planner
