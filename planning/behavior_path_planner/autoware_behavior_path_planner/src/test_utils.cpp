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

#include "autoware/behavior_path_planner/test_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <tier4_planning_msgs/msg/lateral_offset.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();

  // set subscriber with topic name: behavior_path_planner → test_node_
  test_manager->subscribeOutput<autoware_internal_planning_msgs::msg::PathWithLaneId>(
    "behavior_path_planner/output/path");

  return test_manager;
}

std::shared_ptr<BehaviorPathPlannerNode> generateNode(
  const std::vector<std::string> & module_name_vec,
  const std::vector<std::string> & plugin_name_vec)
{
  auto node_options = rclcpp::NodeOptions{};
  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto behavior_path_planner_dir =
    ament_index_cpp::get_package_share_directory("autoware_behavior_path_planner");

  std::vector<std::string> plugin_names;
  for (const auto & plugin_name : plugin_name_vec) {
    plugin_names.emplace_back(plugin_name);
  }

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("launch_modules", plugin_names);
  node_options.parameter_overrides(params);

  const auto get_behavior_path_module_config = [](const std::string & module) {
    const auto package_name = "autoware_behavior_path_" + module + "_module";
    const auto package_path = ament_index_cpp::get_package_share_directory(package_name);
    return package_path + "/config/" + module + ".param.yaml";
  };

  auto yaml_files = std::vector<std::string>{
    autoware_test_utils_dir + "/config/test_common.param.yaml",
    autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
    autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
    behavior_path_planner_dir + "/config/behavior_path_planner.param.yaml",
    behavior_path_planner_dir + "/config/drivable_area_expansion.param.yaml",
    behavior_path_planner_dir + "/config/scene_module_manager.param.yaml"};
  for (const auto & module_name : module_name_vec) {
    yaml_files.push_back(get_behavior_path_module_config(module_name));
  }

  autoware::test_utils::updateNodeOptions(node_options, yaml_files);

  return std::make_shared<BehaviorPathPlannerNode>(node_options);
}

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<BehaviorPathPlannerNode> test_target_node)
{
  // publish necessary topics from test_manager
  test_manager->publishInput(
    test_target_node, "behavior_path_planner/input/odometry",
    autoware::test_utils::makeInitialPose());
  test_manager->publishInput(
    test_target_node, "behavior_path_planner/input/accel",
    geometry_msgs::msg::AccelWithCovarianceStamped{});
  test_manager->publishInput(
    test_target_node, "behavior_path_planner/input/perception",
    autoware_perception_msgs::msg::PredictedObjects{});
  test_manager->publishInput(
    test_target_node, "behavior_path_planner/input/occupancy_grid_map",
    autoware::test_utils::makeCostMapMsg());
  test_manager->publishInput(
    test_target_node, "behavior_path_planner/input/scenario",
    autoware::test_utils::makeScenarioMsg(
      autoware_internal_planning_msgs::msg::Scenario::LANEDRIVING));
  test_manager->publishInput(
    test_target_node, "behavior_path_planner/input/vector_map",
    autoware::test_utils::makeMapBinMsg());
  test_manager->publishInput(
    test_target_node, "behavior_path_planner/input/costmap",
    autoware::test_utils::makeCostMapMsg());
  test_manager->publishInput(
    test_target_node, "system/operation_mode/state",
    autoware_adapi_v1_msgs::msg::OperationModeState{});
  test_manager->publishInput(
    test_target_node, "behavior_path_planner/input/lateral_offset",
    tier4_planning_msgs::msg::LateralOffset{});
}
}  // namespace autoware::behavior_path_planner
