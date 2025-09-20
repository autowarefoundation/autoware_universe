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
// pull_out_test_utils.cpp
#include "start_planner_test_helper.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/behavior_path_start_planner_module/util.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager_utils.hpp>
#include <autoware/pyplot/pyplot.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner::testing
{
using autoware::test_utils::get_absolute_path_to_config;
using autoware_planning_test_manager::utils::makeBehaviorRouteFromLaneId;
namespace start_planner_utils = autoware::behavior_path_planner::start_planner_utils;

std::string get_absolute_path_to_test_data(
  const std::string & package_name, const std::string & route_filename)
{
  const auto dir = ament_index_cpp::get_package_share_directory(package_name);
  return dir + "/test_data/" + route_filename;
}

template <class T>
T loadMessageInYaml(
  const std::string & yaml_file, std::vector<std::string> corrupted_check_list = {})
{
  const auto yaml_path =
    get_absolute_path_to_test_data("autoware_behavior_path_start_planner_module", yaml_file);

  YAML::Node node = YAML::LoadFile(yaml_path);
  for (auto & word : corrupted_check_list) {
    if (node[word].IsNull()) {
      throw std::runtime_error(
        "Failed to parse YAML file: " + yaml_path + ". The file might be corrupted.");
    }
  }

  return autoware::test_utils::parse<T>(node);
}

rclcpp::NodeOptions StartPlannerTestHelper::make_node_options()
{
  // Load common configuration files
  auto node_options = rclcpp::NodeOptions{};

  const auto common_param_path =
    get_absolute_path_to_config("autoware_test_utils", "test_common.param.yaml");
  const auto nearest_search_param_path =
    get_absolute_path_to_config("autoware_test_utils", "test_nearest_search.param.yaml");
  const auto vehicle_info_param_path =
    get_absolute_path_to_config("autoware_test_utils", "test_vehicle_info.param.yaml");
  const auto behavior_path_planner_param_path = get_absolute_path_to_config(
    "autoware_behavior_path_planner", "behavior_path_planner.param.yaml");
  const auto drivable_area_expansion_param_path = get_absolute_path_to_config(
    "autoware_behavior_path_planner", "drivable_area_expansion.param.yaml");
  const auto scene_module_manager_param_path = get_absolute_path_to_config(
    "autoware_behavior_path_planner", "scene_module_manager.param.yaml");
  const auto start_planner_param_path = get_absolute_path_to_config(
    "autoware_behavior_path_start_planner_module", "start_planner.param.yaml");

  autoware::test_utils::updateNodeOptions(
    node_options, {common_param_path, nearest_search_param_path, vehicle_info_param_path,
                   behavior_path_planner_param_path, drivable_area_expansion_param_path,
                   scene_module_manager_param_path, start_planner_param_path});

  return node_options;
}

void StartPlannerTestHelper::set_odometry(
  std::shared_ptr<PlannerData> & planner_data, const geometry_msgs::msg::Pose & start_pose)
{
  auto odometry = std::make_shared<nav_msgs::msg::Odometry>();
  odometry->pose.pose = start_pose;
  odometry->header.frame_id = "map";
  planner_data->self_odometry = odometry;
}

void StartPlannerTestHelper::set_route(
  std::shared_ptr<PlannerData> & planner_data, const int route_start_lane_id,
  const int route_goal_lane_id)
{
  const auto shoulder_map_path = autoware::test_utils::get_absolute_path_to_lanelet_map(
    "autoware_test_utils", "road_shoulder/lanelet2_map.osm");
  const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(shoulder_map_path, 0.5);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map_bin_msg);

  const auto route = makeBehaviorRouteFromLaneId(
    route_start_lane_id, route_goal_lane_id, "autoware_test_utils",
    "road_shoulder/lanelet2_map.osm");
  route_handler->setRoute(route);
  planner_data->route_handler = route_handler;
}

void StartPlannerTestHelper::set_costmap(
  std::shared_ptr<PlannerData> & planner_data, const geometry_msgs::msg::Pose & start_pose,
  const double grid_resolution, const double grid_length_x, const double grid_length_y)
{
  nav_msgs::msg::OccupancyGrid costmap;
  costmap.header.frame_id = "map";
  costmap.info.width = static_cast<uint>(grid_length_x / grid_resolution);
  costmap.info.height = static_cast<uint>(grid_length_y / grid_resolution);
  costmap.info.resolution = grid_resolution;

  costmap.info.origin.position.x = start_pose.position.x - grid_length_x / 2;
  costmap.info.origin.position.y = start_pose.position.y - grid_length_y / 2;
  costmap.data = std::vector<int8_t>(costmap.info.width * costmap.info.height, 0);

  planner_data->costmap = std::make_shared<nav_msgs::msg::OccupancyGrid>(costmap);
}

autoware_planning_msgs::msg::LaneletRoute StartPlannerTestHelper::set_route_from_yaml(
  std::shared_ptr<PlannerData> & planner_data, const std::string & yaml_file)
{
  const auto shoulder_map_path = autoware::test_utils::get_absolute_path_to_lanelet_map(
    "autoware_test_utils", "road_shoulder/lanelet2_map.osm");
  const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(shoulder_map_path, 0.5);
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map_bin_msg);

  const auto route = loadMessageInYaml<autoware_planning_msgs::msg::LaneletRoute>(
    yaml_file, {"start_pose", "goal_pose"});
  route_handler->setRoute(route);
  planner_data->route_handler = route_handler;

  return route;
}

void StartPlannerTestHelper::plot_lanelet(
  autoware::pyplot::Axes & ax, const std::vector<lanelet::ConstLanelet> & lanelets)
{
  for (lanelet::ConstLanelet lanelet : lanelets) {
    const auto lefts = lanelet.leftBound();
    const auto rights = lanelet.rightBound();
    std::vector<double> xs_left, ys_left;
    for (const auto & point : lefts) {
      xs_left.push_back(point.x());
      ys_left.push_back(point.y());
    }

    std::vector<double> xs_right, ys_right;
    for (const auto & point : rights) {
      xs_right.push_back(point.x());
      ys_right.push_back(point.y());
    }

    std::vector<double> xs_center, ys_center;
    for (const auto & point : lanelet.centerline()) {
      xs_center.push_back(point.x());
      ys_center.push_back(point.y());
    }

    ax.plot(Args(xs_left, ys_left), Kwargs("color"_a = "grey", "linewidth"_a = 0.5));
    ax.plot(Args(xs_right, ys_right), Kwargs("color"_a = "grey", "linewidth"_a = 0.5));
    ax.plot(
      Args(xs_center, ys_center),
      Kwargs("color"_a = "grey", "linewidth"_a = 0.5, "linestyle"_a = "dashed"));
  }
}

void StartPlannerTestHelper::plot_footprint(
  autoware::pyplot::Axes & ax, const geometry_msgs::msg::Pose & pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  // Vehicle rectangle corners in local frame
  const double base_to_front = vehicle_info.front_overhang_m + vehicle_info.wheel_base_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;
  const double base_to_left = vehicle_info.wheel_tread_m / 2.0 + vehicle_info.left_overhang_m;

  const double base_to_right = vehicle_info.wheel_tread_m / 2.0 + vehicle_info.right_overhang_m;
  std::vector<std::pair<double, double>> corners = {
    {base_to_front, base_to_left},    // Front left
    {base_to_front, -base_to_right},  // Front right
    {-base_to_rear, -base_to_right},  // Rear right
    {-base_to_rear, base_to_left},    // Rear left
  };

  // footprint
  double yaw = tf2::getYaw(pose.orientation);
  std::vector<double> x, y;
  for (const auto & [cx, cy] : corners) {
    double gx = pose.position.x + cx * std::cos(yaw) - cy * std::sin(yaw);
    double gy = pose.position.y + cx * std::sin(yaw) + cy * std::cos(yaw);
    x.push_back(gx);
    y.push_back(gy);
  }
  ax.fill(Args(x, y), Kwargs("color"_a = "red", "linewidth"_a = 1.5, "alpha"_a = 0.5));

  // arrow for vehicle orientation
  const double arrow_length = 5.0;  // Length of the arrow
  const double arrow_x = arrow_length * std::cos(yaw);
  const double arrow_y = arrow_length * std::sin(yaw);
  ax.quiver(
    Args(pose.position.x, pose.position.y, arrow_x, arrow_y),
    Kwargs(
      "color"_a = "green", "width"_a = 0.1, "angles"_a = "xy", "scale_units"_a = "xy",
      "scale"_a = 1.0));
}

void StartPlannerTestHelper::plot_and_save_path(
  const std::vector<autoware_internal_planning_msgs::msg::PathWithLaneId> & partial_paths,
  const std::shared_ptr<PlannerData> & planner_data,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const PlannerType planner_type,
  const std::string & filename)
{
  if (partial_paths.empty()) {
    std::cerr << "Path is empty" << std::endl;
    return;
  }

  // Get lanelets that actually overlap with the path using existing util functions
  std::vector<lanelet::ConstLanelet> lanelets;
  std::set<lanelet::Id> added_lanelet_ids;

  // Get all available lanelets from the map
  const lanelet::LaneletMap & map = *planner_data->route_handler->getLaneletMapPtr();
  lanelet::ConstLanelets all_lanelets;
  for (const auto & lanelet : map.laneletLayer) {
    all_lanelets.push_back(lanelet);
  }

  for (const auto & partial_path : partial_paths) {
    for (const auto & point : partial_path.points) {
      const auto lane_ids = start_planner_utils::get_lane_ids_from_pose(
        point.point.pose, all_lanelets, std::vector<int64_t>{});

      for (const auto & lane_id : lane_ids) {
        if (added_lanelet_ids.find(lane_id) == added_lanelet_ids.end()) {
          const auto lanelet = planner_data->route_handler->getLaneletsFromId(lane_id);
          lanelets.push_back(lanelet);
          added_lanelet_ids.insert(lane_id);
        }
      }
    }
  }

  // Initialize pyplot
  static pybind11::scoped_interpreter guard{};
  auto plt = autoware::pyplot::import();

  auto [fig, axes] = plt.subplots(1, 1);

  plt.title(Args("Generated Pull Out Path"));
  plt.xlabel(Args("Position x [m]"));
  plt.ylabel(Args("Position y [m]"));
  axes[0].set_aspect(Args("equal"));

  // plot lanelets
  if (!lanelets.empty()) {
    plot_lanelet(axes[0], lanelets);
  } else {
    std::cout << "No lanelets to plot." << std::endl;
  }

  // plot path line
  for (const auto & path : partial_paths) {
    // Extract x and y coordinates from path points
    std::vector<double> x_coords, y_coords;
    x_coords.reserve(path.points.size());
    y_coords.reserve(path.points.size());

    for (const auto & point : path.points) {
      x_coords.push_back(point.point.pose.position.x);
      y_coords.push_back(point.point.pose.position.y);
    }

    plt.plot(
      Args(x_coords, y_coords), Kwargs("color"_a = "blue", "linewidth"_a = 1.0));  // Blue line
  }

  // Plot vehicle footprint at start and end poses
  const auto start_pose = partial_paths.front().points.front().point.pose;
  plot_footprint(axes[0], start_pose, vehicle_info);
  const auto end_pose = partial_paths.back().points.back().point.pose;
  plot_footprint(axes[0], end_pose, vehicle_info);

  const std::string file_path = __FILE__;
  const std::string package_name = "autoware_behavior_path_start_planner_module";
  size_t pos = file_path.rfind(package_name);
  if (pos != std::string::npos) {
    std::string test_result_dir = file_path.substr(0, pos) + package_name + "/test_results/";
    std::string output_path;
    switch (planner_type) {
      case PlannerType::CLOTHOID:
        output_path = test_result_dir + "clothoid_pull_out/";
        break;
      case PlannerType::SHIFT:
        output_path = test_result_dir + "shift_pull_out/";
        break;
      case PlannerType::GEOMETRIC:
        output_path = test_result_dir + "geometric_pull_out/";
        break;
      case PlannerType::FREESPACE:
        output_path = test_result_dir + "freespace_pull_out/";
        break;
      default:
        // Don't save plot for default case
        std::cerr << "Unsupported planner type for plotting: " << static_cast<int>(planner_type)
                  << std::endl;
        return;
    }
    // Save the plot
    plt.savefig(Args(output_path + filename), Kwargs("dpi"_a = 300));
  } else {
    std::cerr << "Failed to get test_results directory path. Cannot save plot." << std::endl;
  }
}

}  // namespace autoware::behavior_path_planner::testing
