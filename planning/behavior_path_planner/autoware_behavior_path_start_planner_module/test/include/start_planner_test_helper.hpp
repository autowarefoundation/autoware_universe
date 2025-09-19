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
#pragma once

#include <autoware/behavior_path_start_planner_module/start_planner_module.hpp>
#include <autoware/behavior_path_start_planner_module/util.hpp>
#include <autoware/pyplot/pyplot.hpp>
#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <tf2/utils.h>

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner::testing
{

class StartPlannerTestHelper
{
public:
  static rclcpp::NodeOptions make_node_options();

  static void set_odometry(
    std::shared_ptr<PlannerData> & planner_data, const geometry_msgs::msg::Pose & start_pose);
  static void set_route(
    std::shared_ptr<PlannerData> & planner_data, const int route_start_lane_id,
    const int route_goal_lane_id);
  static void set_costmap(
    std::shared_ptr<PlannerData> & planner_data, const geometry_msgs::msg::Pose & start_pose,
    const double grid_resolution, const double grid_length_x, const double grid_length_y);
  static autoware_planning_msgs::msg::LaneletRoute set_route_from_yaml(
    std::shared_ptr<PlannerData> & planner_data, const std::string & yaml_file);

  // Path plotting functions
  static void plot_and_save_path(
    const std::vector<autoware_internal_planning_msgs::msg::PathWithLaneId> & partial_paths,
    const std::shared_ptr<PlannerData> & planner_data,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const PlannerType planner_type,
    const std::string & filename);

private:
  static void plot_lanelet(
    autoware::pyplot::Axes & ax, const std::vector<lanelet::ConstLanelet> & lanelets);
  static void plot_footprint(
    autoware::pyplot::Axes & ax, const geometry_msgs::msg::Pose & pose,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);
};

}  // namespace autoware::behavior_path_planner::testing
