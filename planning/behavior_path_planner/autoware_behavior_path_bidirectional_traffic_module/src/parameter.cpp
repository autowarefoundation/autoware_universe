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

#include "autoware/behavior_path_bidirectional_traffic_module/parameter.hpp"

#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"

#include <string>
#include <string_view>

namespace autoware::behavior_path_planner
{

void BidirectionalTrafficModuleParameters::init_from_node(rclcpp::Node * node, std::string_view ns)
{
  time_to_prepare_pull_over =
    node->declare_parameter<double>(std::string(ns) + ".time_to_prepare_pull_over");
  min_distance_from_roadside =
    node->declare_parameter<double>(std::string(ns) + ".min_distance_from_roadside");
  keep_left_distance_from_center_line =
    node->declare_parameter<double>(std::string(ns) + ".keep_left_distance_from_center_line");
  shift_distance_to_pull_over_from_center_line = node->declare_parameter<double>(
    std::string(ns) + ".shift_distance_to_pull_over_from_center_line");
  wait_time_for_oncoming_car =
    node->declare_parameter<double>(std::string(ns) + ".wait_time_for_oncoming_car");
  max_lateral_jerk = node->declare_parameter<double>(std::string(ns) + ".max_lateral_jerk");
  min_lateral_jerk = node->declare_parameter<double>(std::string(ns) + ".min_lateral_jerk");
}

EgoParameters::EgoParameters(
  const double & base_link2front, const double & base_link2rear, const double & vehicle_width)
: base_link2front(base_link2front), base_link2rear(base_link2rear), vehicle_width(vehicle_width)
{
}

[[nodiscard]] autoware::universe_utils::Polygon2d EgoParameters::ego_polygon(
  const geometry_msgs::msg::Pose & ego_pose) const
{
  return autoware::universe_utils::toFootprint(
    ego_pose, base_link2front, base_link2rear, vehicle_width);
}

EgoParameters EgoParameters::from_planner_data(const PlannerData & planner_data)
{
  return {
    planner_data.parameters.base_link2front, planner_data.parameters.base_link2rear,
    planner_data.parameters.vehicle_width};
}

};  // namespace autoware::behavior_path_planner
