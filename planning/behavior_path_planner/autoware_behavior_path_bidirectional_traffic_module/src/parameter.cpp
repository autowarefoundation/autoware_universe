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

#include <string_view>

namespace autoware::behavior_path_planner
{

void BidirectionalTrafficModuleParameters::init_from_node(rclcpp::Node * node, std::string_view ns)
{
  forward_looking_distance =
    node->declare_parameter<double>(std::string(ns) + ".forward_looking_distance");
  keep_left_ratio = node->declare_parameter<double>(std::string(ns) + ".keep_left_ratio");
  pull_over_ratio = node->declare_parameter<double>(std::string(ns) + ".pull_over_ratio");
  time_to_prepare_pull_over =
    node->declare_parameter<double>(std::string(ns) + ".time_to_prepare_pull_over");
  default_shift_distance_to_pull_over =
    node->declare_parameter<double>(std::string(ns) + ".default_shift_distance_to_pull_over");
  max_curvature_increase =
    node->declare_parameter<double>(std::string(ns) + ".max_curvature_increase");
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

};  // namespace autoware::behavior_path_planner
