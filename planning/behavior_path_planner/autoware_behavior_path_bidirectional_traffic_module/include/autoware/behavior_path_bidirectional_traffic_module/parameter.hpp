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

#ifndef AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__PARAMETER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__PARAMETER_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <rclcpp/node.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <limits>
#include <string_view>
namespace autoware::behavior_path_planner
{

struct BidirectionalTrafficModuleParameters
{
  double forward_looking_distance = std::numeric_limits<double>::max();
  double keep_left_ratio = 0.5;  // 0.0 to 1.0. 0.0 means keep right, 1.0 means keep left.
  double pull_over_ratio = 0.7;  // keep_left_ratio to 1.0
  double time_to_prepare_pull_over =
    0.5;  // if ego starts to pull over when ego finds an oncoming car, it would be sudden. So, ego
          // needs to prepare to pull over.
  double default_shift_distance_to_pull_over = 0.5;
  double default_shift_distance_to_keep_left = 0.5;
  double max_curvature_increase = 0.1;

  void init_from_node(rclcpp::Node * node, std::string_view ns = "bidirectional_traffic");
};

struct EgoParameters
{
  double base_link2front;
  double base_link2rear;
  double vehicle_width;

  EgoParameters(
    const double & base_link2front, const double & base_link2rear, const double & vehicle_width);

  [[nodiscard]] autoware::universe_utils::Polygon2d ego_polygon(
    const geometry_msgs::msg::Pose & ego_pose) const;
};

};  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__PARAMETER_HPP_
