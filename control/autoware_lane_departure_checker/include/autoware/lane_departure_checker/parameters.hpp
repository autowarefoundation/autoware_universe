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

#ifndef AUTOWARE__LANE_DEPARTURE_CHECKER__PARAMETERS_HPP_
#define AUTOWARE__LANE_DEPARTURE_CHECKER__PARAMETERS_HPP_

#include <autoware/boundary_departure_checker/parameters.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <rclcpp/node.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <string>
#include <vector>

namespace autoware::lane_departure_checker
{
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils::PoseDeviation;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_utils::LinearRing2d;
using boundary_departure_checker::Param;

struct NodeParam
{
  static NodeParam init(rclcpp::Node & node);
  bool will_out_of_lane_checker{};
  bool out_of_lane_checker{};
  bool boundary_departure_checker{};

  double update_rate{};
  bool visualize_lanelet{};
  bool include_right_lanes{};
  bool include_left_lanes{};
  bool include_opposite_lanes{};
  bool include_conflicting_lanes{};
  std::vector<std::string> boundary_types_to_detect;
};

Param init(rclcpp::Node & node);
}  // namespace autoware::lane_departure_checker

#endif  // AUTOWARE__LANE_DEPARTURE_CHECKER__PARAMETERS_HPP_
