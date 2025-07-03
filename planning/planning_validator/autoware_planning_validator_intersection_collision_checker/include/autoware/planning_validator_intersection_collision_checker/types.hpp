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

#ifndef AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__TYPES_HPP_
#define AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__TYPES_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_planning_validator_intersection_collision_checker/intersection_collision_checker_node_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::planning_validator
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using lanelet::BasicLineString2d;
using lanelet::BasicPolygon2d;
using lanelet::BasicPolygons2d;
using lanelet::ConstLanelet;
using lanelet::ConstLanelets;
using route_handler::Direction;
using route_handler::RouteHandler;

using TrajectoryPoints = std::vector<TrajectoryPoint>;

struct EgoTrajectory
{
  TrajectoryPoints front_traj;
  TrajectoryPoints back_traj;
  size_t front_index;
  size_t back_index;
};

struct TargetLanelet
{
  lanelet::Id id;
  lanelet::ConstLanelets lanelets;
  geometry_msgs::msg::Pose overlap_point;
  std::pair<double, double> ego_overlap_time;
  bool is_active{false};

  TargetLanelet() = default;
  TargetLanelet(
    lanelet::Id id, const lanelet::ConstLanelets & lanelets,
    const geometry_msgs::msg::Pose & overlap_point,
    const std::pair<double, double> ego_overlap_time, const bool is_active = true)
  : id(id),
    lanelets(lanelets),
    overlap_point(overlap_point),
    ego_overlap_time(ego_overlap_time),
    is_active(is_active)
  {
  }
};

struct EgoLanelets
{
  lanelet::ConstLanelet first_turn_lanelet;
  lanelet::ConstLanelets trajectory_lanelets;
  lanelet::ConstLanelets connected_lanelets;
};

struct PCDObject
{
  rclcpp::Time last_update_time;
  geometry_msgs::msg::Pose pose;
  lanelet::Id overlap_lanelet_id;
  double track_duration{};
  double distance_to_overlap{};
  double delay_compensated_distance_to_overlap{};
  double velocity{};
  double moving_time{};
  double ttc{};
};

using PCDObjectsMap = std::unordered_map<lanelet::Id, PCDObject>;
using TargetLaneletsMap = std::unordered_map<lanelet::Id, TargetLanelet>;

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__TYPES_HPP_
