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

#ifndef AUTOWARE__PLANNING_VALIDATOR_COLLISION_CHECKER__TYPES_HPP_
#define AUTOWARE__PLANNING_VALIDATOR_COLLISION_CHECKER__TYPES_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <unordered_map>
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

struct TargetLanelet
{
  lanelet::Id id;
  lanelet::ConstLanelets lanelets;
  geometry_msgs::msg::Pose overlap_point;
  double ego_time_to_reach{};

  TargetLanelet() = default;
  TargetLanelet(
    lanelet::Id id, const lanelet::ConstLanelets & lanelets,
    const geometry_msgs::msg::Pose & overlap_point, const double ego_time_to_reach)
  : id(id), lanelets(lanelets), overlap_point(overlap_point), ego_time_to_reach(ego_time_to_reach)
  {
  }
};

using TargetLanelets = std::vector<TargetLanelet>;

struct CollisionCheckerLanelets
{
  lanelet::ConstLanelets trajectory_lanelets;
  lanelet::ConstLanelets connected_lanelets;
  TargetLanelets target_lanelets;
};

struct PCDObject
{
  rclcpp::Time last_update_time;
  geometry_msgs::msg::Pose pose;
  lanelet::Id overlap_lanelet_id;
  double track_duration{};
  double distance_to_overlap{};
  double velocity{};
  double ttc{};
};

using PCDObjectsMap = std::unordered_map<lanelet::Id, PCDObject>;

struct DirectionCheckFlags
{
  bool enable{};
  bool check_oncoming_lanes{};
  bool check_crossing_lanes{};
  bool check_turning_lanes{};
};

struct PointcloudParams
{
  double height_buffer{};
  double min_height{};
  double observation_time{};

  struct VoxelGridFilterParams
  {
    double x{};
    double y{};
    double z{};
  } voxel_grid_filter;

  struct ClusteringParams
  {
    double tolerance{};
    double min_height{};
    int min_size{};
    int max_size{};
  } clustering;
};

struct CollisionCheckerParams
{
  bool enable{};
  bool is_critical{};
  double detection_range{};
  double ttc_threshold{};
  double ego_deceleration{};
  double min_time_horizon{};
  double timeout{};

  DirectionCheckFlags right_turn;
  DirectionCheckFlags left_turn;

  PointcloudParams pointcloud;
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR_COLLISION_CHECKER__TYPES_HPP_
