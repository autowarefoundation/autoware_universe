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

#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <deque>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::planning_validator
{
using autoware_internal_debug_msgs::msg::StringStamped;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using lanelet::BasicLineString2d;
using lanelet::BasicPolygon2d;
using lanelet::BasicPolygons2d;
using lanelet::ConstLanelet;
using lanelet::ConstLanelets;
using route_handler::Direction;
using route_handler::RouteHandler;
using sensor_msgs::msg::PointCloud2;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

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
  lanelet::ConstLanelets turn_lanelets;
  lanelet::ConstLanelets trajectory_lanelets;
  lanelet::ConstLanelets connected_lanelets;
};

struct PCDObject
{
private:
  size_t buffer_size{10ul};

public:
  rclcpp::Time last_update_time;
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Point overlap_point;
  lanelet::Id overlap_lanelet_id;
  std::deque<double> distance_history;
  std::deque<double> timestamp_history;
  double track_duration{};
  double distance_to_overlap{};
  double delay_compensated_distance_to_overlap{};
  double velocity{};
  double moving_time{};
  double ttc{std::numeric_limits<double>::max()};
  bool is_safe{true};
  bool is_reliable{false};
  bool is_moving{false};

  PCDObject() = default;

  PCDObject(
    const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Point & overlap_point,
    const rclcpp::Time & time_stamp, const lanelet::Id ll_id, const double distance)
  : last_update_time(time_stamp),
    pose(pose),
    overlap_point(overlap_point),
    overlap_lanelet_id(ll_id),
    distance_to_overlap(distance),
    delay_compensated_distance_to_overlap(distance)
  {
    distance_history.push_back(distance);
    timestamp_history.push_back(0.0);
  }

  void set_buffer_size(const size_t size) { buffer_size = size; }

  void update_history(const double dist, const double dt)
  {
    const auto last_timestamp = timestamp_history.empty() ? 0.0 : timestamp_history.back();

    distance_history.push_back(dist);
    timestamp_history.push_back(last_timestamp + dt);

    if (distance_history.size() > buffer_size) distance_history.pop_front();
    if (timestamp_history.size() > buffer_size) timestamp_history.pop_front();
  }

  bool update_velocity()
  {
    // need at least two measurements to initialize velocity
    if (distance_history.size() < 2) return false;

    // perform linear regression to estimate velocity from first few samples
    const auto n = distance_history.size();
    const auto x0 = timestamp_history.front();

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xy = 0.0;
    double sum_x2 = 0.0;
    for (size_t i = 0; i < n; ++i) {
      double x_val = timestamp_history[i] - x0;
      double y_val = distance_history[i];
      sum_x += x_val;
      sum_y += y_val;
      sum_xy += x_val * y_val;
      sum_x2 += x_val * x_val;
    }

    double denominator = n * sum_x2 - sum_x * sum_x;
    if (std::abs(denominator) < std::numeric_limits<double>::epsilon()) {
      return false;
    }

    double slope_m = (n * sum_xy - sum_x * sum_y) / denominator;
    velocity = -1.0 * slope_m;  // negate value to estimate approach velocity
    return true;
  }

  void update(
    const double dist, const double dt, const double raw_vel_th, const double accel_th,
    const double min_dist_th)
  {
    // distance measurement is not reliable near the overlap point, skip velocity update
    if (is_reliable && delay_compensated_distance_to_overlap < min_dist_th) {
      update_history(dist, dt);
      track_duration += dt;
      return;
    }

    const auto raw_velocity = (distance_to_overlap - dist) / dt;
    if (abs(raw_velocity) > raw_vel_th) {
      reset();
      distance_history.push_back(dist);
      timestamp_history.push_back(0.0);
      return;
    }

    const auto last_vel = velocity;
    update_history(dist, dt);
    update_velocity();
    track_duration += dt;

    if (distance_history.size() < 3) return;

    const auto accel = (velocity - last_vel) / dt;
    if (abs(accel) > accel_th) {
      reset();
      distance_history.push_back(dist);
      timestamp_history.push_back(0.0);
    }
  }

  void reset()
  {
    velocity = 0.0;
    track_duration = 0.0;
    distance_history.clear();
    timestamp_history.clear();
    is_reliable = false;
    is_safe = true;
  }
};

struct DebugData
{
  EgoLanelets ego_lanelets;
  std::vector<TargetLanelet> target_lanelets;
  std::vector<PCDObject> pcd_objects;
  PointCloud2::SharedPtr cluster_points;
  PointCloud2::SharedPtr voxel_points;
  Direction turn_direction{Direction::NONE};
  double processing_time_detail_ms{0.0};
  bool is_active{false};
  bool is_safe{true};
  std::string text{"-"};
};

using PCDObjectsMap = std::unordered_map<lanelet::Id, PCDObject>;
using TargetLaneletsMap = std::unordered_map<lanelet::Id, TargetLanelet>;

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__TYPES_HPP_
