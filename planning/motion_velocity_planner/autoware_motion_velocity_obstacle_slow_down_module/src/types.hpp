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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include "type_alias.hpp"

#include <algorithm>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{

struct SlowDownPointData
{
  std::optional<geometry_msgs::msg::Point> front{std::nullopt};
  std::optional<geometry_msgs::msg::Point> back{std::nullopt};

  double lat_dist_to_traj{0};

  SlowDownPointData(
    const std::optional<geometry_msgs::msg::Point> & front_point,
    const std::optional<geometry_msgs::msg::Point> & back_point, double lat_dist_to_traj)
  : front(front_point), back(back_point), lat_dist_to_traj(lat_dist_to_traj)
  {
  }
};
struct SlowDownObstacle
{
  SlowDownObstacle(
    const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
    const ObjectClassification & object_classification, const geometry_msgs::msg::Pose & arg_pose,
    const double arg_lon_velocity, const double arg_lat_velocity,
    const double arg_dist_to_traj_poly, const geometry_msgs::msg::Point & arg_front_collision_point,
    const geometry_msgs::msg::Point & arg_back_collision_point)
  : uuid(arg_uuid),
    stamp(arg_stamp),
    pose(arg_pose),
    velocity(arg_lon_velocity),
    lat_velocity(arg_lat_velocity),
    dist_to_traj_poly(arg_dist_to_traj_poly),
    front_collision_point(arg_front_collision_point),
    back_collision_point(arg_back_collision_point),
    classification(object_classification)
  {
  }
  std::string uuid{};
  rclcpp::Time stamp{};
  geometry_msgs::msg::Pose pose{};  // interpolated with the current stamp
  double velocity{};                // longitudinal velocity against ego's trajectory
  double lat_velocity{};            // lateral velocity against ego's trajectory

  double dist_to_traj_poly{};  // for efficient calculation
  geometry_msgs::msg::Point front_collision_point{};
  geometry_msgs::msg::Point back_collision_point{};
  ObjectClassification classification{};
};

struct SlowDownOutput
{
  SlowDownOutput() = default;
  SlowDownOutput(
    const std::string & arg_uuid, const std::vector<TrajectoryPoint> & traj_points,
    const std::optional<size_t> & start_idx, const std::optional<size_t> & end_idx,
    const double arg_target_vel, const double arg_feasible_target_vel,
    const double arg_dist_from_obj_poly_to_traj_poly, const bool is_obstacle_moving)
  : uuid(arg_uuid),
    target_vel(arg_target_vel),
    feasible_target_vel(arg_feasible_target_vel),
    dist_from_obj_poly_to_traj_poly(arg_dist_from_obj_poly_to_traj_poly),
    is_obstacle_moving(is_obstacle_moving)
  {
    if (start_idx) {
      start_point = traj_points.at(*start_idx).pose;
    }
    if (end_idx) {
      end_point = traj_points.at(*end_idx).pose;
    }
  }

  std::string uuid{};
  double target_vel{};
  double feasible_target_vel{};
  double dist_from_obj_poly_to_traj_poly{};
  std::optional<geometry_msgs::msg::Pose> start_point{std::nullopt};
  std::optional<geometry_msgs::msg::Pose> end_point{std::nullopt};
  bool is_obstacle_moving{};
};

struct SlowDownConditionCounter
{
  void reset_current_uuids() { current_uuids_.clear(); }
  void add_current_uuid(const std::string & uuid) { current_uuids_.push_back(uuid); }
  void remove_counter_unless_updated()
  {
    std::vector<std::string> obsolete_uuids;
    for (const auto & key_and_value : counter_) {
      if (
        std::find(current_uuids_.begin(), current_uuids_.end(), key_and_value.first) ==
        current_uuids_.end()) {
        obsolete_uuids.push_back(key_and_value.first);
      }
    }

    for (const auto & obsolete_uuid : obsolete_uuids) {
      counter_.erase(obsolete_uuid);
    }
  }

  int increase_counter(const std::string & uuid)
  {
    if (counter_.count(uuid) != 0) {
      counter_.at(uuid) = std::max(1, counter_.at(uuid) + 1);
    } else {
      counter_.emplace(uuid, 1);
    }
    return counter_.at(uuid);
  }
  int decrease_counter(const std::string & uuid)
  {
    if (counter_.count(uuid) != 0) {
      counter_.at(uuid) = std::min(-1, counter_.at(uuid) - 1);
    } else {
      counter_.emplace(uuid, -1);
    }
    return counter_.at(uuid);
  }
  void reset(const std::string & uuid) { counter_.emplace(uuid, 0); }

  // NOTE: positive is for meeting entering condition, and negative is for exiting.
  std::unordered_map<std::string, int> counter_{};
  std::vector<std::string> current_uuids_{};
};

struct DebugData
{
  DebugData() = default;
  std::vector<SlowDownObstacle> obstacles_to_slow_down{};
  std::vector<Polygon2d> decimated_traj_polys{};
  MarkerArray slow_down_debug_wall_marker{};
  MarkerArray slow_down_wall_marker{};
};
}  // namespace autoware::motion_velocity_planner

#endif  // TYPES_HPP_
