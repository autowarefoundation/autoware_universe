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

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <algorithm>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner
{

struct StopObstacle
{
  StopObstacle(
    const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
    const ObjectClassification & object_classification, const geometry_msgs::msg::Pose & arg_pose,
    const Shape & arg_shape, const double arg_lon_velocity,
    const geometry_msgs::msg::Point & arg_collision_point,
    const double arg_dist_to_collide_on_decimated_traj,
    const std::optional<double> arg_braking_dist = std::nullopt)
  : uuid(arg_uuid),
    stamp(arg_stamp),
    pose(arg_pose),
    velocity(arg_lon_velocity),
    shape(arg_shape),
    collision_point(arg_collision_point),
    dist_to_collide_on_decimated_traj(arg_dist_to_collide_on_decimated_traj),
    classification(object_classification),
    braking_dist(arg_braking_dist)
  {
  }
  std::string uuid;
  rclcpp::Time stamp;
  geometry_msgs::msg::Pose pose;  // interpolated with the current stamp
  double velocity;                // longitudinal velocity against ego's trajectory

  Shape shape;
  geometry_msgs::msg::Point collision_point;
  double dist_to_collide_on_decimated_traj;
  ObjectClassification classification;
  std::optional<double> braking_dist;

  // additional fields for road user stop module
  bool is_wrong_way = false;
  PredictedObject original_object;  // keep original object for reference
};

struct StopPointCandidate
{
  size_t stop_index;
  geometry_msgs::msg::Point stop_position;
  double stop_distance;  // distance from ego vehicle
  autoware_perception_msgs::msg::PredictedObject target_object;
  bool is_wrong_way;
  double required_deceleration;
};

// 使用しないようにする
struct TrackedObject
{
  unique_identifier_msgs::msg::UUID object_id;
  rclcpp::Time first_detection_time;
  rclcpp::Time last_detection_time;
  geometry_msgs::msg::Point last_position;
  std::optional<StopPointCandidate>
    wrong_way_stop_candidate;  // cached stop point for wrong-way users
};
}  // namespace autoware::motion_velocity_planner

#endif  // TYPES_HPP_
