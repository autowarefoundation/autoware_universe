// Copyright 2020-2024 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE__OBSTACLE_COLLISION_CHECKER__OBSTACLE_COLLISION_CHECKER_HPP_
#define AUTOWARE__OBSTACLE_COLLISION_CHECKER__OBSTACLE_COLLISION_CHECKER_HPP_

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <string>
#include <vector>

namespace autoware::obstacle_collision_checker
{
using autoware_utils::LinearRing2d;

struct Param
{
  double delay_time;
  double footprint_margin;
  double max_deceleration;
  double resample_interval;
  double search_radius;
};

struct Input
{
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;
  geometry_msgs::msg::Twist::ConstSharedPtr current_twist;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr obstacle_pointcloud;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr obstacle_transform;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr reference_trajectory;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr predicted_trajectory;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
  Param param;
};

struct Output
{
  std::map<std::string, double> processing_time_map;
  bool will_collide;
  autoware_planning_msgs::msg::Trajectory resampled_trajectory;
  std::vector<LinearRing2d> vehicle_footprints;
  std::vector<LinearRing2d> vehicle_passing_areas;
};

Output check_for_collisions(const Input & input);

//! This function assumes the input trajectory is sampled dense enough
autoware_planning_msgs::msg::Trajectory resample_trajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double interval);

autoware_planning_msgs::msg::Trajectory cut_trajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double length);

std::vector<LinearRing2d> create_vehicle_footprints(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const Param & param,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);

std::vector<LinearRing2d> create_vehicle_passing_areas(
  const std::vector<LinearRing2d> & vehicle_footprints);

LinearRing2d create_hull_from_footprints(const LinearRing2d & area1, const LinearRing2d & area2);

bool will_collide(
  const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
  const std::vector<LinearRing2d> & vehicle_footprints);

bool has_collision(
  const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
  const LinearRing2d & vehicle_footprint);
}  // namespace autoware::obstacle_collision_checker

#endif  // AUTOWARE__OBSTACLE_COLLISION_CHECKER__OBSTACLE_COLLISION_CHECKER_HPP_
