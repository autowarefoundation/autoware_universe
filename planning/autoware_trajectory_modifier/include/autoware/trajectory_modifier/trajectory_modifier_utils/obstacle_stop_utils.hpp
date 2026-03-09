// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__OBSTACLE_STOP_UTILS_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__OBSTACLE_STOP_UTILS_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_modifier::utils::obstacle_stop
{
using sensor_msgs::msg::PointCloud2;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

enum class ObjectType : uint8_t {
  UNKNOWN = 0,
  CAR,
  TRUCK,
  BUS,
  TRAILER,
  MOTORCYCLE,
  BICYCLE,
  PEDESTRIAN
};

inline static const std::unordered_map<std::string, ObjectType> string_to_object_type = {
  {"unknown", ObjectType::UNKNOWN}, {"car", ObjectType::CAR},
  {"truck", ObjectType::TRUCK},     {"bus", ObjectType::BUS},
  {"trailer", ObjectType::TRAILER}, {"motorcycle", ObjectType::MOTORCYCLE},
  {"bicycle", ObjectType::BICYCLE}, {"pedestrian", ObjectType::PEDESTRIAN}};

inline static const std::unordered_map<uint8_t, ObjectType> classification_to_object_type = {
  {ObjectClassification::UNKNOWN, ObjectType::UNKNOWN},
  {ObjectClassification::CAR, ObjectType::CAR},
  {ObjectClassification::TRUCK, ObjectType::TRUCK},
  {ObjectClassification::BUS, ObjectType::BUS},
  {ObjectClassification::TRAILER, ObjectType::TRAILER},
  {ObjectClassification::MOTORCYCLE, ObjectType::MOTORCYCLE},
  {ObjectClassification::BICYCLE, ObjectType::BICYCLE},
  {ObjectClassification::PEDESTRIAN, ObjectType::PEDESTRIAN}};

struct CollisionPoint
{
  geometry_msgs::msg::Point point;
  double arc_length;
  rclcpp::Time start_time;
  bool is_active{false};

  CollisionPoint(const geometry_msgs::msg::Point & point, const double arc_length)
  : point(point), arc_length(arc_length)
  {
  }

  CollisionPoint(
    const CollisionPoint & collision_point, const rclcpp::Time & start_time, const bool active)
  : point(collision_point.point),
    arc_length(collision_point.arc_length),
    start_time(start_time),
    is_active(active)
  {
  }
};

struct DebugData
{
  PointCloud2::SharedPtr cluster_points;
  PointCloud2::SharedPtr voxel_points;
  PointCloud2::SharedPtr target_pcd_points;
  std::vector<autoware_utils_geometry::Polygon2d> target_polygons;
  autoware_utils_geometry::MultiPolygon2d trajectory_polygon;
  std::vector<geometry_msgs::msg::Point> target_collision_points;
  geometry_msgs::msg::Point active_collision_point;
};

autoware_utils_geometry::MultiPolygon2d get_trajectory_polygon(
  const TrajectoryPoints & trajectory_points, const geometry_msgs::msg::Pose & ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double lateral_margin = 0.0,
  const double longitudinal_margin = 0.0);

void filter_pointcloud_by_range(
  PointCloud::Ptr & pointcloud, const double detection_range, const double min_height,
  const double max_height);

void filter_pointcloud_by_voxel_grid(
  PointCloud::Ptr & pointcloud, const double voxel_size_x, const double voxel_size_y,
  const double voxel_size_z, const int min_size);

void cluster_pointcloud(
  PointCloud::Ptr & input, PointCloud::Ptr & output, const size_t min_size, const size_t max_size,
  const double tolerance, const double min_height, const double height_offset);

void filter_objects_by_type(
  PredictedObjects & objects, const std::vector<std::string> & object_type_strings);

void filter_objects_by_velocity(PredictedObjects & objects, const double max_velocity);

std::optional<CollisionPoint> get_nearest_pcd_collision(
  const TrajectoryPoints & trajectory_points,
  const autoware_utils_geometry::MultiPolygon2d & trajectory_polygon,
  const PointCloud::Ptr & pointcloud);

std::optional<CollisionPoint> get_nearest_object_collision(
  const TrajectoryPoints & trajectory_points,
  const autoware_utils_geometry::MultiPolygon2d & trajectory_polygon,
  const PredictedObjects & objects);

}  // namespace autoware::trajectory_modifier::utils::obstacle_stop

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__OBSTACLE_STOP_UTILS_HPP_
