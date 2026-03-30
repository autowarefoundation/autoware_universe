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

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_hash.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_modifier::utils::obstacle_stop
{
using sensor_msgs::msg::PointCloud2;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_utils_geometry::MultiPolygon2d;

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

struct TrajectoryShape
{
  MultiPolygon2d polygon;
  autoware_utils_geometry::Box2d bounding_box;
};

struct DebugData
{
  PointCloud2::SharedPtr cluster_points;
  PointCloud2::SharedPtr voxel_points;
  MultiPolygon2d target_polygons;
  TrajectoryShape trajectory_shape;
  std::vector<geometry_msgs::msg::Point> target_pcd_points;
  geometry_msgs::msg::Point active_collision_point;
  std::optional<PredictedObject> colliding_object;
};

TrajectoryShape get_trajectory_shape(
  const TrajectoryPoints & trajectory_points, const geometry_msgs::msg::Pose & ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, double trim_length,
  const double lateral_margin = 0.0, const double longitudinal_margin = 0.0);

void filter_objects_by_type(
  PredictedObjects & objects, const std::vector<std::string> & object_type_strings);

void filter_objects_by_velocity(PredictedObjects & objects, const double max_velocity);

std::optional<CollisionPoint> get_nearest_pcd_collision(
  const TrajectoryPoints & trajectory_points, const MultiPolygon2d & trajectory_polygon,
  const PointCloud::Ptr & pointcloud, std::vector<geometry_msgs::msg::Point> & target_pcd_points);

std::optional<CollisionPoint> get_nearest_object_collision(
  const TrajectoryPoints & trajectory_points, const MultiPolygon2d & trajectory_polygon,
  const PredictedObjects & objects, MultiPolygon2d & target_polygons,
  PredictedObject & colliding_object);

struct PointCloudFilter
{
  PointCloudFilter(
    double voxel_size_x, double voxel_size_y, double voxel_size_z, int voxel_min_size,
    double cluster_tolerance, int cluster_min_size, int cluster_max_size)
  {
    tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    ec_.setClusterTolerance(cluster_tolerance);
    ec_.setMinClusterSize(cluster_min_size);
    ec_.setMaxClusterSize(cluster_max_size);
    voxel_grid_.setLeafSize(voxel_size_x, voxel_size_y, voxel_size_z);
    voxel_grid_.setMinimumPointsNumberPerVoxel(voxel_min_size);
    convex_hull_.setDimension(2);
  };

  void set_params(
    double voxel_size_x, double voxel_size_y, double voxel_size_z, int voxel_min_size,
    double cluster_tolerance, int cluster_min_size, int cluster_max_size)
  {
    voxel_grid_.setLeafSize(voxel_size_x, voxel_size_y, voxel_size_z);
    voxel_grid_.setMinimumPointsNumberPerVoxel(voxel_min_size);
    ec_.setClusterTolerance(cluster_tolerance);
    ec_.setMinClusterSize(cluster_min_size);
    ec_.setMaxClusterSize(cluster_max_size);
  }

  void filter_pointcloud(
    PointCloud::Ptr & pointcloud, const double min_x, const double max_x, const double min_y,
    const double max_y, const double min_z, const double max_z);
  void cluster_pointcloud(
    const PointCloud::Ptr & input, PointCloud::Ptr & output, const double min_height);
  void filter_pointcloud_by_object(PointCloud::Ptr & pointcloud, const PredictedObjects & objects);

private:
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;
  pcl::CropBox<pcl::PointXYZ> crop_box_;
  pcl::ConvexHull<pcl::PointXYZ> convex_hull_;
};

struct ObstacleTracker
{
  ObstacleTracker(
    const double on_time_buffer, const double off_time_buffer, const double object_distance_th,
    const double object_yaw_th, const double pcd_distance_th, const double grace_period)
  : on_time_buffer_(on_time_buffer),
    off_time_buffer_(off_time_buffer),
    object_distance_th_(object_distance_th),
    object_yaw_th_(object_yaw_th),
    pcd_distance_th_(pcd_distance_th),
    grace_period_(grace_period)
  {
  }

  void set_params(
    const double on_time_buffer, const double off_time_buffer, const double object_distance_th,
    const double object_yaw_th, const double pcd_distance_th, const double grace_period)
  {
    on_time_buffer_ = on_time_buffer;
    off_time_buffer_ = off_time_buffer;
    object_distance_th_ = object_distance_th;
    object_yaw_th_ = object_yaw_th;
    pcd_distance_th_ = pcd_distance_th;
    grace_period_ = grace_period;
  }

  /**
   * @brief Update tracked objects
   * @details Use input objects to update the tracked objects history and remove obsolete objects,
   * based on the on_time_buffer and off_time_buffer.
   * @param objects Input predicted objects after filtering
   * @param persistent_objects Output persistent objects
   */
  void update_objects(
    const PredictedObjects & objects, PredictedObjects & persistent_objects,
    const rclcpp::Time & now);

  /**
   * @brief Update tracked points
   * @details Use input pointcloud to update the tracked points history and remove obsolete points,
   * based on the on_time_buffer and off_time_buffer.
   * @param points Input pointcloud
   * @param persistent_points Output persistent points
   */
  void update_points(
    const PointCloud::Ptr & points, PointCloud::Ptr & persistent_points, const rclcpp::Time & now);

private:
  double on_time_buffer_;
  double off_time_buffer_;
  double object_distance_th_;
  double object_yaw_th_;
  double pcd_distance_th_;
  double grace_period_;

  struct PersistentObstacle
  {
    rclcpp::Time first_seen_time;
    rclcpp::Time last_seen_time;
    bool is_active{false};

    explicit PersistentObstacle(const rclcpp::Time & now)
    : first_seen_time(now), last_seen_time(now)
    {
    }
  };

  struct PersistentObject : public PersistentObstacle
  {
    PredictedObject object;

    explicit PersistentObject(const PredictedObject & object, const rclcpp::Time & now)
    : PersistentObstacle(now), object(object)
    {
    }
  };
  std::unordered_map<boost::uuids::uuid, PersistentObject, boost::hash<boost::uuids::uuid>>
    persistent_objects_map_;

  struct PersistentPoint : public PersistentObstacle
  {
    geometry_msgs::msg::Point position;
    explicit PersistentPoint(const geometry_msgs::msg::Point & position, const rclcpp::Time & now)
    : PersistentObstacle(now), position(position)
    {
    }
  };
  std::unordered_map<boost::uuids::uuid, PersistentPoint, boost::hash<boost::uuids::uuid>>
    persistent_point_map_;

  boost::uuids::random_generator id_generator_;
};

}  // namespace autoware::trajectory_modifier::utils::obstacle_stop

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__OBSTACLE_STOP_UTILS_HPP_
