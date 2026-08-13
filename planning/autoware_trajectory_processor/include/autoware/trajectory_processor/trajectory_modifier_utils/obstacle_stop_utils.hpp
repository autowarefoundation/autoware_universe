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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_UTILS__OBSTACLE_STOP_UTILS_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_UTILS__OBSTACLE_STOP_UTILS_HPP_

#include <autoware/object_recognition_utils/object_classification.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>

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

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
  PEDESTRIAN,
  ANIMAL,
  HAZARD
};

inline static const std::unordered_map<std::string, ObjectType> string_to_object_type = {
  {"unknown", ObjectType::UNKNOWN}, {"car", ObjectType::CAR},
  {"truck", ObjectType::TRUCK},     {"bus", ObjectType::BUS},
  {"trailer", ObjectType::TRAILER}, {"motorcycle", ObjectType::MOTORCYCLE},
  {"bicycle", ObjectType::BICYCLE}, {"pedestrian", ObjectType::PEDESTRIAN},
  {"animal", ObjectType::ANIMAL},   {"hazard", ObjectType::HAZARD}};

inline static const std::unordered_map<uint8_t, ObjectType> classification_to_object_type = {
  {ObjectClassification::UNKNOWN, ObjectType::UNKNOWN},
  {ObjectClassification::CAR, ObjectType::CAR},
  {ObjectClassification::TRUCK, ObjectType::TRUCK},
  {ObjectClassification::BUS, ObjectType::BUS},
  {ObjectClassification::TRAILER, ObjectType::TRAILER},
  {ObjectClassification::MOTORCYCLE, ObjectType::MOTORCYCLE},
  {ObjectClassification::BICYCLE, ObjectType::BICYCLE},
  {ObjectClassification::PEDESTRIAN, ObjectType::PEDESTRIAN},
  {ObjectClassification::ANIMAL, ObjectType::ANIMAL},
  {ObjectClassification::HAZARD, ObjectType::HAZARD}};

struct CollisionPoint
{
  geometry_msgs::msg::Point point;
  double arc_length;
  rclcpp::Time start_time;
  bool is_active{false};
  bool is_dynamic{false};

  /**
   * @brief Construct a collision sample from a point and its arc length along the reference path.
   * @param point Collision position in map frame.
   * @param arc_length Signed arc length from the start of the trajectory to the collision point.
   * @param is_dynamic Whether this collision is dynamic.
   */
  CollisionPoint(
    const geometry_msgs::msg::Point & point, const double arc_length, const bool is_dynamic = false)
  : point(point), arc_length(arc_length), is_dynamic(is_dynamic)
  {
  }

  /**
   * @brief Copy a collision point and attach timing / activation state (e.g. for hysteresis).
   * @param collision_point Source geometry and arc length.
   * @param start_time Time associated with this collision state.
   * @param active Whether this collision is currently considered active.
   */
  CollisionPoint(
    const CollisionPoint & collision_point, const rclcpp::Time & start_time, const bool active)
  : point(collision_point.point),
    arc_length(collision_point.arc_length),
    start_time(start_time),
    is_active(active),
    is_dynamic(collision_point.is_dynamic)
  {
  }
};

struct ObjectState
{
  double arc_length;
  double lon_vel;
  geometry_msgs::msg::Point nearest_point;
};

struct TrajectoryShape
{
  MultiPolygon2d polygon;
  autoware_utils_geometry::Box2d bounding_box;
  double trajectory_length;
  double forward_traj_length;
};

struct DebugData
{
  PointCloud2::SharedPtr cluster_points;
  PointCloud2::SharedPtr filtered_points;
  PredictedObjects filtered_objects;
  MultiPolygon2d target_polygons;
  TrajectoryShape trajectory_shape;
  std::vector<geometry_msgs::msg::Point> target_pcd_points;
  geometry_msgs::msg::Point active_collision_point;
  std::optional<PredictedObject> colliding_object;
  double ego_z = 0.0;  // cached for marker placement during publish_debug_data
};

/**
 * @brief Trim the trajectory after the first zero-velocity point and remove overlapping duplicate
 * points.
 * @param[in,out] trajectory_points Trajectory to shorten and deduplicate in place.
 */
void trim_trajectory_and_remove_duplicates(TrajectoryPoints & trajectory_points);

/**
 * @brief Build a 2D swept footprint of the ego vehicle along the portion of the path used for
 * obstacle detection.
 * @details The detection length combines stopping-distance logic (speed, deceleration, jerk,
 * margins) with the path ahead of the ego. The polygon is the union of vehicle footprints
 * sampled along that segment; the returned bounding box encloses that multi-polygon.
 * @param trajectory_points Full reference trajectory.
 * @param ego_pose Current ego pose on the trajectory.
 * @param vehicle_info Vehicle geometry for the footprint polygon.
 * @param ego_vel Current longitudinal speed [m/s].
 * @param ego_accel Current longitudinal acceleration [m/s^2].
 * @param decel Magnitude of comfortable deceleration used for stopping distance [m/s^2].
 * @param jerk Longitudinal jerk limit used for stopping distance [m/s^3].
 * @param stop_margin Extra longitudinal margin added to the detection range [m].
 * @param lateral_margin Lateral expansion of the footprint [m].
 * @param longitudinal_margin Longitudinal expansion of the footprint [m].
 * @return Swept shape, its axis-aligned bounding box, total trajectory length, and forward arc
 * length from the ego.
 */
TrajectoryShape get_trajectory_shape(
  const TrajectoryPoints & trajectory_points, const geometry_msgs::msg::Pose & ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double ego_vel,
  const double ego_accel, const double decel, const double jerk, const double stop_margin,
  const double lateral_margin = 0.0, const double longitudinal_margin = 0.0);

/**
 * @brief Find the closest point-cloud obstacle inside the trajectory swept area.
 * @details Points inside `trajectory_shape.polygon` are considered; the collision is the one with
 * minimum arc length along `trajectory_points`. All inlier points are appended to
 * `target_pcd_points` for visualization or debugging.
 * @param trajectory_points Reference path for arc-length queries.
 * @param trajectory_shape Swept ego region from get_trajectory_shape().
 * @param pointcloud Input points in the same frame as the trajectory.
 * @param[out] target_pcd_points Points that fell inside the trajectory polygon.
 * @return Nearest collision by arc length, or nullopt if the cloud is empty or none intersect.
 */
std::optional<CollisionPoint> get_nearest_pcd_collision(
  const TrajectoryPoints & trajectory_points, const TrajectoryShape & trajectory_shape,
  const PointCloud::Ptr & pointcloud, std::vector<geometry_msgs::msg::Point> & target_pcd_points);

/**
 * @brief Find the nearest obstacle along the path using each object's footprint polygon at the
 * current time.
 * @details For every target object, polygon vertices are projected onto arc length along the
 * trajectory; the minimum over all vertices and objects defines the collision point.
 * @param trajectory_points Reference path.
 * @param target_objects Predicted objects to test (typically already filtered).
 * @param[out] colliding_object Object that yielded the minimum arc-length collision.
 * @return Collision point and arc length, or nullopt if inputs are invalid or no objects.
 */
std::optional<CollisionPoint> get_nearest_object_collision(
  const TrajectoryPoints & trajectory_points, const PredictedObjects & target_objects,
  PredictedObject & colliding_object);

using ObjectDecelMap = std::unordered_map<ObjectType, double>;

/**
 * @brief Find the nearest predicted object that is not longitudinally safe relative to the ego
 * path within a time horizon.
 * @details For each trajectory point up to `lookahead_horizon`, an RSS check is applied to the
 * ego's and the object's predicted states to determine if the object is longitudinally safe
 * relative to the ego path. If the object is not longitudinally safe, the object is considered to
 * be a collision candidate. If the object's lon. velocity along ego path is below `stopped_vel_th`,
 * the object is considered to be a collision candidate. The object with the smallest arc length
 * along `trajectory_points` is returned as the collision point.
 * @param trajectory_points Reference path; arc lengths and ego motion are read from these points.
 * @param vehicle_info Used for the ego front longitudinal offset when comparing gap to safe
 * distance.
 * @param target_objects Predicted objects to evaluate (already filtered).
 * @param object_decel_map Braking magnitude [m/s^2] per object type for the object stopping term in
 * the safe-distance model.
 * @param ego_decel Magnitude of ego deceleration [m/s^2] for the ego stopping term.
 * @param reaction_time system reaction time [s] to respond to detected collision.
 * @param safety_margin Extra longitudinal buffer [m] added to the computed safe distance.
 * @param stopped_vel_th Objects with longitudinal speed along the path below this [m/s] are
 * considered static.
 * @param lookahead_horizon Maximum `time_from_start` along the trajectory [s] to propagate objects
 * and ego states.
 * @param[out] colliding_object Object with the smallest arc-length collision among unsafe cases.
 * @return Collision geometry and arc length, or nullopt if inputs are invalid or objects are safe
 */
std::optional<CollisionPoint> get_nearest_object_collision(
  const TrajectoryPoints & trajectory_points,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const PredictedObjects & target_objects, const ObjectDecelMap & object_decel_map,
  const double ego_decel, const double reaction_time, const double safety_margin,
  const double stopped_vel_th, const double lookahead_horizon, PredictedObject & colliding_object);

/// Filters predicted objects by semantic type, speed, and spatial relationship to the trajectory.
struct ObjectFilter
{
  /**
   * @brief Construct a filter from allowed type names and speed thresholds.
   * @param object_type_strings Allowed object classes (see string_to_object_type).
   * @param max_velocity_th Remove objects with longitudinal twist.x above this [m/s].
   * @param stopped_velocity_th Used when filtering by target area for "moving" vs stopped.
   * @param max_lateral_velocity_th Lateral speed threshold for the exiting-object heuristic [m/s].
   * @param safety_buffer Safety buffer to expand object shape [m].
   */
  ObjectFilter(
    const std::vector<std::string> & object_type_strings, const double max_velocity_th,
    const double stopped_velocity_th, const double max_lateral_velocity_th,
    const double safety_buffer)
  : max_velocity_th_(max_velocity_th),
    stopped_velocity_th_(stopped_velocity_th),
    max_lateral_velocity_th_(max_lateral_velocity_th),
    safety_buffer_(safety_buffer)
  {
    for (const auto & object_type_string : object_type_strings) {
      if (string_to_object_type.count(object_type_string) == 0) continue;
      object_types_.emplace(string_to_object_type.at(object_type_string));
    }
  }

  /**
   * @brief Remove objects that are too fast or whose class is not in the configured allow-list.
   * @param[in,out] objects Predicted objects message updated in place.
   */
  void filter_objects(PredictedObjects & objects)
  {
    objects.objects.erase(
      std::remove_if(
        objects.objects.begin(), objects.objects.end(),
        [&](const auto & object) {
          if (object.kinematics.initial_twist_with_covariance.twist.linear.x > max_velocity_th_)
            return true;
          const auto label =
            object.classification.empty()
              ? ObjectClassification::UNKNOWN
              : autoware::object_recognition_utils::getHighestProbLabel(object.classification);
          if (classification_to_object_type.count(label) == 0) return true;
          return object_types_.count(classification_to_object_type.at(label)) == 0;
        }),
      objects.objects.end());
  }

  /**
   * @brief Keep only objects that intersect the target region, dropping those leaving laterally.
   * @details Objects disjoint from `target_area` are removed. Moving objects that are judged to be
   * exiting the corridor (using lateral velocity and a short prediction horizon) are also removed.
   * Polygons of retained objects are accumulated in `target_polygons`.
   * @param[in,out] objects Predicted objects to filter in place.
   * @param trajectory_points Reference path for time and geometry queries.
   * @param target_area Multi-polygon region of interest (e.g. expanded path).
   * @param[out] target_polygons Footprints of objects that remain after filtering.
   */
  void filter_by_target_area(
    PredictedObjects & objects, const TrajectoryPoints & trajectory_points,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
    const MultiPolygon2d & target_area, MultiPolygon2d & target_polygons);

  /**
   * @brief Update allow-listed types and velocity thresholds without reconstructing the filter.
   */
  void set_params(
    const std::vector<std::string> & object_type_strings, const double max_velocity_th,
    const double stopped_velocity_th, const double max_lateral_velocity_th,
    const double safety_buffer)
  {
    object_types_.clear();
    for (const auto & object_type_string : object_type_strings) {
      if (string_to_object_type.count(object_type_string) == 0) continue;
      object_types_.emplace(string_to_object_type.at(object_type_string));
    }
    max_velocity_th_ = max_velocity_th;
    stopped_velocity_th_ = stopped_velocity_th;
    max_lateral_velocity_th_ = max_lateral_velocity_th;
    safety_buffer_ = safety_buffer;
  }

private:
  std::unordered_set<ObjectType> object_types_;
  double max_velocity_th_;
  double stopped_velocity_th_;
  double max_lateral_velocity_th_;
  double safety_buffer_;
};

/// PCL-based downsampling, cropping, clustering, and object masking for obstacle point clouds.
struct PointCloudFilter
{
  /**
   * @brief Configure voxel grid and euclidean clustering parameters used by subsequent filters.
   */
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

  /**
   * @brief Update voxel and clustering parameters at runtime.
   */
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

  /**
   * @brief Crop the cloud to an axis-aligned box, then apply voxel grid downsampling.
   * @param[in,out] pointcloud Cloud updated in place; empty if nothing remains.
   * @param min_x,max_x,min_y,max_y,min_z,max_z Crop box bounds in the cloud frame.
   */
  void filter_pointcloud(
    PointCloud::Ptr & pointcloud, const double min_x, const double max_x, const double min_y,
    const double max_y, const double min_z, const double max_z);
  /**
   * @brief Cluster the cloud and output 2D convex hull vertices of clusters above a height cutoff.
   * @param input Downsampled or cropped cloud.
   * @param[out] output Hull vertices of qualifying clusters (typically for footprint tests).
   * @param min_height A cluster is kept only if at least one point has z >= this value [m].
   */
  void cluster_pointcloud(
    const PointCloud::Ptr & input, PointCloud::Ptr & output, const double min_height);
  /**
   * @brief Remove points that lie inside predicted object footprints (expanded by a small margin).
   * @param[in,out] pointcloud Cloud to strip in place.
   * @param objects Predicted obstacles whose shapes define removal regions.
   */
  void filter_pointcloud_by_object(PointCloud::Ptr & pointcloud, const PredictedObjects & objects);

private:
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;
  pcl::CropBox<pcl::PointXYZ> crop_box_;
  pcl::ConvexHull<pcl::PointXYZ> convex_hull_;
};

/// Temporal association of obstacle detections (objects and points) with hysteresis.
struct ObstacleTracker
{
  /**
   * @brief Construct with time buffers and association thresholds.
   * @param on_time_buffer Seconds a track must be seen before it is considered active.
   * @param off_time_buffer Seconds without observation before an active track may be removed.
   * @param object_distance_th Max 2D distance to match a new detection to an existing object track.
   * @param object_yaw_th Max yaw difference to match object detections [rad].
   * @param pcd_distance_th Max 2D distance to match a point to an existing point track.
   * @param grace_period Seconds to keep inactive tracks before deletion.
   */
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

  /**
   * @brief Update association and timing parameters.
   */
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
   * @param now Current stamp used for track aging and activation timing
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
   * @param now Current stamp used for track aging and activation timing
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

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_UTILS__OBSTACLE_STOP_UTILS_HPP_
