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

#include "autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <range/v3/view.hpp>

#include <boost/geometry.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier::utils::obstacle_stop
{

TrajectoryShape get_trajectory_shape(
  const TrajectoryPoints & trajectory_points, const geometry_msgs::msg::Pose & ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double lateral_margin,
  const double longitudinal_margin)
{
  const auto offset_pose =
    autoware_utils::calc_offset_pose(ego_pose, vehicle_info.max_longitudinal_offset_m, 0, 0);
  auto start_idx = motion_utils::findNearestSegmentIndex(trajectory_points, offset_pose.position);
  const auto forward_traj_points =
    TrajectoryPoints(trajectory_points.begin() + start_idx, trajectory_points.end());

  autoware_utils_geometry::LineString2d ls_front_right;
  autoware_utils_geometry::LineString2d ls_front_left;
  autoware_utils_geometry::LineString2d ls_rear_right;
  autoware_utils_geometry::LineString2d ls_rear_left;
  ls_front_right.reserve(forward_traj_points.size());
  ls_front_left.reserve(forward_traj_points.size());
  ls_rear_right.reserve(forward_traj_points.size());
  ls_rear_left.reserve(forward_traj_points.size());

  autoware_utils_geometry::Polygon2d polygon_front;
  autoware_utils_geometry::Polygon2d polygon_rear;

  constexpr double min_resolution = 0.1;

  const auto base_footprint = vehicle_info.createFootprint(lateral_margin, longitudinal_margin);
  for (const auto & [idx, p] : forward_traj_points | ranges::views::enumerate) {
    if (idx > 0) {
      const auto prev_p = forward_traj_points[idx - 1];
      const auto dist = autoware_utils::calc_distance2d(prev_p, p);
      if (dist < min_resolution) continue;
    }
    const autoware_utils_geometry::Point2d base_link(p.pose.position.x, p.pose.position.y);
    const auto angle = tf2::getYaw(p.pose.orientation);
    const Eigen::Rotation2Dd rotation(angle);
    const auto front_left_offset =
      rotation * base_footprint[vehicle_info_utils::VehicleInfo::FrontLeftIndex];
    const auto front_right_offset =
      rotation * base_footprint[vehicle_info_utils::VehicleInfo::FrontRightIndex];
    const auto rear_right_offset =
      rotation * base_footprint[vehicle_info_utils::VehicleInfo::RearRightIndex];
    const auto rear_left_offset =
      rotation * base_footprint[vehicle_info_utils::VehicleInfo::RearLeftIndex];
    ls_front_left.emplace_back(
      base_link.x() + front_left_offset.x(), base_link.y() + front_left_offset.y());
    ls_front_right.emplace_back(
      base_link.x() + front_right_offset.x(), base_link.y() + front_right_offset.y());
    ls_rear_right.emplace_back(
      base_link.x() + rear_right_offset.x(), base_link.y() + rear_right_offset.y());
    ls_rear_left.emplace_back(
      base_link.x() + rear_left_offset.x(), base_link.y() + rear_left_offset.y());
  }

  boost::geometry::reverse(ls_front_right);
  boost::geometry::reverse(ls_rear_right);

  boost::geometry::append(polygon_front, ls_front_left);
  boost::geometry::append(polygon_front, ls_front_right);
  boost::geometry::append(polygon_rear, ls_rear_left);
  boost::geometry::append(polygon_rear, ls_rear_right);

  boost::geometry::correct(polygon_front);
  boost::geometry::correct(polygon_rear);

  autoware_utils_geometry::MultiPolygon2d trajectory_polygon;
  boost::geometry::union_(polygon_front, polygon_rear, trajectory_polygon);

  autoware_utils_geometry::Box2d envelope;
  boost::geometry::envelope(trajectory_polygon, envelope);

  return TrajectoryShape{trajectory_polygon, envelope};
}

void filter_objects_by_type(
  PredictedObjects & objects, const std::vector<std::string> & object_type_strings)
{
  const std::vector<ObjectType> object_types = std::invoke([&]() {
    std::vector<ObjectType> object_types;
    object_types.reserve(object_type_strings.size());
    for (const auto & object_type_string : object_type_strings) {
      object_types.push_back(string_to_object_type.at(object_type_string));
    }
    return object_types;
  });

  objects.objects.erase(
    std::remove_if(
      objects.objects.begin(), objects.objects.end(),
      [&](const auto & object) {
        return std::find(
                 object_types.begin(), object_types.end(),
                 classification_to_object_type.at(object.classification.front().label)) ==
               object_types.end();
      }),
    objects.objects.end());
}

void filter_objects_by_velocity(PredictedObjects & objects, const double max_velocity)
{
  objects.objects.erase(
    std::remove_if(
      objects.objects.begin(), objects.objects.end(),
      [&](const auto & object) {
        return object.kinematics.initial_twist_with_covariance.twist.linear.x > max_velocity;
      }),
    objects.objects.end());
}

std::optional<CollisionPoint> get_nearest_pcd_collision(
  const TrajectoryPoints & trajectory_points,
  const autoware_utils_geometry::MultiPolygon2d & trajectory_polygon,
  const PointCloud::Ptr & pointcloud, std::vector<geometry_msgs::msg::Point> & target_pcd_points)
{
  if (pointcloud->empty()) return std::nullopt;

  PointCloud::Ptr pointcloud_in_polygon(new PointCloud);
  for (const auto & point : *pointcloud) {
    if (boost::geometry::within(autoware_utils::Point2d{point.x, point.y}, trajectory_polygon)) {
      pointcloud_in_polygon->push_back(point);
    }
  }

  if (pointcloud_in_polygon->empty()) return std::nullopt;

  auto min_arc_length = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point nearest_collision_point;
  for (const auto & point : *pointcloud_in_polygon) {
    geometry_msgs::msg::Point p;
    p.x = point.x;
    p.y = point.y;
    const auto arc_length = motion_utils::calcSignedArcLength(trajectory_points, 0, p);
    if (arc_length < min_arc_length) {
      min_arc_length = arc_length;
      nearest_collision_point = p;
    }
    target_pcd_points.emplace_back(p);
  }

  return CollisionPoint(nearest_collision_point, min_arc_length);
}

std::optional<CollisionPoint> get_nearest_object_collision(
  const TrajectoryPoints & trajectory_points,
  const autoware_utils_geometry::MultiPolygon2d & trajectory_polygon,
  const PredictedObjects & objects, MultiPolygon2d & target_polygons,
  PredictedObject & colliding_object)
{
  if (objects.objects.empty()) return std::nullopt;

  auto min_arc_length = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point nearest_collision_point;
  bool found_collision = false;
  for (const auto & object : objects.objects) {
    const auto object_pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto object_polygon = autoware_utils::to_polygon2d(object_pose, object.shape);
    if (boost::geometry::disjoint(object_polygon, trajectory_polygon)) {
      continue;
    }
    found_collision = true;
    for (const auto & point : object_polygon.outer()) {
      geometry_msgs::msg::Point p = geometry_msgs::msg::Point().set__x(point.x()).set__y(point.y());
      const auto arc_length = motion_utils::calcSignedArcLength(trajectory_points, 0, p);
      if (arc_length < min_arc_length) {
        min_arc_length = arc_length;
        nearest_collision_point = p;
        colliding_object = object;
      }
    }
    target_polygons.emplace_back(object_polygon);
  }

  if (!found_collision) return std::nullopt;
  return CollisionPoint(nearest_collision_point, min_arc_length);
}

void PointCloudFilter::filter_pointcloud(
  PointCloud::Ptr & pointcloud, const double min_x, const double max_x, const double min_y,
  const double max_y, const double min_z, const double max_z)
{
  if (pointcloud->empty()) return;

  crop_box_.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
  crop_box_.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
  crop_box_.setInputCloud(pointcloud);
  crop_box_.filter(*pointcloud);

  if (pointcloud->empty()) return;

  voxel_grid_.setInputCloud(pointcloud);
  voxel_grid_.filter(*pointcloud);
}

void PointCloudFilter::cluster_pointcloud(
  const PointCloud::Ptr & input, PointCloud::Ptr & output, const double min_height)
{
  if (input->empty()) return;

  std::vector<pcl::PointIndices> cluster_indices;
  tree_->setInputCloud(input);
  ec_.setSearchMethod(tree_);
  ec_.setInputCloud(input);
  ec_.extract(cluster_indices);

  PointCloud::Ptr cluster_buffer(new PointCloud);
  PointCloud::Ptr hull_buffer(new PointCloud);

  for (const auto & indices : cluster_indices) {
    cluster_buffer->clear();
    hull_buffer->clear();
    bool cluster_above_height_threshold{false};
    for (const auto & index : indices.indices) {
      const auto & point = (*input)[index];

      cluster_above_height_threshold |= point.z >= min_height;
      cluster_buffer->push_back(point);
    }
    if (!cluster_above_height_threshold || cluster_buffer->empty()) continue;

    convex_hull_.setInputCloud(cluster_buffer);
    convex_hull_.reconstruct(*hull_buffer);
    for (const auto & point : *hull_buffer) {
      output->push_back(point);
    }
  }
}

void PointCloudFilter::filter_pointcloud_by_object(
  PointCloud::Ptr & pointcloud, const PredictedObjects & objects)
{
  if (pointcloud->empty() || objects.objects.empty()) return;

  const auto margin = 0.1;
  auto get_object_polygon = [&](const auto & object) {
    auto polygon = autoware_utils::to_polygon2d(object);
    auto expanded_polygon = autoware_utils::expand_polygon(polygon, margin);
    return expanded_polygon;
  };

  for (const auto & obj : objects.objects) {
    const auto obj_pose = obj.kinematics.initial_pose_with_covariance.pose;
    std::optional<autoware_utils_geometry::Polygon2d> obj_polygon;
    const auto x_th = obj.shape.dimensions.x / 2.0 + margin;
    const auto y_th = obj.shape.dimensions.y / 2.0 + margin;
    pointcloud->erase(
      std::remove_if(
        pointcloud->begin(), pointcloud->end(),
        [&](const auto & point) {
          const autoware_utils_geometry::Point2d p(point.x, point.y);
          const auto rel_p = autoware_utils::inverse_transform_point(p.to_3d(), obj_pose);
          if (abs(rel_p.x()) > x_th || abs(rel_p.y()) > y_th) {
            return false;
          }
          if (!obj_polygon) obj_polygon = get_object_polygon(obj);
          return !boost::geometry::disjoint(p, *obj_polygon);
        }),
      pointcloud->end());
  }
}

void ObstacleTracker::update_objects(
  const PredictedObjects & objects, PredictedObjects & persistent_objects)
{
  auto now = std::chrono::system_clock::now();
  using Seconds = std::chrono::duration<double>;

  constexpr double distance_threshold = 1.0;
  constexpr double grace_period = 0.2;  // allow grace period for flickering objects
  auto closest_object_uuid =
    [&](const PredictedObject & object) -> std::optional<boost::uuids::uuid> {
    std::optional<boost::uuids::uuid> closest_uuid = std::nullopt;
    if (persistent_objects_map_.empty()) return std::nullopt;
    double min_distance = std::numeric_limits<double>::max();
    for (const auto & [uuid, existing_object] : persistent_objects_map_) {
      if (
        existing_object.object.classification.front().label != object.classification.front().label)
        continue;
      const auto distance = autoware_utils::calc_distance2d(
        object.kinematics.initial_pose_with_covariance.pose.position,
        existing_object.object.kinematics.initial_pose_with_covariance.pose.position);
      if (distance < min_distance) {
        min_distance = distance;
        closest_uuid = uuid;
      }
    }
    if (closest_uuid && min_distance > distance_threshold) {
      closest_uuid = std::nullopt;
    }
    return closest_uuid;
  };

  for (const auto & object : objects.objects) {
    const auto closest_uuid = closest_object_uuid(object);
    if (!closest_uuid) {
      persistent_objects_map_.emplace(id_generator_(), PersistentObject(object, now));
      continue;
    }
    auto & closest_object = persistent_objects_map_.at(closest_uuid.value());
    closest_object.last_seen_time = now;
    closest_object.object = object;
    const auto duration = std::chrono::duration_cast<Seconds>(now - closest_object.first_seen_time);
    closest_object.is_active = duration.count() >= on_time_buffer_;
  }

  for (auto it = persistent_objects_map_.begin(); it != persistent_objects_map_.end();) {
    const auto idle_time =
      std::chrono::duration_cast<Seconds>(now - it->second.last_seen_time).count();
    const bool is_erase =
      !it->second.is_active ? idle_time > grace_period : idle_time > off_time_buffer_;
    if (is_erase) {
      it = persistent_objects_map_.erase(it);
    } else {
      if (it->second.is_active) {
        persistent_objects.objects.push_back(it->second.object);
      }
      it++;
    }
  }
}

void ObstacleTracker::update_points(
  const PointCloud::Ptr & points, PointCloud::Ptr & persistent_points)
{
  auto now = std::chrono::system_clock::now();
  using Seconds = std::chrono::duration<double>;

  constexpr double distance_threshold = 0.5;
  constexpr double grace_period = 0.2;  // allow grace period for flickering points
  auto closest_point_uuid =
    [&](const geometry_msgs::msg::Point & point) -> std::optional<boost::uuids::uuid> {
    std::optional<boost::uuids::uuid> closest_uuid = std::nullopt;
    if (persistent_point_map_.empty()) return std::nullopt;
    double min_distance = std::numeric_limits<double>::max();
    for (const auto & [uuid, existing_point] : persistent_point_map_) {
      const auto distance = autoware_utils::calc_distance2d(point, existing_point.position);
      if (distance < min_distance) {
        min_distance = distance;
        closest_uuid = uuid;
      }
    }
    if (closest_uuid && min_distance > distance_threshold) {
      closest_uuid = std::nullopt;
    }
    return closest_uuid;
  };

  for (const auto & point : points->points) {
    auto point_msg = autoware_utils::create_point(point.x, point.y, point.z);
    const auto closest_uuid = closest_point_uuid(point_msg);
    if (!closest_uuid) {
      persistent_point_map_.emplace(id_generator_(), PersistentPoint(point_msg, now));
      continue;
    }
    auto & closest_point = persistent_point_map_.at(closest_uuid.value());
    closest_point.last_seen_time = now;
    closest_point.position = point_msg;
    const auto duration = std::chrono::duration_cast<Seconds>(now - closest_point.first_seen_time);
    closest_point.is_active = duration.count() >= on_time_buffer_;
  }

  for (auto it = persistent_point_map_.begin(); it != persistent_point_map_.end();) {
    const auto idle_time =
      std::chrono::duration_cast<Seconds>(now - it->second.last_seen_time).count();
    const bool is_erase =
      !it->second.is_active ? idle_time > grace_period : idle_time > off_time_buffer_;
    if (is_erase) {
      it = persistent_point_map_.erase(it);
    } else {
      if (it->second.is_active) {
        persistent_points->points.emplace_back(
          it->second.position.x, it->second.position.y, it->second.position.z);
      }
      it++;
    }
  }
}

}  // namespace autoware::trajectory_modifier::utils::obstacle_stop
