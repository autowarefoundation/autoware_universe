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

#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier::utils::obstacle_stop
{

autoware_utils_geometry::MultiPolygon2d get_trajectory_polygon(
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
  return trajectory_polygon;
}

void filter_pointcloud_by_range(
  PointCloud::Ptr & pointcloud, const double detection_range, const double min_height,
  const double max_height)
{
  if (pointcloud->empty()) return;

  {
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(pointcloud);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(-1.0 * detection_range, detection_range);
    filter.filter(*pointcloud);
  }

  if (pointcloud->empty()) return;

  {
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(pointcloud);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(min_height, max_height);
    filter.filter(*pointcloud);
  }
}

void filter_pointcloud_by_voxel_grid(
  PointCloud::Ptr & pointcloud, const double voxel_size_x, const double voxel_size_y,
  const double voxel_size_z, const int min_size)
{
  if (pointcloud->empty()) return;

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(pointcloud);
  filter.setLeafSize(voxel_size_x, voxel_size_y, voxel_size_z);
  filter.setMinimumPointsNumberPerVoxel(min_size);
  filter.filter(*pointcloud);
}

void cluster_pointcloud(
  PointCloud::Ptr & input, PointCloud::Ptr & output, const size_t min_size, const size_t max_size,
  const double tolerance, const double min_height, const double height_offset)
{
  if (input->empty()) return;

  const std::vector<pcl::PointIndices> cluster_indices = std::invoke([&]() {
    std::vector<pcl::PointIndices> cluster_idx;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(cluster_idx);
    return cluster_idx;
  });

  auto above_height_threshold = [&](const double z) {
    const auto rel_height = z - height_offset;
    return rel_height > min_height;
  };

  for (const auto & indices : cluster_indices) {
    PointCloud::Ptr cluster(new PointCloud);
    bool cluster_above_height_threshold{false};
    for (const auto & index : indices.indices) {
      const auto & point = (*input)[index];

      cluster_above_height_threshold |= above_height_threshold(point.z);
      cluster->push_back(point);
    }
    if (!cluster_above_height_threshold) continue;

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setDimension(2);
    hull.setInputCloud(cluster);
    PointCloud::Ptr surface_hull(new PointCloud);
    hull.reconstruct(*surface_hull);
    for (const auto & point : *surface_hull) {
      output->push_back(point);
    }
  }
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
  const PointCloud::Ptr & pointcloud)
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
  }

  return CollisionPoint(nearest_collision_point, min_arc_length);
}

std::optional<CollisionPoint> get_nearest_object_collision(
  const TrajectoryPoints & trajectory_points,
  const autoware_utils_geometry::MultiPolygon2d & trajectory_polygon,
  const PredictedObjects & objects)
{
  if (objects.objects.empty()) return std::nullopt;

  auto min_arc_length = std::numeric_limits<double>::max();
  geometry_msgs::msg::Point nearest_collision_point;
  bool found_collision = false;
  for (const auto & object : objects.objects) {
    const auto object_pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto object_polygon = autoware_utils::to_polygon2d(object_pose, object.shape);
    ;
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
      }
    }
  }

  if (!found_collision) return std::nullopt;
  return CollisionPoint(nearest_collision_point, min_arc_length);
}

}  // namespace autoware::trajectory_modifier::utils::obstacle_stop
