// Copyright 2025 Instituto de Telecomunições-Porto Branch, Inc. All rights reserved.
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

#include "autoware/spheric_collision_detector/spheric_collision_detector.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/geometry.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <iostream>
#include <memory>
#include <vector>

namespace
{

double calcBrakingDistance(
  const double abs_velocity, const double max_deceleration, const double delay_time)
{
  const double idling_distance = abs_velocity * delay_time;
  const double braking_distance = (abs_velocity * abs_velocity) / (2.0 * max_deceleration);
  return idling_distance + braking_distance;
}

std::vector<std::vector<std::shared_ptr<sphere3::Sphere3>>> createObstacleSpheres(
  const autoware_perception_msgs::msg::DetectedObjects & object_recognition,
  const geometry_msgs::msg::TransformStamped & transform,
  std::vector<autoware_utils::LinearRing2d> & object_area)
{
  using autoware_utils::LinearRing2d;
  using autoware_utils::Point2d;

  std::vector<std::vector<std::shared_ptr<sphere3::Sphere3>>> obstacles;

  for (const auto & obj : object_recognition.objects) {
    std::vector<std::shared_ptr<sphere3::Sphere3>> obstacle_spheres;
    geometry_msgs::msg::Pose map_pose;
    tf2::doTransform(obj.kinematics.pose_with_covariance.pose, map_pose, transform);

    const auto pz = map_pose.position.z;

    autoware_utils::LinearRing2d local_obj_footprint;

    const auto dim_x = obj.shape.dimensions.x;  // Length
    const auto dim_y = obj.shape.dimensions.y;  // Width
    auto object_type = obj.classification.front().label;

    const auto sphere_radius = dim_y * 0.5;
    const double lon_margin = 0.35;  // to tuck spheres about the bounding boxes

    const auto x_front = lon_margin * dim_x;
    const auto x_rear = -x_front;

    local_obj_footprint.push_back(Point2d{x_front, 0.0});
    local_obj_footprint.push_back(Point2d{x_front * 0.5, 0.0});
    local_obj_footprint.push_back(Point2d{0.0, 0.0});
    local_obj_footprint.push_back(Point2d{x_rear * 0.5, 0.0});
    local_obj_footprint.push_back(Point2d{x_rear, 0.0});

    autoware_utils::LinearRing2d current_object_area =
      autoware_utils::transform_vector<autoware_utils::LinearRing2d>(
        local_obj_footprint, autoware_utils::pose2transform(map_pose));
    object_area.push_back(current_object_area);

    for (const auto & area : current_object_area) {
      const auto obstacle_sphere = std::make_shared<sphere3::Sphere3>(
        Eigen::Vector3d(area.x(), area.y(), pz), sphere_radius, object_type);
      obstacle_spheres.push_back(obstacle_sphere);
    }

    obstacles.push_back(obstacle_spheres);
  }

  return obstacles;
}

double computeLargestDistFootprint(
  const spheric_collision_detector::FootprintCoords & footprint_coords_)
{
  double dist;

  double d1 = abs(footprint_coords_.x_front - footprint_coords_.x_center);
  double d2 = abs(footprint_coords_.x_center - footprint_coords_.x_rear);
  double d3 = abs(footprint_coords_.y_left - footprint_coords_.y_right);

  if (d1 > d2 && d1 > d3)
    dist = d1;
  else if (d2 > d1 && d2 > d3)
    dist = d2;
  else
    dist = d3;

  return dist * 0.5;
}

}  // namespace

namespace spheric_collision_detector
{
SphericCollisionDetector::SphericCollisionDetector(rclcpp::Node & node)
: vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo())
{
  const double lon_margin = 0.0;
  const double lat_margin = 0.0;

  footprint_coords_.x_front =
    vehicle_info_.front_overhang_m + vehicle_info_.wheel_base_m + lon_margin;
  footprint_coords_.x_center = vehicle_info_.wheel_base_m / 2.0;
  footprint_coords_.x_rear = -(vehicle_info_.rear_overhang_m + lon_margin);
  footprint_coords_.y_left =
    vehicle_info_.wheel_tread_m / 2.0 + vehicle_info_.left_overhang_m + lat_margin;
  footprint_coords_.y_right =
    -(vehicle_info_.wheel_tread_m / 2.0 + vehicle_info_.right_overhang_m + lat_margin);

  ego_sphere_radius_ = computeLargestDistFootprint(footprint_coords_);
  const double tuck_in_margin = abs(footprint_coords_.y_left - footprint_coords_.y_right) * 0.5;
  vehicle_footprint_.push_back(
    autoware_utils::Point2d{
      footprint_coords_.x_front - tuck_in_margin, footprint_coords_.y_left - tuck_in_margin});
  vehicle_footprint_.push_back(
    autoware_utils::Point2d{
      footprint_coords_.x_center, footprint_coords_.y_right + tuck_in_margin});
  vehicle_footprint_.push_back(
    autoware_utils::Point2d{
      footprint_coords_.x_rear + tuck_in_margin, footprint_coords_.y_left - tuck_in_margin});
  vehicle_footprint_.push_back(
    autoware_utils::Point2d{
      footprint_coords_.x_front - tuck_in_margin, footprint_coords_.y_left - tuck_in_margin});
}

Output SphericCollisionDetector::update(const Input & input)
{
  Output output;

  // resample trajectory by braking distance
  constexpr double min_velocity = 0.01;
  const auto & raw_abs_velocity = std::abs(input.current_twist->linear.x);
  const auto abs_velocity = raw_abs_velocity < min_velocity ? 0.0 : raw_abs_velocity;
  const auto braking_distance =
    calcBrakingDistance(abs_velocity, param_.max_deceleration, param_.delay_time);

  output.resampled_trajectory = cutTrajectory(
    resampleTrajectory(*input.predicted_trajectory, ego_sphere_radius_), braking_distance);

  output.vehicle_footprints =
    createVehicleFootprints(output.resampled_trajectory, vehicle_footprint_);

  auto vehicle_pose_z = input.current_pose->pose.position.z;
  output.vehicle_passing_areas =
    createVehiclePassingAreas(output.vehicle_footprints, vehicle_pose_z, ego_sphere_radius_);

  std::vector<autoware_utils::LinearRing2d> obstacle_areas;
  const auto obstacles = createObstacleSpheres(
    *input.object_recognition, input.object_recognition_transform, obstacle_areas);
  output.obstacles = obstacles;
  output.obstacle_areas = obstacle_areas;

  output.will_collide = false;

  // collision check
  for (const auto & obstacle : obstacles) {
    auto cc_start_time = std::chrono::steady_clock::now();
    output.will_collide = checkCollision(output.vehicle_passing_areas, obstacle);
    auto cc_end_time = std::chrono::steady_clock::now();

    auto cc_elapsed_time =
      std::chrono::duration_cast<std::chrono::nanoseconds>(cc_end_time - cc_start_time);
    output.collision_elapsed_time = cc_elapsed_time.count();

    if (output.will_collide) {
      return output;
    }
  }

  return output;
}

autoware_planning_msgs::msg::Trajectory SphericCollisionDetector::resampleTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double sphere_radius)
{
  autoware_planning_msgs::msg::Trajectory resampled;
  resampled.header = trajectory.header;

  const double dist = 2.0 * sphere_radius;
  resampled.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    const auto & point = trajectory.points.at(i);

    const auto p1 = autoware_utils::from_msg(resampled.points.back().pose.position).to_2d();
    const auto p2 = autoware_utils::from_msg(point.pose.position).to_2d();

    if (boost::geometry::distance(p1, p2) > dist) {
      resampled.points.push_back(point);
    }
  }
  resampled.points.push_back(trajectory.points.back());

  return resampled;
}

autoware_planning_msgs::msg::Trajectory SphericCollisionDetector::cutTrajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double length)
{
  autoware_planning_msgs::msg::Trajectory cut;
  cut.header = trajectory.header;

  double total_length = 0.0;
  cut.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points.at(i);

    const auto p1 = autoware_utils::from_msg(cut.points.back().pose.position);
    const auto p2 = autoware_utils::from_msg(point.pose.position);
    const auto points_distance = boost::geometry::distance(p1.to_2d(), p2.to_2d());

    const auto remain_distance = length - total_length;

    // Over length
    if (remain_distance <= 0.0) {
      break;
    }

    // Require interpolation
    if (remain_distance <= points_distance) {
      const Eigen::Vector3d p_interpolated = p1 + remain_distance * (p2 - p1).normalized();

      autoware_planning_msgs::msg::TrajectoryPoint p;
      p.pose.position.x = p_interpolated.x();
      p.pose.position.y = p_interpolated.y();
      p.pose.position.z = p_interpolated.z();
      p.pose.orientation = point.pose.orientation;

      cut.points.push_back(p);
      break;
    }

    cut.points.push_back(point);
    total_length += points_distance;
  }

  return cut;
}

std::vector<LinearRing2d> SphericCollisionDetector::createVehicleFootprints(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const LinearRing2d & local_vehicle_footprint)
{
  // Create vehicle footprint on each TrajectoryPoint
  std::vector<LinearRing2d> vehicle_footprints;
  for (const auto & p : trajectory.points) {
    vehicle_footprints.push_back(
      autoware_utils::transform_vector<autoware_utils::LinearRing2d>(
        local_vehicle_footprint, autoware_utils::pose2transform(p.pose)));
  }

  return vehicle_footprints;
}

std::vector<std::shared_ptr<sphere3::Sphere3>> SphericCollisionDetector::createVehiclePassingAreas(
  const std::vector<LinearRing2d> & vehicle_footprints, const double vehicle_pose_z,
  const double sphere_radius)
{
  const auto z = vehicle_pose_z + sphere_radius;
  // Create a sphere from vehicle footprints
  std::vector<std::shared_ptr<sphere3::Sphere3>> areas;
  for (const auto & vehicle_footprint : vehicle_footprints) {
    for (size_t i = 0; i < vehicle_footprint.size() - 1; ++i) {
      const auto x = vehicle_footprint.at(i).x();
      const auto y = vehicle_footprint.at(i).y();

      const auto sphere =
        std::make_shared<sphere3::Sphere3>(Eigen::Vector3d(x, y, z), sphere_radius, -1);

      areas.push_back(sphere);
    }
  }

  return areas;
}

bool SphericCollisionDetector::checkCollision(
  const std::vector<std::shared_ptr<sphere3::Sphere3>> & ego_spheres,
  const std::vector<std::shared_ptr<sphere3::Sphere3>> & obstacle_spheres)
{
  for (const auto & obstacle_sphere : obstacle_spheres) {
    for (const auto & ego_sphere : ego_spheres) {
      double dx = ego_sphere->center_.x() - obstacle_sphere->center_.x();
      double dy = ego_sphere->center_.y() - obstacle_sphere->center_.y();
      double dz = ego_sphere->center_.z() - obstacle_sphere->center_.z();

      double center_dist = dx * dx + dy * dy + dz * dz;
      double sum_radii = ego_sphere->radius_ + obstacle_sphere->radius_;

      if (center_dist <= sum_radii * sum_radii) {
        return true;
      }
    }
  }

  return false;
}

}  // namespace spheric_collision_detector
