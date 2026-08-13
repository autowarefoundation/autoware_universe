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

#ifndef AUTOWARE__SPHERIC_COLLISION_DETECTOR__SPHERIC_COLLISION_DETECTOR_HPP_
#define AUTOWARE__SPHERIC_COLLISION_DETECTOR__SPHERIC_COLLISION_DETECTOR_HPP_

#include "autoware/spheric_collision_detector/sphere3.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/optional.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace spheric_collision_detector
{
using autoware_utils::LinearRing2d;
using autoware_utils::Point2d;

using Path = std::vector<geometry_msgs::msg::Pose>;

struct Param
{
  double delay_time;
  double max_deceleration;
};

struct FootprintCoords
{
  double x_front;
  double x_center;
  double x_rear;
  double y_left;
  double y_right;
};

struct Input
{
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;
  geometry_msgs::msg::Twist::ConstSharedPtr current_twist;
  autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr object_recognition;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr obstacle_transform;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr predicted_trajectory;
  geometry_msgs::msg::TransformStamped object_recognition_transform;
};

struct Output
{
  std::map<std::string, double> processing_time_map;
  bool will_collide;
  autoware_planning_msgs::msg::Trajectory resampled_trajectory;
  std::vector<LinearRing2d> vehicle_footprints;
  std::vector<std::shared_ptr<sphere3::Sphere3>> vehicle_passing_areas;
  int64_t collision_elapsed_time;
  std::vector<std::vector<std::shared_ptr<sphere3::Sphere3>>> obstacles;
  std::vector<LinearRing2d> obstacle_areas;
};

class SphericCollisionDetector
{
public:
  explicit SphericCollisionDetector(rclcpp::Node & node);
  Output update(const Input & input);

  void setParam(const Param & param) { param_ = param; }

private:
  Param param_;
  FootprintCoords footprint_coords_;
  double ego_sphere_radius_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  LinearRing2d vehicle_footprint_;

  std::ofstream out_file_;

  //! This function assumes the input trajectory is sampled dense enough
  static autoware_planning_msgs::msg::Trajectory resampleTrajectory(
    const autoware_planning_msgs::msg::Trajectory & trajectory, const double interval);

  static autoware_planning_msgs::msg::Trajectory cutTrajectory(
    const autoware_planning_msgs::msg::Trajectory & trajectory, const double length);

  static std::vector<LinearRing2d> createVehicleFootprints(
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    const LinearRing2d & local_vehicle_footprint);

  static std::vector<std::shared_ptr<sphere3::Sphere3>> createVehiclePassingAreas(
    const std::vector<LinearRing2d> & vehicle_footprints, const double vehicle_height,
    const double sphere_radius);

  static bool checkCollision(
    const std::vector<std::shared_ptr<sphere3::Sphere3>> & ego_spheres,
    const std::vector<std::shared_ptr<sphere3::Sphere3>> & obstacle_spheres);
};
}  // namespace spheric_collision_detector

#endif  // AUTOWARE__SPHERIC_COLLISION_DETECTOR__SPHERIC_COLLISION_DETECTOR_HPP_
