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

#ifndef SPHERIC_COLLISION_DETECTOR__SPHERIC_COLLISION_DETECTOR_HPP_
#define SPHERIC_COLLISION_DETECTOR__SPHERIC_COLLISION_DETECTOR_HPP_

#include "spheric_collision_detector/sphere3.hpp"

#include <tier4_autoware_utils/geometry/boost_geometry.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/optional.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <string>
#include <vector>
#include <fstream>
#include <chrono>

namespace spheric_collision_detector
{
using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::Point2d;

using Path = std::vector<geometry_msgs::msg::Pose>;

struct Param
{
  double delay_time;
  double max_deceleration;
};

struct Input
{
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;
  geometry_msgs::msg::Twist::ConstSharedPtr current_twist;
  autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr object_recognition;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr obstacle_transform;
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr predicted_trajectory;
  geometry_msgs::msg::TransformStamped object_recognition_transform;
};

struct Output
{ 
  std::map<std::string, double> processing_time_map;
  bool will_collide;
  autoware_auto_planning_msgs::msg::Trajectory resampled_trajectory;
  std::vector<LinearRing2d> vehicle_footprints;
  std::vector<std::shared_ptr<sphere3::Sphere3>> vehicle_passing_areas;
  long int collision_elapsed_time;
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
  double ego_sphere_radius_;
  vehicle_info_util::VehicleInfo vehicle_info_;
  LinearRing2d vehicle_footprint_;

  std::ofstream out_file_;

  //! This function assumes the input trajectory is sampled dense enough
  static autoware_auto_planning_msgs::msg::Trajectory resampleTrajectory(
    const autoware_auto_planning_msgs::msg::Trajectory & trajectory, const double interval);

  static autoware_auto_planning_msgs::msg::Trajectory cutTrajectory(
    const autoware_auto_planning_msgs::msg::Trajectory & trajectory, const double length);

  static std::vector<LinearRing2d> createVehicleFootprints(
    const autoware_auto_planning_msgs::msg::Trajectory & trajectory, 
    const LinearRing2d & local_vehicle_footprint);

  static std::vector<std::shared_ptr<sphere3::Sphere3>> createVehiclePassingAreas(
    const std::vector<LinearRing2d> & vehicle_footprints, 
    const double vehicle_height,
    const double sphere_radius);

    static bool checkCollision(
      const std::vector<std::shared_ptr<sphere3::Sphere3>> & ego_spheres, 
      const std::vector<std::shared_ptr<sphere3::Sphere3>> & obstacle_spheres);
};
}  // namespace spheric_collision_detector

#endif  // SPHERIC_COLLISION_DETECTOR__SPHERIC_COLLISION_DETECTOR_HPP_
