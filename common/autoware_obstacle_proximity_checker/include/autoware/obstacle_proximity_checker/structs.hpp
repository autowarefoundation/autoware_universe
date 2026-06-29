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

#ifndef AUTOWARE__OBSTACLE_PROXIMITY_CHECKER__STRUCTS_HPP_
#define AUTOWARE__OBSTACLE_PROXIMITY_CHECKER__STRUCTS_HPP_

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

namespace autoware::obstacle_proximity_checker
{

struct ObstacleTypeParameters
{
  double surround_check_front_distance{0.0};
  double surround_check_side_distance{0.0};
  double surround_check_back_distance{0.0};
};

struct Parameters
{
  bool pointcloud_enable_check{false};
  std::unordered_map<std::string, bool> object_type_enable_check;
  std::unordered_map<std::string, ObstacleTypeParameters> obstacle_types_map;
};

struct ProximityObstacle
{
  bool is_point_cloud{false};
  double nearest_distance{0.0};
  geometry_msgs::msg::Point nearest_point;
  unique_identifier_msgs::msg::UUID uuid;
};

struct Inputs
{
  geometry_msgs::msg::Pose ego_pose;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud_in_base_link{nullptr};
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects{nullptr};
};

struct CheckResult
{
  bool is_obstacle_found{false};
  std::optional<ProximityObstacle> nearest_obstacle;
};

}  // namespace autoware::obstacle_proximity_checker

#endif  // AUTOWARE__OBSTACLE_PROXIMITY_CHECKER__STRUCTS_HPP_
