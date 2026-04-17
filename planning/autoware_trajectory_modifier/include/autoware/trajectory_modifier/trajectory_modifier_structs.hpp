// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_STRUCTS_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_STRUCTS_HPP_
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace autoware::trajectory_modifier
{
using autoware_perception_msgs::msg::PredictedObjects;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;
struct TrajectoryModifierData
{
  explicit TrajectoryModifierData(rclcpp::Node * node)
  : vehicle_info(autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo()),
    tf_buffer{node->get_clock()},
    tf_listener{tf_buffer}
  {
  }

  Odometry::ConstSharedPtr current_odometry;
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration;
  PredictedObjects::ConstSharedPtr predicted_objects;
  PointCloud2::ConstSharedPtr obstacle_pointcloud;

  autoware::vehicle_info_utils::VehicleInfo vehicle_info;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  tl::expected<std::string, std::string> is_ready()
  {
    if (!current_odometry) {
      return tl::make_unexpected("current_odometry is not set");
    }
    if (!current_acceleration) {
      return tl::make_unexpected("current_acceleration is not set");
    }
    if (!predicted_objects) {
      return "predicted_objects is not set";
    }
    if (!obstacle_pointcloud) {
      return "obstacle_pointcloud is not set";
    }
    return "";
  }
};
}  // namespace autoware::trajectory_modifier
#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_STRUCTS_HPP_
