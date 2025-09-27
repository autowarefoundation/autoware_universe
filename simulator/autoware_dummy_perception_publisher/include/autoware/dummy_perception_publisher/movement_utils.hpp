// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__MOVEMENT_UTILS_HPP_
#define AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__MOVEMENT_UTILS_HPP_

#include "autoware/dummy_perception_publisher/object_info.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tier4_simulation_msgs/msg/dummy_object.hpp>

namespace autoware::dummy_perception_publisher::utils
{

class MovementUtils
{
public:
  // Calculate position for straight-line movement
  static geometry_msgs::msg::Pose calculate_straight_line_position(
    const tier4_simulation_msgs::msg::DummyObject & object, const rclcpp::Time & current_time);

  // Helper to stop at zero velocity
  static void stop_at_zero_velocity(double & current_vel, double initial_vel, double initial_acc);

  // Create a basic ObjectInfo with common fields populated
  static ObjectInfo create_basic_object_info(
    const tier4_simulation_msgs::msg::DummyObject & object);

  // Update ObjectInfo with calculated pose and velocity
  static void update_object_info_with_movement(
    ObjectInfo & obj_info, const tier4_simulation_msgs::msg::DummyObject & object,
    const geometry_msgs::msg::Pose & current_pose, const rclcpp::Time & current_time);

  // Calculate position based on predicted trajectory path
  static geometry_msgs::msg::Pose calculate_trajectory_based_position(
    const tier4_simulation_msgs::msg::DummyObject & object,
    const autoware_perception_msgs::msg::PredictedObject & predicted_object,
    const rclcpp::Time & predicted_time, const rclcpp::Time & current_time);
};

}  // namespace autoware::dummy_perception_publisher::utils

#endif  // AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__MOVEMENT_UTILS_HPP_
