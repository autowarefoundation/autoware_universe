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

#include "autoware/dummy_perception_publisher/object_info.hpp"

#include "autoware_utils_geometry/geometry.hpp"

#include <algorithm>
#include <limits>

namespace autoware::dummy_perception_publisher
{
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Transform;

ObjectInfo::ObjectInfo(
  const tier4_simulation_msgs::msg::DummyObject & object, const rclcpp::Time & current_time)
: length(object.shape.dimensions.x),
  width(object.shape.dimensions.y),
  height(object.shape.dimensions.z),
  std_dev_x(std::sqrt(object.initial_state.pose_covariance.covariance[0])),
  std_dev_y(std::sqrt(object.initial_state.pose_covariance.covariance[7])),
  std_dev_z(std::sqrt(object.initial_state.pose_covariance.covariance[14])),
  std_dev_yaw(std::sqrt(object.initial_state.pose_covariance.covariance[35])),
  twist_covariance_(object.initial_state.twist_covariance),
  pose_covariance_(object.initial_state.pose_covariance)
{
  const auto current_pose = calculateStraightLinePosition(object, current_time);

  // calculate tf from map to moved_object
  Transform ros_map2moved_object;
  ros_map2moved_object.translation.x = current_pose.position.x;
  ros_map2moved_object.translation.y = current_pose.position.y;
  ros_map2moved_object.translation.z = current_pose.position.z;
  ros_map2moved_object.rotation = current_pose.orientation;
  tf2::fromMsg(ros_map2moved_object, tf_map2moved_object);

  // set twist and pose information
  const double initial_vel = std::clamp(
    object.initial_state.twist_covariance.twist.linear.x, static_cast<double>(object.min_velocity),
    static_cast<double>(object.max_velocity));
  const double initial_acc = object.initial_state.accel_covariance.accel.linear.x;
  const double elapsed_time = current_time.seconds() - rclcpp::Time(object.header.stamp).seconds();
  double current_vel = initial_vel + initial_acc * elapsed_time;
  if (initial_acc != 0.0) {
    current_vel = std::clamp(
      current_vel, static_cast<double>(object.min_velocity),
      static_cast<double>(object.max_velocity));
  }

  stopAtZeroVelocity(current_vel, initial_vel, initial_acc);

  twist_covariance_.twist.linear.x = current_vel;
  pose_covariance_.pose = current_pose;
}

ObjectInfo::ObjectInfo(
  const tier4_simulation_msgs::msg::DummyObject & object, const PredictedObject & predicted_object,
  const rclcpp::Time & predicted_time, const rclcpp::Time & current_time,
  const double switch_time_threshold)
: length(object.shape.dimensions.x),
  width(object.shape.dimensions.y),
  height(object.shape.dimensions.z),
  std_dev_x(std::sqrt(object.initial_state.pose_covariance.covariance[0])),
  std_dev_y(std::sqrt(object.initial_state.pose_covariance.covariance[7])),
  std_dev_z(std::sqrt(object.initial_state.pose_covariance.covariance[14])),
  std_dev_yaw(std::sqrt(object.initial_state.pose_covariance.covariance[35])),
  twist_covariance_(object.initial_state.twist_covariance),
  pose_covariance_(object.initial_state.pose_covariance)
{
  // Check if threshold time has passed since object creation
  const double time_since_creation = (current_time - rclcpp::Time(object.header.stamp)).seconds();
  // Use straight-line movement for first switch_time_threshold seconds, then switch to predicted
  // path
  if (
    time_since_creation < switch_time_threshold ||
    predicted_object.kinematics.predicted_paths.empty()) {
    // Reuse the logic from the other constructor
    *this = ObjectInfo(object, current_time);
    return;
  }

  const auto interpolated_pose =
    calculateTrajectoryBasedPosition(object, predicted_object, predicted_time, current_time);

  // Update pose and transform
  pose_covariance_.pose = interpolated_pose;
  tf2::fromMsg(interpolated_pose, tf_map2moved_object);

  // Use dummy object's velocity consistently
  twist_covariance_.twist.linear.x = object.initial_state.twist_covariance.twist.linear.x;
}

void ObjectInfo::stopAtZeroVelocity(double & current_vel, double initial_vel, double initial_acc)
{
  // stop at zero velocity
  if (initial_acc < 0 && initial_vel > 0) {
    current_vel = std::max(current_vel, 0.0);
  }
  if (initial_acc > 0 && initial_vel < 0) {
    current_vel = std::min(current_vel, 0.0);
  }
}

Pose ObjectInfo::calculateStraightLinePosition(
  const tier4_simulation_msgs::msg::DummyObject & object, const rclcpp::Time & current_time)
{
  const auto & initial_pose = object.initial_state.pose_covariance.pose;
  const double initial_vel = std::clamp(
    object.initial_state.twist_covariance.twist.linear.x, static_cast<double>(object.min_velocity),
    static_cast<double>(object.max_velocity));
  const double initial_acc = object.initial_state.accel_covariance.accel.linear.x;

  const double elapsed_time = current_time.seconds() - rclcpp::Time(object.header.stamp).seconds();

  double move_distance{0.0};
  double current_vel = initial_vel + initial_acc * elapsed_time;
  if (initial_acc == 0.0) {
    move_distance = initial_vel * elapsed_time;
    return autoware_utils_geometry::calc_offset_pose(initial_pose, move_distance, 0.0, 0.0);
  }

  stopAtZeroVelocity(current_vel, initial_vel, initial_acc);
  // clamp velocity within min/max
  current_vel = std::clamp(
    current_vel, static_cast<double>(object.min_velocity),
    static_cast<double>(object.max_velocity));

  // add distance on acceleration or deceleration
  move_distance = (std::pow(current_vel, 2) - std::pow(initial_vel, 2)) * 0.5 / initial_acc;

  // add distance after reaching max_velocity
  if (initial_acc > 0) {
    const double time_to_reach_max_vel =
      std::max(static_cast<double>(object.max_velocity) - initial_vel, 0.0) / initial_acc;
    move_distance += static_cast<double>(object.max_velocity) *
                     std::max(elapsed_time - time_to_reach_max_vel, 0.0);
  }

  // add distance after reaching min_velocity
  if (initial_acc < 0) {
    const double time_to_reach_min_vel =
      std::min(static_cast<double>(object.min_velocity) - initial_vel, 0.0) / initial_acc;
    move_distance += static_cast<double>(object.min_velocity) *
                     std::max(elapsed_time - time_to_reach_min_vel, 0.0);
  }

  return autoware_utils_geometry::calc_offset_pose(initial_pose, move_distance, 0.0, 0.0);
}

}  // namespace autoware::dummy_perception_publisher
