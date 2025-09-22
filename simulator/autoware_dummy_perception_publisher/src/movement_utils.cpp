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

#include "autoware/dummy_perception_publisher/movement_utils.hpp"

#include "autoware/trajectory/interpolator/akima_spline.hpp"
#include "autoware/trajectory/interpolator/interpolator.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <geometry_msgs/msg/transform.hpp>

#include <tf2/LinearMath/Transform.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace autoware::dummy_perception_publisher::utils
{
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Transform;
using tier4_simulation_msgs::msg::DummyObject;

geometry_msgs::msg::Pose MovementUtils::calculate_straight_line_position(
  const DummyObject & object, const rclcpp::Time & current_time)
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

  stop_at_zero_velocity(current_vel, initial_vel, initial_acc);

  // Clamp velocity within min/max
  current_vel = std::clamp(
    current_vel, static_cast<double>(object.min_velocity),
    static_cast<double>(object.max_velocity));

  // Add distance on acceleration or deceleration
  move_distance = (std::pow(current_vel, 2) - std::pow(initial_vel, 2)) * 0.5 / initial_acc;

  // Add distance after reaching max_velocity
  if (initial_acc > 0) {
    const double time_to_reach_max_vel =
      std::max(static_cast<double>(object.max_velocity) - initial_vel, 0.0) / initial_acc;
    move_distance += static_cast<double>(object.max_velocity) *
                     std::max(elapsed_time - time_to_reach_max_vel, 0.0);
  }

  // Add distance after reaching min_velocity
  if (initial_acc < 0) {
    const double time_to_reach_min_vel =
      std::min(static_cast<double>(object.min_velocity) - initial_vel, 0.0) / initial_acc;
    move_distance += static_cast<double>(object.min_velocity) *
                     std::max(elapsed_time - time_to_reach_min_vel, 0.0);
  }

  return autoware_utils_geometry::calc_offset_pose(initial_pose, move_distance, 0.0, 0.0);
}

void MovementUtils::stop_at_zero_velocity(
  double & current_vel, double initial_vel, double initial_acc)
{
  // Stop at zero velocity
  if (initial_acc < 0 && initial_vel > 0) {
    current_vel = std::max(current_vel, 0.0);
  }
  if (initial_acc > 0 && initial_vel < 0) {
    current_vel = std::min(current_vel, 0.0);
  }
}

ObjectInfo MovementUtils::create_basic_object_info(const DummyObject & object)
{
  ObjectInfo obj_info;

  // Set dimensions
  obj_info.length = object.shape.dimensions.x;
  obj_info.width = object.shape.dimensions.y;
  obj_info.height = object.shape.dimensions.z;

  // Set standard deviations
  obj_info.std_dev_x = std::sqrt(object.initial_state.pose_covariance.covariance[0]);
  obj_info.std_dev_y = std::sqrt(object.initial_state.pose_covariance.covariance[7]);
  obj_info.std_dev_z = std::sqrt(object.initial_state.pose_covariance.covariance[14]);
  obj_info.std_dev_yaw = std::sqrt(object.initial_state.pose_covariance.covariance[35]);

  // Set twist and pose covariances
  obj_info.twist_covariance_ = object.initial_state.twist_covariance;
  obj_info.pose_covariance_ = object.initial_state.pose_covariance;

  return obj_info;
}

void MovementUtils::update_object_info_with_movement(
  ObjectInfo & obj_info, const DummyObject & object, const Pose & current_pose,
  const rclcpp::Time & current_time)
{
  // Calculate tf from map to moved_object
  Transform ros_map2moved_object;
  ros_map2moved_object.translation.x = current_pose.position.x;
  ros_map2moved_object.translation.y = current_pose.position.y;
  ros_map2moved_object.translation.z = current_pose.position.z;
  ros_map2moved_object.rotation = current_pose.orientation;
  tf2::fromMsg(ros_map2moved_object, obj_info.tf_map2moved_object);

  // Update pose
  obj_info.pose_covariance_.pose = current_pose;

  // Calculate and update velocity
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

  stop_at_zero_velocity(current_vel, initial_vel, initial_acc);
  obj_info.twist_covariance_.twist.linear.x = current_vel;
}

geometry_msgs::msg::Pose MovementUtils::calculate_trajectory_based_position(
  const DummyObject & object,
  const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  const rclcpp::Time & predicted_time, const rclcpp::Time & current_time)
{
  using autoware::experimental::trajectory::Trajectory;
  using autoware::experimental::trajectory::interpolator::AkimaSpline;
  using InterpolationTrajectory = Trajectory<Pose>;

  // Select first path (which has been ordered based on random or highest confidence strategy)
  const auto & selected_path = predicted_object.kinematics.predicted_paths.front();

  // Calculate elapsed time from predicted object timestamp to current time
  const double elapsed_time = (current_time - predicted_time).seconds();

  // Calculate distance traveled based on elapsed time and dummy object speed
  const double speed = object.initial_state.twist_covariance.twist.linear.x;
  const double distance_traveled = speed * elapsed_time;

  if (distance_traveled <= 0.0 || selected_path.path.empty()) {
    return predicted_object.kinematics.initial_pose_with_covariance.pose;
  }

  if (selected_path.path.size() < 2) {  // Fallback to last pose if path has only one point
    return selected_path.path.back();
  }

  const auto interpolated_pose = std::invoke([&]() {
    auto trajectory_interpolation_util =
      InterpolationTrajectory::Builder{}
        .set_xy_interpolator<AkimaSpline>()  // Set interpolator for x-y plane
        .build(selected_path.path);

    if (!trajectory_interpolation_util) {
      // Fallback to initial pose if failed to build interpolation trajectory
      return object.initial_state.pose_covariance.pose;
    }
    trajectory_interpolation_util->align_orientation_with_trajectory_direction();

    const auto total_length = trajectory_interpolation_util->length();
    // Check if the distance traveled exceeds the path length (extrapolation)
    if (distance_traveled >= total_length) {
      const double overshoot_distance = distance_traveled - total_length;

      // Use the last two points to determine direction and extrapolate
      const auto & second_last_pose = selected_path.path[selected_path.path.size() - 2];
      const auto & last_pose = selected_path.path.back();

      // Calculate direction vector from second-last to last pose
      const double dx = last_pose.position.x - second_last_pose.position.x;
      const double dy = last_pose.position.y - second_last_pose.position.y;
      const double dz = last_pose.position.z - second_last_pose.position.z;
      const double segment_length = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (segment_length < std::numeric_limits<double>::epsilon()) {
        // Fallback to last pose if segment length is zero
        return last_pose;
      }

      // Normalize direction vector
      const double dir_x = dx / segment_length;
      const double dir_y = dy / segment_length;
      const double dir_z = dz / segment_length;

      // Extrapolate position
      Pose extrapolated_pose;
      extrapolated_pose.position.x = last_pose.position.x + dir_x * overshoot_distance;
      extrapolated_pose.position.y = last_pose.position.y + dir_y * overshoot_distance;
      extrapolated_pose.position.z = last_pose.position.z + dir_z * overshoot_distance;

      // Keep the last orientation
      extrapolated_pose.orientation = last_pose.orientation;
      return extrapolated_pose;
    }

    // Interpolation within the path
    return trajectory_interpolation_util->compute(distance_traveled);
  });

  return interpolated_pose;
}

}  // namespace autoware::dummy_perception_publisher::utils
