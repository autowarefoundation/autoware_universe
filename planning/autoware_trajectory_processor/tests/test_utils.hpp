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

#ifndef PLANNING__AUTOWARE_TRAJECTORY_PROCESSOR__TESTS__TEST_UTILS_HPP_
#define PLANNING__AUTOWARE_TRAJECTORY_PROCESSOR__TESTS__TEST_UTILS_HPP_

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <optional>
#include <string>
#include <vector>

namespace trajectory_optimizer_test_utils
{

using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

/**
 * @brief A named parameter value selected for a parameter-service test.
 */
struct ParameterCase
{
  std::string name;
  rclcpp::Parameter value;
};

/**
 * @brief Create options for an optimizer node used by parameter-update tests.
 *
 * Loads the optimizer, plugin, legacy EB/MPT, and vehicle parameter files and configures a
 * deterministic list containing every optimizer plugin under test.
 */
rclcpp::NodeOptions make_optimizer_node_options();

/**
 * @brief Create options for a modifier node used by parameter-update tests.
 *
 * Loads the modifier and vehicle parameter files required to initialize every modifier plugin.
 */
rclcpp::NodeOptions make_modifier_node_options();

/**
 * @brief Return whether a parameter controls the loaded plugin list.
 *
 * Plugin-list updates are tested separately because runtime plugin reconciliation is not yet
 * implemented.
 */
bool is_plugin_list_parameter(const std::string & name);

/**
 * @brief Return whether a parameter descriptor marks the parameter as read-only.
 */
bool is_read_only(const rcl_interfaces::msg::ParameterDescriptor & descriptor);

/**
 * @brief Create a valid value different from the current scalar parameter value.
 *
 * Descriptor ranges are respected when present. Unsupported parameter types and parameters with
 * no alternative value return std::nullopt.
 */
std::optional<rclcpp::Parameter> make_updated_parameter(
  const rclcpp::Parameter & current, const rcl_interfaces::msg::ParameterDescriptor & descriptor);

/**
 * @brief List parameters declared by a remote node through its parameter services.
 *
 * The supplied executor is spun until the asynchronous request completes or times out.
 */
std::vector<std::string> list_declared_parameters(
  rclcpp::AsyncParametersClient & client, rclcpp::executors::SingleThreadedExecutor & executor);

/**
 * @brief Set one remote parameter atomically after verifying its value will change.
 *
 * The supplied executor services both the initial-value request and the atomic set request.
 */
void set_parameter_atomically(
  rclcpp::AsyncParametersClient & client, rclcpp::executors::SingleThreadedExecutor & executor,
  const rclcpp::Parameter & parameter);

/**
 * @brief Set multiple remote parameters atomically after verifying every value will change.
 *
 * The supplied executor services both the initial-value request and the atomic set request.
 */
void set_parameters_atomically(
  rclcpp::AsyncParametersClient & client, rclcpp::executors::SingleThreadedExecutor & executor,
  const std::vector<rclcpp::Parameter> & parameters);

/**
 * @brief Creates a simple sample trajectory with uniformly spaced points.
 *
 * @param resolution Distance between consecutive points
 * @param offset Starting position offset
 * @param num_points Number of points to create (default: 10)
 * @return Vector of trajectory points
 */
inline std::vector<TrajectoryPoint> create_sample_trajectory(
  double resolution = 1.0, double offset = 0.0, int num_points = 10)
{
  std::vector<TrajectoryPoint> points;
  for (int i = 0; i < num_points; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = i * resolution + offset;
    point.pose.position.y = i * resolution + offset;
    point.pose.position.z = 0.0;

    // Default orientation (facing forward along x-axis)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    point.pose.orientation = tf2::toMsg(q);

    point.longitudinal_velocity_mps = 1.0;
    point.acceleration_mps2 = 0.1;
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 0;
    points.push_back(point);
  }
  return points;
}

/**
 * @brief Creates a single trajectory point at the specified position.
 *
 * @param x X coordinate
 * @param y Y coordinate
 * @param velocity Longitudinal velocity (default: 1.0 m/s)
 * @param acceleration Longitudinal acceleration (default: 0.0 m/s²)
 * @return TrajectoryPoint with specified properties
 */
inline TrajectoryPoint create_point(
  double x, double y, float velocity = 1.0f, float acceleration = 0.0f)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = 0.0;

  // Default orientation (facing forward along x-axis)
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  point.pose.orientation = tf2::toMsg(q);

  point.longitudinal_velocity_mps = velocity;
  point.acceleration_mps2 = acceleration;
  point.time_from_start.sec = 0;
  point.time_from_start.nanosec = 0;
  return point;
}

/**
 * @brief Creates a trajectory point with a specified yaw angle.
 *
 * @param x X coordinate
 * @param y Y coordinate
 * @param yaw Yaw angle in radians
 * @param velocity Longitudinal velocity (default: 1.0 m/s)
 * @return TrajectoryPoint with specified position and orientation
 */
inline TrajectoryPoint create_point_with_yaw(double x, double y, double yaw, float velocity = 1.0f)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  point.pose.orientation = tf2::toMsg(q);

  point.longitudinal_velocity_mps = velocity;
  point.acceleration_mps2 = 0.0f;
  point.time_from_start.sec = 0;
  point.time_from_start.nanosec = 0;
  return point;
}

/**
 * @brief Creates an odometry message for testing.
 *
 * @param x X coordinate
 * @param y Y coordinate
 * @param yaw Yaw angle in radians
 * @param velocity Linear velocity in x direction
 * @return Odometry message with specified properties
 */
inline Odometry create_odometry(double x, double y, double yaw, double velocity)
{
  Odometry odom;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  odom.pose.pose.orientation = tf2::toMsg(q);

  odom.twist.twist.linear.x = velocity;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;
  return odom;
}

}  // namespace trajectory_optimizer_test_utils

#endif  // PLANNING__AUTOWARE_TRAJECTORY_PROCESSOR__TESTS__TEST_UTILS_HPP_
