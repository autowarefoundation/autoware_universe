// Copyright 2021 Tier IV, Inc.
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

#include "autoware/planning_validator_test_utils/planning_validator_test_utils.hpp"

#include "autoware/planning_validator_test_utils/test_parameters.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <math.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils::create_quaternion_from_yaw;

namespace autoware::planning_validator::test_utils
{

Trajectory generateTrajectoryWithConstantAcceleration(
  const double interval_distance, const double speed, const double yaw, const size_t size,
  const double acceleration)
{
  Trajectory trajectory;
  trajectory.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  double s = 0.0, v = speed, a = acceleration;
  constexpr auto MAX_DT = 10.0;
  for (size_t i = 0; i < size; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = s * std::cos(yaw);
    p.pose.position.y = s * std::sin(yaw);
    p.pose.orientation = create_quaternion_from_yaw(yaw);
    p.longitudinal_velocity_mps = v;
    p.acceleration_mps2 = a;
    p.front_wheel_angle_rad = 0.0;
    trajectory.points.push_back(p);
    s += interval_distance;

    const auto dt = std::abs(v) > 0.1 ? interval_distance / v : MAX_DT;
    v += acceleration * dt;
    if (v < 0.0) {
      v = 0.0;
      a = 0.0;
    }
  }
  return trajectory;
}

Trajectory generateTrajectoryWithVariableAcceleration(
  const double interval_distance, const double initial_speed, const double yaw, const size_t size,
  const std::function<double(size_t)> & acceleration_pattern)
{
  Trajectory trajectory;
  trajectory.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  double s = 0.0, v = initial_speed;
  constexpr auto MAX_DT = 10.0;

  for (size_t i = 0; i < size; ++i) {
    // Get acceleration for this point based on the provided pattern
    const double a = acceleration_pattern(i);

    TrajectoryPoint p;
    p.pose.position.x = s * std::cos(yaw);
    p.pose.position.y = s * std::sin(yaw);
    p.pose.orientation = create_quaternion_from_yaw(yaw);
    p.longitudinal_velocity_mps = v;
    p.acceleration_mps2 = a;
    p.front_wheel_angle_rad = 0.0;
    trajectory.points.push_back(p);

    s += interval_distance;

    // Calculate time interval and update velocity
    const auto dt = std::abs(v) > 0.1 ? interval_distance / v : MAX_DT;
    v += a * dt;

    // Ensure velocity doesn't go negative
    if (v < 0.0) {
      v = 0.0;
    }
  }
  return trajectory;
}

Trajectory generateTrajectoryWithSinusoidalAcceleration(
  const double interval_distance, const double initial_speed, const double yaw, const size_t size,
  const double max_acceleration, const double oscillation_period)
{
  return generateTrajectoryWithVariableAcceleration(
    interval_distance, initial_speed, yaw, size,
    [max_acceleration, oscillation_period](size_t i) -> double {
      return max_acceleration * std::sin(2.0 * M_PI * i / oscillation_period);
    });
}

Trajectory generateTrajectoryWithStepAcceleration(
  const double interval_distance, const double initial_speed, const double yaw, const size_t size,
  const std::vector<double> & acceleration_values, const size_t steps_per_value)
{
  return generateTrajectoryWithVariableAcceleration(
    interval_distance, initial_speed, yaw, size,
    [acceleration_values, steps_per_value](size_t i) -> double {
      const size_t pattern_index = (i / steps_per_value) % acceleration_values.size();
      return acceleration_values[pattern_index];
    });
}

Trajectory generateTrajectory(
  const double interval_distance, const double speed, const double yaw, const size_t size)
{
  constexpr auto acceleration = 0.0;
  return generateTrajectoryWithConstantAcceleration(
    interval_distance, speed, yaw, size, acceleration);
}

Trajectory generateTrajectoryWithConstantCurvature(
  const double interval_distance, const double speed, const double curvature, const size_t size,
  const double wheelbase)
{
  if (std::abs(curvature) < 1.0e-5) {
    return generateTrajectory(interval_distance, speed, 0.0, size);
  }

  const auto steering = std::atan(curvature * wheelbase);
  const auto radius = 1.0 / curvature;

  Trajectory trajectory;
  trajectory.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  double x = 0.0, y = 0.0, yaw = 0.0;

  for (size_t i = 0; i <= size; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation = create_quaternion_from_yaw(yaw);
    p.longitudinal_velocity_mps = speed;
    p.front_wheel_angle_rad = steering;
    trajectory.points.push_back(p);

    // Update x, y, yaw for the next point
    const auto prev_yaw = yaw;
    double delta_yaw = curvature * interval_distance;
    yaw += delta_yaw;
    x += radius * (std::sin(yaw) - std::sin(prev_yaw));
    y -= radius * (std::cos(yaw) - std::cos(prev_yaw));
  }
  return trajectory;
}

Trajectory generateTrajectoryWithConstantSteering(
  const double interval_distance, const double speed, const double steering_angle_rad,
  const size_t size, const double wheelbase)
{
  const auto curvature = std::tan(steering_angle_rad) / wheelbase;
  return generateTrajectoryWithConstantCurvature(
    interval_distance, speed, curvature, size, wheelbase);
}

Trajectory generateTrajectoryWithConstantSteeringRate(
  const double interval_distance, const double speed, const double steering_rate, const size_t size,
  const double wheelbase)
{
  Trajectory trajectory;
  trajectory.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  double x = 0.0, y = 0.0, yaw = 0.0, steering_angle_rad = 0.0;

  constexpr double MAX_STEERING_ANGLE_RAD = M_PI / 3.0;

  for (size_t i = 0; i <= size; ++i) {
    // Limit the steering angle to the maximum value
    steering_angle_rad =
      std::clamp(steering_angle_rad, -MAX_STEERING_ANGLE_RAD, MAX_STEERING_ANGLE_RAD);

    TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation = create_quaternion_from_yaw(yaw);
    p.longitudinal_velocity_mps = speed;
    p.front_wheel_angle_rad = steering_angle_rad;
    p.acceleration_mps2 = 0.0;

    trajectory.points.push_back(p);

    // Update x, y, yaw, and steering_angle for the next point
    const auto curvature = std::tan(steering_angle_rad) / wheelbase;
    double delta_yaw = curvature * interval_distance;
    yaw += delta_yaw;
    x += interval_distance * cos(yaw);
    y += interval_distance * sin(yaw);
    if (std::abs(speed) > 0.01) {
      steering_angle_rad += steering_rate * interval_distance / speed;
    } else {
      steering_angle_rad = steering_rate > 0.0 ? MAX_STEERING_ANGLE_RAD : -MAX_STEERING_ANGLE_RAD;
    }
  }

  return trajectory;
}

Trajectory generateShiftedTrajectory(
  const Trajectory & trajectory, const double lat_shift, const double lon_shift, const size_t size)
{
  Trajectory shifted_traj;
  shifted_traj.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();
  if (
    abs(lat_shift) <= std::numeric_limits<double>::epsilon() &&
    abs(lon_shift) <= std::numeric_limits<double>::epsilon() && size >= trajectory.points.size()) {
    shifted_traj.points = trajectory.points;
    return shifted_traj;
  }

  const auto nb_points = std::min(size, trajectory.points.size());
  shifted_traj.points = {trajectory.points.begin(), trajectory.points.begin() + nb_points};

  if (
    abs(lat_shift) > std::numeric_limits<double>::epsilon() ||
    abs(lon_shift) > std::numeric_limits<double>::epsilon()) {
    for (auto & t_p : shifted_traj.points) {
      t_p.pose = autoware_utils::calc_offset_pose(t_p.pose, lon_shift, lat_shift, 0.0, 0.0);
    }
  }

  return shifted_traj;
}

Trajectory generateNanTrajectory()
{
  Trajectory trajectory = generateTrajectory(1.0);
  trajectory.points.front().pose.position.x = NAN;
  return trajectory;
}

Trajectory generateInfTrajectory()
{
  Trajectory trajectory = generateTrajectory(1.0);
  trajectory.points.front().pose.position.x = INFINITY;
  return trajectory;
}

Trajectory generateBadCurvatureTrajectory()
{
  Trajectory trajectory;
  trajectory.header.stamp = rclcpp::Clock{RCL_ROS_TIME}.now();

  double y = 1.5;
  for (double s = 0.0; s <= 10.0; s += 1.0) {
    TrajectoryPoint p;
    p.longitudinal_velocity_mps = 1.0;
    p.pose.position.x = s;
    p.pose.position.y = y;
    y *= -1.0;  // invert sign
    trajectory.points.push_back(p);
  }

  return trajectory;
}

Odometry generateDefaultOdometry(const double x, const double y, const double vx)
{
  Odometry odom;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.twist.twist.linear.x = vx;
  return odom;
}

AccelWithCovarianceStamped generateDefaultAcceleration(const double ax)
{
  AccelWithCovarianceStamped acceleration;
  acceleration.accel.accel.linear.x = ax;
  return acceleration;
}

rclcpp::NodeOptions getNodeOptionsWithDefaultParams()
{
  rclcpp::NodeOptions node_options;

  // for planning validator
  node_options.append_parameter_override("planning_hz", PLANNING_HZ);
  node_options.append_parameter_override("handling_type.noncritical", 0);
  node_options.append_parameter_override("handling_type.critical", 2);
  node_options.append_parameter_override("publish_diag", true);
  node_options.append_parameter_override("diag_error_count_threshold", 0);
  node_options.append_parameter_override("display_on_terminal", true);
  node_options.append_parameter_override("enable_soft_stop_on_prev_traj", false);
  node_options.append_parameter_override("soft_stop_deceleration", -1.0);
  node_options.append_parameter_override("soft_stop_jerk_lim", 0.3);

  node_options.append_parameter_override("latency_checker.enable", true);
  node_options.append_parameter_override("latency_checker.threshold", THRESHOLD_LATENCY);
  node_options.append_parameter_override("latency_checker.is_critical", false);

  node_options.append_parameter_override("trajectory_checker.interval.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.interval.threshold", THRESHOLD_INTERVAL);
  node_options.append_parameter_override("trajectory_checker.interval.is_critical", false);

  node_options.append_parameter_override("trajectory_checker.relative_angle.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.relative_angle.threshold", THRESHOLD_RELATIVE_ANGLE);
  node_options.append_parameter_override("trajectory_checker.relative_angle.is_critical", false);

  node_options.append_parameter_override("trajectory_checker.curvature.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.curvature.threshold", THRESHOLD_CURVATURE);
  node_options.append_parameter_override("trajectory_checker.curvature.is_critical", false);

  node_options.append_parameter_override("trajectory_checker.lateral_accel.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.lateral_accel.threshold", THRESHOLD_LATERAL_ACC);
  node_options.append_parameter_override("trajectory_checker.lateral_accel.is_critical", false);

  node_options.append_parameter_override("trajectory_checker.max_lon_accel.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.max_lon_accel.threshold", THRESHOLD_LONGITUDINAL_MAX_ACC);
  node_options.append_parameter_override("trajectory_checker.max_lon_accel.is_critical", false);

  node_options.append_parameter_override("trajectory_checker.min_lon_accel.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.min_lon_accel.threshold", THRESHOLD_LONGITUDINAL_MIN_ACC);
  node_options.append_parameter_override("trajectory_checker.min_lon_accel.is_critical", false);

  node_options.append_parameter_override("trajectory_checker.lateral_jerk.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.lateral_jerk.threshold", THRESHOLD_LATERAL_JERK);
  node_options.append_parameter_override("trajectory_checker.lateral_jerk.is_critical", false);

  node_options.append_parameter_override("trajectory_checker.distance_deviation.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.distance_deviation.threshold", THRESHOLD_DISTANCE_DEVIATION);
  node_options.append_parameter_override(
    "trajectory_checker.distance_deviation.is_critical", false);

  node_options.append_parameter_override("trajectory_checker.lon_distance_deviation.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.lon_distance_deviation.threshold",
    THRESHOLD_LONGITUDINAL_DISTANCE_DEVIATION);
  node_options.append_parameter_override(
    "trajectory_checker.lon_distance_deviation.is_critical", false);

  node_options.append_parameter_override("trajectory_checker.velocity_deviation.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.velocity_deviation.threshold", THRESHOLD_VELOCITY_DEVIATION);
  node_options.append_parameter_override(
    "trajectory_checker.velocity_deviation.is_critical", false);

  node_options.append_parameter_override("trajectory_checker.yaw_deviation.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.yaw_deviation.threshold", THRESHOLD_YAW_DEVIATION);
  node_options.append_parameter_override("trajectory_checker.yaw_deviation.is_critical", false);

  node_options.append_parameter_override("trajectory_checker.steering.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.steering.threshold", THRESHOLD_STEERING);
  node_options.append_parameter_override("trajectory_checker.steering.is_critical", false);

  node_options.append_parameter_override("trajectory_checker.steering_rate.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.steering_rate.threshold", THRESHOLD_STEERING_RATE);
  node_options.append_parameter_override("trajectory_checker.steering_rate.is_critical", false);

  node_options.append_parameter_override(
    "trajectory_checker.forward_trajectory_length.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.forward_trajectory_length.acceleration",
    PARAMETER_FORWARD_TRAJECTORY_LENGTH_ACCELERATION);
  node_options.append_parameter_override(
    "trajectory_checker.forward_trajectory_length.margin",
    PARAMETER_FORWARD_TRAJECTORY_LENGTH_MARGIN);
  node_options.append_parameter_override(
    "trajectory_checker.forward_trajectory_length.is_critical", false);

  node_options.append_parameter_override("trajectory_checker.trajectory_shift.enable", true);
  node_options.append_parameter_override(
    "trajectory_checker.trajectory_shift.lat_shift_th", THRESHOLD_LATERAL_SHIFT);
  node_options.append_parameter_override(
    "trajectory_checker.trajectory_shift.forward_shift_th", THRESHOLD_FORWARD_SHIFT);
  node_options.append_parameter_override(
    "trajectory_checker.trajectory_shift.backward_shift_th", THRESHOLD_BACKWARD_SHIFT);
  node_options.append_parameter_override("trajectory_checker.trajectory_shift.is_critical", true);

  // for vehicle info
  node_options.append_parameter_override("wheel_radius", 0.5);
  node_options.append_parameter_override("wheel_width", 0.2);
  node_options.append_parameter_override("wheel_base", WHEELBASE);
  node_options.append_parameter_override("wheel_tread", 2.0);
  node_options.append_parameter_override("front_overhang", 1.0);
  node_options.append_parameter_override("rear_overhang", 1.0);
  node_options.append_parameter_override("left_overhang", 0.5);
  node_options.append_parameter_override("right_overhang", 0.5);
  node_options.append_parameter_override("vehicle_height", 1.5);
  node_options.append_parameter_override("max_steer_angle", 0.7);

  return node_options;
}

}  // namespace autoware::planning_validator::test_utils
