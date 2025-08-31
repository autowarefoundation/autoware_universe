// Copyright 2018-2019 Autoware Foundation
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

#ifndef AUTOWARE__DEVIATION_ESTIMATOR__UTILS_HPP_
#define AUTOWARE__DEVIATION_ESTIMATOR__UTILS_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_internal_debug_msgs/msg/float64_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include <tf2/transform_datatypes.h>

#include <fstream>
#include <string>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2/utils.h>

namespace autoware::deviation_estimator
{

struct TrajectoryData
{
  std::vector<geometry_msgs::msg::PoseStamped> pose_list;
  std::vector<autoware_internal_debug_msgs::msg::Float64Stamped> vx_list;
  std::vector<geometry_msgs::msg::Vector3Stamped> gyro_list;
};

double double_round(const double x, const int n);

bool whether_to_use_data(
  const bool is_straight, const bool is_moving, const bool is_constant_velocity,
  const bool only_use_straight, const bool only_use_moving, const bool only_use_constant_velocity);

template <typename T>
double calculate_mean(const std::vector<T> & v)
{
  if (v.size() == 0) {
    return 0;
  }

  double mean = 0;
  for (const T & t : v) {
    mean += t;
  }
  mean /= v.size();
  return mean;
}

template <typename T>
double calculate_std(const std::vector<T> & v)
{
  if (v.size() == 0) {
    return 0;
  }

  const double mean = calculate_mean(v);
  double error = 0;
  for (const T & t : v) {
    error += pow(t - mean, 2);
  }
  return std::sqrt(error / v.size());
}

template <typename T>
double calculate_std_mean_const(const std::vector<T> & v, const double mean)
{
  if (v.size() == 0) {
    return 0;
  }

  double error = 0;
  for (const T & t : v) {
    error += pow(t - mean, 2);
  }
  return std::sqrt(error / v.size());
}

struct CompareMsgTimestamp
{
  bool operator()(
    autoware_internal_debug_msgs::msg::Float64Stamped const & t1, double const & t2) const
  {
    return rclcpp::Time(t1.stamp).seconds() < t2;
  }

  bool operator()(
    autoware_internal_debug_msgs::msg::Float64Stamped const & t1, rclcpp::Time const & t2) const
  {
    return rclcpp::Time(t1.stamp).seconds() < t2.seconds();
  }

  template <typename T1>
  bool operator()(T1 const & t1, double const & t2) const
  {
    return rclcpp::Time(t1.header.stamp).seconds() < t2;
  }

  template <typename T2>
  bool operator()(double const & t1, T2 const & t2) const
  {
    return t1 < rclcpp::Time(t2.header.stamp).seconds();
  }

  template <typename T1, typename T2>
  bool operator()(T1 const & t1, T2 const & t2) const
  {
    return rclcpp::Time(t1.header.stamp).seconds() < rclcpp::Time(t2.header.stamp).seconds();
  }

  template <typename T1>
  bool operator()(T1 const & t1, rclcpp::Time const & t2) const
  {
    return rclcpp::Time(t1.header.stamp).seconds() < t2.seconds();
  }

  template <typename T2>
  bool operator()(rclcpp::Time const & t1, T2 const & t2) const
  {
    return t1.seconds() < rclcpp::Time(t2.header.stamp).seconds();
  }
};

geometry_msgs::msg::Vector3 interpolate_vector3_stamped(
  const std::vector<geometry_msgs::msg::Vector3Stamped> & vec_list, const double time,
  const double tolerance_sec);

template <typename T>
std::vector<T> extract_sub_trajectory(
  const std::vector<T> & msg_list, const rclcpp::Time & t0, const rclcpp::Time & t1)
{
  const auto start_iter =
    std::lower_bound(msg_list.begin(), msg_list.end(), t0, CompareMsgTimestamp());
  const auto end_iter =
    std::lower_bound(msg_list.begin(), msg_list.end(), t1, CompareMsgTimestamp());
  std::vector<T> msg_list_sub(start_iter, end_iter);
  return msg_list_sub;
}

template <typename T, typename U>
double norm_xy(const T p1, const U p2)
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double clip_radian(const double rad);

geometry_msgs::msg::Point integrate_position(
  const std::vector<autoware_internal_debug_msgs::msg::Float64Stamped> & vx_list,
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list, const double coef_vx,
  const double yaw_init);

geometry_msgs::msg::Vector3 calculate_error_rpy(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list,
  const geometry_msgs::msg::Vector3 & gyro_bias);

geometry_msgs::msg::Vector3 integrate_orientation(
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list,
  const geometry_msgs::msg::Vector3 & gyro_bias);

double get_mean_abs_vx(
  const std::vector<autoware_internal_debug_msgs::msg::Float64Stamped> & vx_list);
double get_mean_abs_wz(const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list);
double get_mean_accel(
  const std::vector<autoware_internal_debug_msgs::msg::Float64Stamped> & vx_list);

geometry_msgs::msg::Vector3 transform_vector3(
  const geometry_msgs::msg::Vector3 & vec, const geometry_msgs::msg::TransformStamped & transform);

inline void myFromMsg(const geometry_msgs::msg::Transform & in, tf2::Transform & out);

geometry_msgs::msg::TransformStamped inverse_transform(
  const geometry_msgs::msg::TransformStamped & transform);

}  // namespace autoware::deviation_estimator

#endif  // AUTOWARE__DEVIATION_ESTIMATOR__UTILS_HPP_
