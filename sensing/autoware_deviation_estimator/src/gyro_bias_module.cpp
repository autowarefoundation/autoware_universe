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

#include "autoware/deviation_estimator/gyro_bias_module.hpp"

#include "autoware/deviation_estimator/utils.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <vector>

namespace autoware::deviation_estimator
{

/**
 * @brief update gyroscope bias based on a given trajectory data
 */
void GyroBiasModule::update_bias(const TrajectoryData & traj_data)
{
  const rclcpp::Time t0_rclcpp_time = rclcpp::Time(traj_data.pose_list.front().header.stamp);
  const rclcpp::Time t1_rclcpp_time = rclcpp::Time(traj_data.pose_list.back().header.stamp);
  const double dt = t1_rclcpp_time.seconds() - t0_rclcpp_time.seconds();

  auto error_rpy =
    calculate_error_rpy(traj_data.pose_list, traj_data.gyro_list, geometry_msgs::msg::Vector3{});
  const double dt_pose = (rclcpp::Time(traj_data.pose_list.back().header.stamp) -
                          rclcpp::Time(traj_data.pose_list.front().header.stamp))
                           .seconds();
  const double dt_gyro = (rclcpp::Time(traj_data.gyro_list.back().header.stamp) -
                          rclcpp::Time(traj_data.gyro_list.front().header.stamp))
                           .seconds();
  error_rpy.x *= dt_pose / dt_gyro;
  error_rpy.y *= dt_pose / dt_gyro;
  error_rpy.z *= dt_pose / dt_gyro;

  gyro_bias_pair_.first.x += dt * error_rpy.x;
  gyro_bias_pair_.first.y += dt * error_rpy.y;
  gyro_bias_pair_.first.z += dt * error_rpy.z;
  gyro_bias_pair_.second.x += dt * dt;
  gyro_bias_pair_.second.y += dt * dt;
  gyro_bias_pair_.second.z += dt * dt;

  geometry_msgs::msg::Vector3 gyro_bias;
  gyro_bias.x = error_rpy.x / dt;
  gyro_bias.y = error_rpy.y / dt;
  gyro_bias.z = error_rpy.z / dt;
  gyro_bias_list_.push_back(gyro_bias);
}

/**
 * @brief getter function for current estimated bias
 */
geometry_msgs::msg::Vector3 GyroBiasModule::get_bias_base_link() const
{
  geometry_msgs::msg::Vector3 gyro_bias_base;
  gyro_bias_base.x = gyro_bias_pair_.first.x / gyro_bias_pair_.second.x;
  gyro_bias_base.y = gyro_bias_pair_.first.y / gyro_bias_pair_.second.y;
  gyro_bias_base.z = gyro_bias_pair_.first.z / gyro_bias_pair_.second.z;
  return gyro_bias_base;
}

/**
 * @brief getter function for current standard deviation of estimated bias
 */
geometry_msgs::msg::Vector3 GyroBiasModule::get_bias_std() const
{
  std::vector<double> stddev_bias_list_x, stddev_bias_list_y, stddev_bias_list_z;
  for (const auto & e : gyro_bias_list_) {
    stddev_bias_list_x.push_back(e.x);
    stddev_bias_list_y.push_back(e.y);
    stddev_bias_list_z.push_back(e.z);
  }
  geometry_msgs::msg::Vector3 stddev_bias;
  stddev_bias.x = calculate_std_mean_const(
    stddev_bias_list_x, gyro_bias_pair_.first.x / gyro_bias_pair_.second.x);
  stddev_bias.y = calculate_std_mean_const(
    stddev_bias_list_y, gyro_bias_pair_.first.y / gyro_bias_pair_.second.y);
  stddev_bias.z = calculate_std_mean_const(
    stddev_bias_list_z, gyro_bias_pair_.first.z / gyro_bias_pair_.second.z);
  return stddev_bias;
}

bool GyroBiasModule::empty() const
{
  return gyro_bias_list_.empty();
}

}  // namespace autoware::deviation_estimator
