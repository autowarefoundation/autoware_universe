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

#include "autoware/deviation_estimator/velocity_coef_module.hpp"

#include "autoware/deviation_estimator/utils.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

namespace autoware::deviation_estimator
{

/**
 * @brief update speed scale factor (or velocity coefficient) based on a given trajectory data
 */
void VelocityCoefModule::update_coef(const TrajectoryData & traj_data)
{
  const rclcpp::Time t0_rclcpp_time = rclcpp::Time(traj_data.pose_list.front().header.stamp);
  const rclcpp::Time t1_rclcpp_time = rclcpp::Time(traj_data.pose_list.back().header.stamp);

  auto d_pos = integrate_position(
    traj_data.vx_list, traj_data.gyro_list, 1.0,
    tf2::getYaw(traj_data.pose_list.front().pose.orientation));
  const double dt_pose = (rclcpp::Time(traj_data.pose_list.back().header.stamp) -
                          rclcpp::Time(traj_data.pose_list.front().header.stamp))
                           .seconds();
  const double dt_velocity =
    (rclcpp::Time(traj_data.vx_list.back().stamp) - rclcpp::Time(traj_data.vx_list.front().stamp))
      .seconds();
  d_pos.x *= dt_pose / dt_velocity;
  d_pos.y *= dt_pose / dt_velocity;

  const double dx =
    traj_data.pose_list.back().pose.position.x - traj_data.pose_list.front().pose.position.x;
  const double dy =
    traj_data.pose_list.back().pose.position.y - traj_data.pose_list.front().pose.position.y;
  if (d_pos.x * d_pos.x + d_pos.y * d_pos.y == 0) return;

  const double d_coef_vx = (d_pos.x * dx + d_pos.y * dy) / (d_pos.x * d_pos.x + d_pos.y * d_pos.y);

  coef_vx_.first += d_coef_vx;
  coef_vx_.second += 1;
  coef_vx_list_.push_back(d_coef_vx);
}

/**
 * @brief getter function for current estimated coefficient
 */
double VelocityCoefModule::get_coef() const
{
  if (coef_vx_.second == 0) return 1.0;
  return coef_vx_.first / coef_vx_.second;
}

/**
 * @brief getter function for current estimated standard deviation of coefficient
 */
double VelocityCoefModule::get_coef_std() const
{
  return calculate_std(coef_vx_list_);
}

bool VelocityCoefModule::empty() const
{
  return coef_vx_.second == 0;
}

}  // namespace autoware::deviation_estimator
