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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <optional>
#include <vector>

namespace autoware::speed_scale_corrector
{
using autoware_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::PoseStamped;
using sensor_msgs::msg::Imu;

/**
 * @brief Nearest IMU sample to a target timestamp
 */
struct NearestImuSample
{
  double angular_velocity_z{};  //!< IMU z-axis angular velocity [rad/s]
  double stamp_diff{};          //!< Absolute time difference from target [s]
};

/**
 * @brief Nearest velocity report to a target timestamp
 */
struct NearestVelocityReportSample
{
  double longitudinal_velocity{};  //!< Longitudinal velocity [m/s]
  double stamp_diff{};             //!< Absolute time difference from target [s]
};

[[nodiscard]] double calc_time_diff(const PoseStamped & pose_a, const PoseStamped & pose_b);

[[nodiscard]] double calc_odometry_velocity(const PoseStamped & pose_a, const PoseStamped & pose_b);

[[nodiscard]] std::optional<NearestImuSample> find_nearest_imu(
  const std::vector<Imu> & imus, const rclcpp::Time & target_time);

[[nodiscard]] std::optional<NearestVelocityReportSample> find_nearest_velocity_report(
  const std::vector<VelocityReport> & velocity_reports, const rclcpp::Time & target_time);

}  // namespace autoware::speed_scale_corrector

#endif  // UTILS_HPP_
