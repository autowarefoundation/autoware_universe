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

#include "utils.hpp"

#include <cmath>
#include <limits>
#include <vector>

namespace autoware::speed_scale_corrector
{

double calc_time_diff(const PoseStamped & pose_a, const PoseStamped & pose_b)
{
  return (rclcpp::Time(pose_b.header.stamp) - rclcpp::Time(pose_a.header.stamp)).seconds();
}

double calc_odometry_velocity(const PoseStamped & pose_a, const PoseStamped & pose_b)
{
  const double dt = calc_time_diff(pose_a, pose_b);
  if (std::abs(dt) < std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  const double dx = pose_b.pose.position.x - pose_a.pose.position.x;
  const double dy = pose_b.pose.position.y - pose_a.pose.position.y;
  const double dz = pose_b.pose.position.z - pose_a.pose.position.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
}

std::optional<NearestImuSample> find_nearest_imu(
  const std::vector<Imu> & imus, const rclcpp::Time & target_time)
{
  if (imus.empty()) {
    return std::nullopt;
  }

  const Imu * nearest_imu = &imus.front();
  double min_stamp_diff =
    std::abs((rclcpp::Time(nearest_imu->header.stamp) - target_time).seconds());

  for (const auto & imu : imus) {
    const double stamp_diff = std::abs((rclcpp::Time(imu.header.stamp) - target_time).seconds());
    if (stamp_diff < min_stamp_diff) {
      min_stamp_diff = stamp_diff;
      nearest_imu = &imu;
    }
  }

  NearestImuSample sample;
  sample.angular_velocity_z = nearest_imu->angular_velocity.z;
  sample.stamp_diff = min_stamp_diff;
  return sample;
}

std::optional<NearestVelocityReportSample> find_nearest_velocity_report(
  const std::vector<VelocityReport> & velocity_reports, const rclcpp::Time & target_time)
{
  if (velocity_reports.empty()) {
    return std::nullopt;
  }

  const VelocityReport * nearest_velocity_report = &velocity_reports.front();
  double min_stamp_diff =
    std::abs((rclcpp::Time(nearest_velocity_report->header.stamp) - target_time).seconds());

  for (const auto & velocity_report : velocity_reports) {
    const double stamp_diff =
      std::abs((rclcpp::Time(velocity_report.header.stamp) - target_time).seconds());
    if (stamp_diff < min_stamp_diff) {
      min_stamp_diff = stamp_diff;
      nearest_velocity_report = &velocity_report;
    }
  }

  NearestVelocityReportSample sample;
  sample.longitudinal_velocity = nearest_velocity_report->longitudinal_velocity;
  sample.stamp_diff = min_stamp_diff;
  return sample;
}

}  // namespace autoware::speed_scale_corrector
