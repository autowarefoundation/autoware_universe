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

namespace autoware::speed_scale_corrector
{

double calc_time_diff(const TimestampedPose & pose_a, const TimestampedPose & pose_b)
{
  return pose_b.time_sec - pose_a.time_sec;
}

double calc_odometry_velocity(const TimestampedPose & pose_a, const TimestampedPose & pose_b)
{
  const double dt = calc_time_diff(pose_a, pose_b);
  if (std::abs(dt) < std::numeric_limits<double>::epsilon()) {
    return 0.0;
  }

  const double dx = pose_b.x - pose_a.x;
  const double dy = pose_b.y - pose_a.y;
  const double dz = pose_b.z - pose_a.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
}

std::optional<NearestImuSample> find_nearest_imu(
  const std::vector<TimestampedImu> & imus, const double target_time_sec)
{
  if (imus.empty()) {
    return std::nullopt;
  }

  const TimestampedImu * nearest_imu = &imus.front();
  double min_stamp_diff = std::abs(nearest_imu->time_sec - target_time_sec);

  for (const auto & imu : imus) {
    const double stamp_diff = std::abs(imu.time_sec - target_time_sec);
    if (stamp_diff < min_stamp_diff) {
      min_stamp_diff = stamp_diff;
      nearest_imu = &imu;
    }
  }

  NearestImuSample sample;
  sample.angular_velocity_z = nearest_imu->angular_velocity_z;
  sample.stamp_diff = min_stamp_diff;
  return sample;
}

std::optional<NearestVelocityReportSample> find_nearest_velocity_report(
  const std::vector<TimestampedVelocity> & velocity_reports, const double target_time_sec)
{
  if (velocity_reports.empty()) {
    return std::nullopt;
  }

  const TimestampedVelocity * nearest_velocity_report = &velocity_reports.front();
  double min_stamp_diff = std::abs(nearest_velocity_report->time_sec - target_time_sec);

  for (const auto & velocity_report : velocity_reports) {
    const double stamp_diff = std::abs(velocity_report.time_sec - target_time_sec);
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
