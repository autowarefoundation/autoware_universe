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

#include <autoware/speed_scale_corrector/types.hpp>

#include <optional>
#include <vector>

namespace autoware::speed_scale_corrector
{

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

/**
 * @brief Calculate time difference between two poses
 *
 * @param pose_a First pose (earlier timestamp)
 * @param pose_b Second pose (later timestamp)
 * @return Time difference in seconds
 */
[[nodiscard]] double calc_time_diff(const TimestampedPose & pose_a, const TimestampedPose & pose_b);

/**
 * @brief Calculate odometry velocity from pose difference
 *
 * @param pose_a First pose (earlier timestamp)
 * @param pose_b Second pose (later timestamp)
 * @return Magnitude of linear velocity [m/s]
 */
[[nodiscard]] double calc_odometry_velocity(
  const TimestampedPose & pose_a, const TimestampedPose & pose_b);

/**
 * @brief Find the IMU sample nearest to the target timestamp
 *
 * @param imus IMU samples (must not be empty)
 * @param target_time_sec Target timestamp [s]
 * @return Nearest IMU sample, or std::nullopt if imus is empty
 */
[[nodiscard]] std::optional<NearestImuSample> find_nearest_imu(
  const std::vector<TimestampedImu> & imus, double target_time_sec);

/**
 * @brief Find the velocity report nearest to the target timestamp
 *
 * @param velocity_reports Velocity samples (must not be empty)
 * @param target_time_sec Target timestamp [s]
 * @return Nearest velocity report sample, or std::nullopt if velocity_reports is empty
 */
[[nodiscard]] std::optional<NearestVelocityReportSample> find_nearest_velocity_report(
  const std::vector<TimestampedVelocity> & velocity_reports, double target_time_sec);

}  // namespace autoware::speed_scale_corrector

#endif  // UTILS_HPP_
