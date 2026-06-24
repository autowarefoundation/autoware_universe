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

#ifndef AUTOWARE__SPEED_SCALE_CORRECTOR__TYPES_HPP_
#define AUTOWARE__SPEED_SCALE_CORRECTOR__TYPES_HPP_

namespace autoware::speed_scale_corrector
{

/**
 * @brief Pose sample with timestamp
 */
struct TimestampedPose
{
  double time_sec{};  //!< Timestamp [s]
  double x{};         //!< Position x [m]
  double y{};         //!< Position y [m]
  double z{};         //!< Position z [m]
};

/**
 * @brief Vehicle velocity sample with timestamp
 */
struct TimestampedVelocity
{
  double time_sec{};              //!< Timestamp [s]
  double longitudinal_velocity{};  //!< Longitudinal velocity [m/s]
};

/**
 * @brief IMU sample with timestamp
 */
struct TimestampedImu
{
  double time_sec{};           //!< Timestamp [s]
  double angular_velocity_z{};  //!< Z-axis angular velocity [rad/s]
};

}  // namespace autoware::speed_scale_corrector

#endif  // AUTOWARE__SPEED_SCALE_CORRECTOR__TYPES_HPP_
