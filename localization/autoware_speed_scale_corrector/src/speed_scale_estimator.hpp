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

#ifndef SPEED_SCALE_ESTIMATOR_HPP_
#define SPEED_SCALE_ESTIMATOR_HPP_

#include "tl_expected/expected.hpp"

#include <autoware/speed_scale_corrector/types.hpp>

#include <optional>
#include <string>
#include <vector>

/**
 * @brief Speed scale corrector namespace
 */
namespace autoware::speed_scale_corrector
{

/**
 * @brief Parameters for speed scale estimator
 */
struct SpeedScaleEstimatorParameters
{
  double update_interval{};                        //!< Update interval [s]
  double initial_speed_scale_factor{};             //!< Initial scale factor
  double initial_speed_scale_factor_covariance{};  //!< Initial scale factor covariance (P)
  double process_noise_covariance{};               //!< Process noise covariance (Q)
  double measurement_noise_covariance{};           //!< Measurement noise covariance (R)
  double max_angular_velocity{};                   //!< Maximum angular velocity constraint [rad/s]
  double max_speed{};                              //!< Maximum speed constraint [m/s]
  double min_speed{};                              //!< Minimum speed constraint [m/s]
};

/**
 * @brief Result when estimation is successfully updated
 */
struct SpeedScaleEstimatorUpdated
{
  double estimated_speed_scale_factor = 0.0;   //!< Estimated speed scale factor
  double covariance = 0.0;                     //!< Covariance of the estimated speed scale factor
  double velocity_from_odometry = 0.0;         //!< Velocity from odometry [m/s]
  double velocity_from_velocity_report = 0.0;  //!< Velocity from velocity report [m/s]
  double kalman_gain = 0.0;                    //!< Kalman gain
  double time_diff = 0.0;                      //!< Time difference between the two updates [s]
};

/**
 * @brief Result when estimation is not updated
 */
struct SpeedScaleEstimatorNotUpdated
{
  std::string reason;                        //!< Reason for not updating
  double last_estimated_speed_scale_factor;  //!< Previous estimated speed scale factor
};

/**
 * @brief Speed scale estimator class
 *
 * Estimates speed scale factors by comparing odometry velocity with velocity
 * report measurements using a Kalman filter. The estimation is performed only
 * when all operational constraints are satisfied.
 */
class SpeedScaleEstimator
{
public:
  explicit SpeedScaleEstimator(const SpeedScaleEstimatorParameters & parameters);

  [[nodiscard]] double get_update_interval_sec() const;

  tl::expected<SpeedScaleEstimatorUpdated, SpeedScaleEstimatorNotUpdated> update(
    const std::vector<TimestampedPose> & poses, const std::vector<TimestampedImu> & imus,
    const std::vector<TimestampedVelocity> & velocity_reports);

private:
  SpeedScaleEstimatorParameters parameters_;
  std::optional<TimestampedPose> previous_pose_;
  double estimated_speed_scale_factor_ = 1.0;
  double covariance_ = 1.0;
};

}  // namespace autoware::speed_scale_corrector

#endif  // SPEED_SCALE_ESTIMATOR_HPP_
