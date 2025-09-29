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

#ifndef AUTOWARE__SPEED_SCALE_CORRECTOR__SPEED_SCALE_ESTIMATOR_HPP_
#define AUTOWARE__SPEED_SCALE_CORRECTOR__SPEED_SCALE_ESTIMATOR_HPP_

#include "autoware/trajectory/interpolator/interpolator.hpp"
#include "tl_expected/expected.hpp"

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <string>
#include <vector>

/**
 * @brief Speed scale corrector namespace
 */
namespace autoware::speed_scale_corrector
{

using autoware::experimental::trajectory::interpolator::InterpolatorInterface;
using autoware_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using sensor_msgs::msg::Imu;

/**
 * @brief Parameters for speed scale estimator
 */
struct SpeedScaleEstimatorParameters
{
  double time_window{};                 //!< Time window width used for estimation [s]
  double time_interval{};               //!< Data sampling interval [s]
  double initial_speed_scale_factor{};  //!< Initial scale factor
  double max_angular_velocity{};        //!< Maximum angular velocity constraint [rad/s]
  double max_speed{};                   //!< Maximum speed constraint [m/s]
  double min_speed{};                   //!< Minimum speed constraint [m/s]
  double max_speed_change{};            //!< Maximum speed change constraint [m/s²]
};

/**
 * @brief State vector for speed scale estimation
 */
struct SpeedScaleEstimatorState
{
  double distance = 0.0;                       //!< Distance between consecutive points [m]
  double distance_from_velocity_report = 0.0;  //!< Integrated distance from velocity reports [m]
  double angular_velocity = 0.0;               //!< Angular velocity from IMU [rad/s]
  double velocity_from_odometry = 0.0;         //!< Velocity calculated from odometry [m/s]
  double velocity_from_velocity_report = 0.0;  //!< Velocity from vehicle reports [m/s]
};

/**
 * @brief Result when estimation is successfully updated
 */
struct SpeedScaleEstimatorUpdated
{
  double estimated_speed_scale_factor;  //!< Estimated speed scale factor
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
 * Estimates speed scale factors by comparing odometry-based travel distance
 * with velocity report-based travel distance. The estimation is performed
 * only when all operational constraints are satisfied.
 */
class SpeedScaleEstimator
{
public:
  /**
   * @brief Constructor
   * @param parameters Estimation parameters
   */
  explicit SpeedScaleEstimator(const SpeedScaleEstimatorParameters & parameters);

  /**
   * @brief Update speed scale estimation
   * @param pose_with_covariance Pose information from odometry
   * @param imus IMU sensor data
   * @param velocity_reports Vehicle velocity reports
   * @return Expected result containing either updated estimation or error information
   */
  tl::expected<SpeedScaleEstimatorUpdated, SpeedScaleEstimatorNotUpdated> update(
    const PoseWithCovarianceStamped & pose_with_covariance, const std::vector<Imu> & imus,
    const std::vector<VelocityReport> & velocity_reports);

private:
  SpeedScaleEstimatorParameters parameters_;  //!< Estimation parameters

  std::vector<PoseWithCovarianceStamped> pose_with_covariance_buffer_;  //!< Pose data buffer
  std::vector<VelocityReport> velocity_report_buffer_;                  //!< Velocity report buffer
  std::vector<Imu> imu_buffer_;                                         //!< IMU data buffer

  int num_update_ = 0;                         //!< Number of successful updates
  double estimated_speed_scale_factor_ = 1.0;  //!< Current estimated scale factor
};

/**
 * @brief Create state vector from interpolated data
 * @param x_interpolator X position interpolator
 * @param y_interpolator Y position interpolator
 * @param angular_velocity_interpolator Angular velocity interpolator
 * @param velocity_interpolator Velocity interpolator
 * @param times Time samples
 * @return Vector of estimator states
 */
std::vector<SpeedScaleEstimatorState> create_states(
  const InterpolatorInterface<double> & x_interpolator,
  const InterpolatorInterface<double> & y_interpolator,
  const InterpolatorInterface<double> & angular_velocity_interpolator,
  const InterpolatorInterface<double> & velocity_interpolator, const std::vector<double> & times);

}  // namespace autoware::speed_scale_corrector

#endif  // AUTOWARE__SPEED_SCALE_CORRECTOR__SPEED_SCALE_ESTIMATOR_HPP_
