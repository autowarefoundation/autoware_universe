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

#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
using geometry_msgs::msg::PoseStamped;
using sensor_msgs::msg::Imu;

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

  static SpeedScaleEstimatorParameters load_parameters(rclcpp::Node * node);
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
  double estimated_speed_scale_factor = 0.0;   //!< Estimated speed scale factor
  double covariance = 0.0;                     //!< Covariance of the estimated speed scale factor
  double velocity_from_odometry = 0.0;         //!< Velocity from odometry [m/s]
  double velocity_from_velocity_report = 0.0;  //!< Velocity from velocity report [m/s]
  double kalman_gain = 0.0;                    //!< Kalman gain
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
   * @param poses Pose information from odometry
   * @param imus IMU sensor data
   * @param velocity_reports Vehicle velocity reports
   * @return Expected result containing either updated estimation or error information
   */
  tl::expected<SpeedScaleEstimatorUpdated, SpeedScaleEstimatorNotUpdated> update(
    const std::vector<PoseStamped> & poses, const std::vector<Imu> & imus,
    const std::vector<VelocityReport> & velocity_reports);

private:
  SpeedScaleEstimatorParameters parameters_;  //!< Estimation parameters

  std::optional<PoseStamped> previous_pose_;  //!< Previous pose for velocity calculation

  double estimated_speed_scale_factor_ = 1.0;  //!< Current estimated scale factor (state x)
  double covariance_ = 1.0;                    //!< State covariance (P)
};

}  // namespace autoware::speed_scale_corrector

#endif  // AUTOWARE__SPEED_SCALE_CORRECTOR__SPEED_SCALE_ESTIMATOR_HPP_
