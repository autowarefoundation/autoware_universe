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

#ifndef AUTOWARE__STEER_OFFSET_ESTIMATOR__STEER_OFFSET_ESTIMATOR_HPP_
#define AUTOWARE__STEER_OFFSET_ESTIMATOR__STEER_OFFSET_ESTIMATOR_HPP_

#include "autoware/steer_offset_estimator/structs.hpp"

#include <rclcpp/rclcpp/time.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <deque>
#include <optional>
#include <string>
#include <vector>

#ifdef ENABLE_PLOTLY
#include "plotly/plotly.hpp"
#endif

/**
 * @brief Steer offset estimator namespace
 */
namespace autoware::steer_offset_estimator
{
using autoware_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TwistStamped;

/**
 * @brief Steer offset estimator class
 *
 * This class estimates steering wheel offset by comparing expected steering
 * angles calculated from vehicle motion with actual steering sensor readings.
 * The estimation uses recursive least squares with forgetting factor for
 * adaptive learning.
 *
 * The algorithm assumes that the relationship between steering angle and
 * yaw rate can be approximated as:
 *   wz = (v / wheelbase) * (steer + offset)
 *
 * Where:
 * - wz: measured yaw rate
 * - v: vehicle velocity
 * - wheelbase: vehicle wheelbase
 * - steer: measured steering angle
 * - offset: estimated steering offset
 */
class SteerOffsetEstimator
{
public:
  /**
   * @brief Constructor
   * @param parameters Configuration parameters for the estimator
   */
  explicit SteerOffsetEstimator(const SteerOffsetEstimatorParameters & parameters);

  /**
   * @brief Update estimation with new measurement data
   * @param input Measurement data containing pose and steering information
   * @return Expected result containing estimated offset or error information
   */
  tl::expected<SteerOffsetEstimationUpdated, SteerOffsetEstimationNotUpdated> update(
    const std::vector<PoseStamped> & poses, const std::vector<SteeringReport> & steers);

  [[nodiscard]] SteerOffsetEstimatorParameters get_parameters() const { return params_; }

private:
  SteerOffsetEstimatorParameters params_;  ///< Estimator parameters
  double estimated_offset_;                ///< Current estimated offset [rad]
  double covariance_;                      ///< Current estimation covariance

  /**
   * @brief Calculate current twist info from consecutive pose data
   * @param poses vector containing ego pose data
   * @result twist information based on current and previous pose
   */
  std::optional<geometry_msgs::msg::Twist> calculate_twist(const std::vector<PoseStamped> & poses);

  /**
   * @brief Update steering data buffer from latest steering report
   * @param steers vector containing steering report data
   */
  void update_steering_buffer(const std::vector<SteeringReport> & steers);

  /**
   * @brief Computes steering value at given time stamp
   * @details This function finds the nearest data within the steering buffer to the given timestamp
              and applies a windowed average around it to compute reliable steering value
   * @param steers avg steering value at given timestamp, if not found will return a nullopt
   */
  [[nodiscard]] std::optional<SteeringInfo> get_steering_at_timestamp(
    const rclcpp::Time & timestamp) const;

  /**
   * @brief Estimates steering offset based on current data
   * @details This function applies Kalman Filter on the given data to estimate the steering offset
              and updates the covariance and estimated_offset
   * @param velocity ego linear velocity computed for latest pose data
   * @param angular_velocity ego angular velocity computed for latest pose data
   * @param steering_angle steering value computed at timestamp corresponding to pose data
   * @result estimation result containing steering offset, covariance, and other data
   */
  SteerOffsetEstimationUpdated estimate_offset(
    const double velocity, const double angular_velocity, const double steering_angle);

  std::optional<geometry_msgs::msg::PoseStamped> previous_pose_;
  std::deque<autoware_vehicle_msgs::msg::SteeringReport> steering_buffer_;
  std::optional<SteeringInfo> previous_steering_;

#ifdef ENABLE_PLOTLY
  plotly::Figure figure_;
#endif
};

}  // namespace autoware::steer_offset_estimator

#endif  // AUTOWARE__STEER_OFFSET_ESTIMATOR__STEER_OFFSET_ESTIMATOR_HPP_
