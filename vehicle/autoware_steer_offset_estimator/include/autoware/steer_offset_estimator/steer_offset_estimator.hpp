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

#include <tl_expected/expected.hpp>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

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
 * @brief Result of steer offset estimation
 */
struct SteerOffsetEstimationUpdated
{
  double offset{0.0};                            ///< Estimated steering offset [rad]
  double covariance{0.0};                        ///< Estimation covariance
  double velocity{0.0};                          ///< Vehicle velocity [m/s]
  double angular_velocity{0.0};                  ///< Vehicle angular velocity [rad/s]
  double steering_angle{0.0};                    ///< Vehicle steering angle [rad]
  double coefficient{0.0};                       ///< Coefficient for covariance matrix
  double observed_angular_velocity_error{0.0};   ///< Observed angular velocity error [rad/s]
  double estimated_angular_velocity_error{0.0};  ///< Estimated angular velocity error [rad/s]
};

struct SteerOffsetEstimationNotUpdated
{
  std::string reason;
};

/**
 * @brief Parameters for steer offset estimation
 */
struct SteerOffsetEstimatorParameters
{
  double initial_covariance{1.0};  ///< Initial covariance value
  double initial_offset{0.0};      ///< Initial steering offset [rad]
  double wheel_base{0.0};          ///< Vehicle wheelbase [m]
  double min_velocity{2.0};        ///< Minimum valid velocity [m/s]
  double max_steer{0.5};           ///< Maximum valid steering angle [rad]
  double forgetting_factor{0.99};  ///< Forgetting factor for covariance matrix
};

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

  /**
   * @brief Get the current parameters
   * @return Current estimator parameters
   */
  [[nodiscard]] const SteerOffsetEstimatorParameters & get_parameters() const;

private:
  SteerOffsetEstimatorParameters params_;  ///< Estimator parameters
  double estimated_offset_;                ///< Current estimated offset [rad]
  double covariance_;                      ///< Current estimation covariance

  std::optional<geometry_msgs::msg::PoseStamped> previous_pose_;

#ifdef ENABLE_PLOTLY
  plotly::Figure figure_;
#endif
};

}  // namespace autoware::steer_offset_estimator

#endif  // AUTOWARE__STEER_OFFSET_ESTIMATOR__STEER_OFFSET_ESTIMATOR_HPP_
