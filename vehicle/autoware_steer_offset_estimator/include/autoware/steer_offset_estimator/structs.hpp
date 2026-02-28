// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__STEER_OFFSET_ESTIMATOR__STRUCTS_HPP_
#define AUTOWARE__STEER_OFFSET_ESTIMATOR__STRUCTS_HPP_

#include <rclcpp/rclcpp/time.hpp>

#include <string>
#include <unordered_map>

namespace autoware::steer_offset_estimator
{

struct SteerOffsetEstimationUpdated
{
  double offset{0.0};            ///< Estimated steering offset [rad]
  double covariance{0.0};        ///< Estimation covariance
  double velocity{0.0};          ///< Vehicle velocity [m/s]
  double angular_velocity{0.0};  ///< Vehicle angular velocity [rad/s]
  double steering_angle{0.0};    ///< Vehicle steering angle [rad]
  double kalman_gain{0.0};       ///< Coefficient for covariance matrix
  double residual{0.0};          ///< Residual [rad/s]
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
  double initial_covariance{1.0};             ///< Initial covariance value
  double initial_offset{0.0};                 ///< Initial steering offset [rad]
  double wheel_base{0.0};                     ///< Vehicle wheelbase [m]
  double min_velocity{2.0};                   ///< Minimum valid velocity [m/s]
  double max_steer{0.5};                      ///< Maximum valid steering angle [rad]
  double max_steer_rate{0.5};                 ///< Maximum valid steering angle rate [rad/s]
  double max_ang_velocity{0.5};               ///< Maximum valid angular velocity [rad/s]
  double measurement_noise_covariance{0.01};  ///< Measurement noise covariance
  double process_noise_covariance{0.01};      ///< Process noise covariance
  double denominator_floor{1.0e-12};          ///< Denominator floor value
  double covariance_floor{1.0e-12};           ///< Covariance floor value
  double max_steer_buffer{1.0};               ///< Steering buffer duration [s]
  double max_pose_lag{1.0};                   ///< Max valid pose age [s]
};

enum class CalibrationMode : uint8_t { OFF, MANUAL, AUTO };

struct SteerOffsetCalibrationParameters
{
  CalibrationMode mode{CalibrationMode::OFF};
  double update_offset_th{0.001};
  double covariance_th{0.0008};
  double min_steady_duration{10.0};
  double min_velocity{1.0};
  double max_offset_limit{0.05};
  double min_update_interval{100.0};
  double warning_offset_th{0.005};
  std::string steer_offset_param_path;
  std::string steer_offset_param_name;
  std::string calibration_file_path;
  std::string calibration_param_name;

  static inline const std::unordered_map<std::string, CalibrationMode> mode_map{
    {"off", CalibrationMode::OFF},
    {"manual", CalibrationMode::MANUAL},
    {"auto", CalibrationMode::AUTO}};

  static inline const std::unordered_map<CalibrationMode, std::string> mode_to_str_map{
    {CalibrationMode::OFF, "off"},
    {CalibrationMode::MANUAL, "manual"},
    {CalibrationMode::AUTO, "auto"}};
};

struct SteeringInfo
{
  rclcpp::Time stamp;
  double steering;
};

}  // namespace autoware::steer_offset_estimator

#endif  // AUTOWARE__STEER_OFFSET_ESTIMATOR__STRUCTS_HPP_
