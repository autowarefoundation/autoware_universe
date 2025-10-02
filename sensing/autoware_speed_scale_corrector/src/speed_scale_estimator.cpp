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

#include "autoware/speed_scale_corrector/speed_scale_estimator.hpp"

#include "autoware/speed_scale_corrector/utils.hpp"

#include <fmt/format.h>

#include <string>
#include <vector>

namespace autoware::speed_scale_corrector
{
SpeedScaleEstimatorParameters SpeedScaleEstimatorParameters::load_parameters(rclcpp::Node * node)
{
  SpeedScaleEstimatorParameters parameters;
  parameters.update_interval = node->declare_parameter<double>("update_interval");
  parameters.initial_speed_scale_factor =
    node->declare_parameter<double>("initial_speed_scale_factor");
  parameters.initial_speed_scale_factor_covariance =
    node->declare_parameter<double>("initial_speed_scale_factor_covariance");
  parameters.process_noise_covariance = node->declare_parameter<double>("process_noise_covariance");
  parameters.measurement_noise_covariance =
    node->declare_parameter<double>("measurement_noise_covariance");
  parameters.max_angular_velocity = node->declare_parameter<double>("max_angular_velocity");
  parameters.max_speed = node->declare_parameter<double>("max_speed");
  parameters.min_speed = node->declare_parameter<double>("min_speed");
  return parameters;
}

/**
 * @brief Result type for successful constraint check
 */
struct CheckConstraintSuccess
{
};

/**
 * @brief Result type for failed constraint check
 */
struct CheckConstraintFailure
{
  std::string reason;  //!< Reason for constraint failure
};

/**
 * @brief Check angular velocity constraint
 * @param states Vector of estimator states to check
 * @param parameters Estimation parameters containing constraints
 * @return Expected result indicating success or failure with reason
 */
tl::expected<CheckConstraintSuccess, CheckConstraintFailure> check_angular_velocity_constraint(
  const std::vector<SpeedScaleEstimatorState> & states,
  const SpeedScaleEstimatorParameters & parameters)
{
  for (const auto & state : states) {
    if (std::abs(state.angular_velocity) > parameters.max_angular_velocity) {
      return tl::make_unexpected(
        CheckConstraintFailure{fmt::format(
          "Angular velocity is too high, angular velocity: {:.3f}, max angular velocity: {:.3f}",
          state.angular_velocity, parameters.max_angular_velocity)});
    }
  }
  return CheckConstraintSuccess{};
}

/**
 * @brief Check velocity constraints (minimum and maximum)
 * @param states Vector of estimator states to check
 * @param parameters Estimation parameters containing constraints
 * @return Expected result indicating success or failure with reason
 */
tl::expected<CheckConstraintSuccess, CheckConstraintFailure> check_velocity_constraint(
  const std::vector<SpeedScaleEstimatorState> & states,
  const SpeedScaleEstimatorParameters & parameters)
{
  for (const auto & state : states) {
    if (state.velocity_from_odometry > parameters.max_speed) {
      return tl::make_unexpected(
        CheckConstraintFailure{fmt::format(
          "Velocity is too high, velocity: {:.3f}, max velocity: {:.3f}",
          state.velocity_from_odometry, parameters.max_speed)});
    }
    if (state.velocity_from_odometry < parameters.min_speed) {
      return tl::make_unexpected(
        CheckConstraintFailure{fmt::format(
          "Velocity is too low, velocity: {:.3f}, min velocity: {:.3f}",
          state.velocity_from_odometry, parameters.min_speed)});
    }
  }
  return CheckConstraintSuccess{};
}

SpeedScaleEstimator::SpeedScaleEstimator(const SpeedScaleEstimatorParameters & parameters)
: parameters_(parameters),
  estimated_speed_scale_factor_(parameters.initial_speed_scale_factor),
  covariance_(parameters.initial_speed_scale_factor_covariance)
{
}

rclcpp::Duration SpeedScaleEstimator::get_update_interval() const
{
  return std::chrono::duration<double>(parameters_.update_interval);
}

tl::expected<SpeedScaleEstimatorUpdated, SpeedScaleEstimatorNotUpdated> SpeedScaleEstimator::update(
  const std::vector<PoseStamped> & poses, const std::vector<Imu> & imus,
  const std::vector<VelocityReport> & velocity_reports)
{
  // Check if input data is available
  if (poses.empty()) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{"Pose is empty", estimated_speed_scale_factor_});
  }

  if (imus.empty()) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{"IMU is empty", estimated_speed_scale_factor_});
  }

  if (velocity_reports.empty()) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{"Velocity report is empty", estimated_speed_scale_factor_});
  }

  // Need previous pose to calculate velocity
  if (!previous_pose_) {
    previous_pose_ = poses.back();
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{"Waiting for next pose", estimated_speed_scale_factor_});
  }

  // Get the current and previous poses
  const auto & pose_curr = poses.back();
  const auto & pose_prev = previous_pose_.value();

  // Check time difference
  const double time_diff = calc_time_diff(pose_prev, pose_curr);
  if (time_diff >= parameters_.update_interval * 2.0) {
    previous_pose_ = pose_curr;
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{
        fmt::format(
          "Time difference is too large, time_diff: {:.3f}, threshold: {:.3f}", time_diff,
          parameters_.update_interval * 2.0),
        estimated_speed_scale_factor_});
  }

  // Calculate twist (velocity and angular velocity) from pose difference
  const auto twist = calc_twist_from_pose(pose_prev, pose_curr);
  const double v_odometry = twist.linear.x;
  const double angular_velocity = twist.angular.z;

  // Get velocity from velocity report (use latest value)
  const double v_report = velocity_reports.back().longitudinal_velocity;

  // Check constraints
  if (std::abs(angular_velocity) > parameters_.max_angular_velocity) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{
        fmt::format(
          "Angular velocity is too high, angular velocity: {:.3f}, max angular velocity: {:.3f}",
          angular_velocity, parameters_.max_angular_velocity),
        estimated_speed_scale_factor_});
  }

  if (v_odometry > parameters_.max_speed) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{
        fmt::format(
          "Velocity is too high, velocity: {:.3f}, max velocity: {:.3f}", v_odometry,
          parameters_.max_speed),
        estimated_speed_scale_factor_});
  }

  if (v_odometry < parameters_.min_speed) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{
        fmt::format(
          "Velocity is too low, velocity: {:.3f}, min velocity: {:.3f}", v_odometry,
          parameters_.min_speed),
        estimated_speed_scale_factor_});
  }

  if (std::abs(v_report) < 1e-6) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{
        fmt::format("Velocity report is too small: {:.3f}", v_report),
        estimated_speed_scale_factor_});
  }

  // Observation model: z = estimated_speed_scale_factor * v_report
  // Observation: z = v_odometry
  const double z = v_odometry;

  // Kalman Filter: Prediction Step
  // x_pred(k) = x(k-1)
  // P_pred(k) = P(k-1) + Q
  const double x_pred = estimated_speed_scale_factor_;
  const double P_pred = covariance_ + parameters_.process_noise_covariance;  // NOLINT

  // Kalman Filter: Update Step
  // Observation matrix: H = v_report
  // Innovation: y = z - H * x_pred = v_odometry - v_report * x_pred
  // Innovation covariance: S = H * P_pred * H + R = v_report^2 * P_pred + R
  // Kalman gain: K = P_pred * H / S = (P_pred * v_report) / (v_report^2 * P_pred + R)
  // State update: x(k) = x_pred + K * y
  // Covariance update: P(k) = (1 - K * H) * P_pred = (1 - K * v_report) * P_pred
  const double H = v_report;  // NOLINT
  const double innovation = z - H * x_pred;
  const double S = H * H * P_pred + parameters_.measurement_noise_covariance;  // NOLINT
  const double K = (P_pred * H) / S;                                           // NOLINT

  estimated_speed_scale_factor_ = x_pred + K * innovation;
  covariance_ = (1.0 - K * H) * P_pred;

  // Update previous pose for next iteration
  previous_pose_ = pose_curr;

  SpeedScaleEstimatorUpdated result;
  result.estimated_speed_scale_factor = estimated_speed_scale_factor_;
  result.covariance = covariance_;
  result.velocity_from_odometry = v_odometry;
  result.velocity_from_velocity_report = v_report;
  result.kalman_gain = K;

  return result;
}

}  // namespace autoware::speed_scale_corrector
