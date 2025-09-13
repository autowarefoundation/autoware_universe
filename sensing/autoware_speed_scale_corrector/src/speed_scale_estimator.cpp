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
#include "autoware/trajectory/interpolator/cubic_spline.hpp"
#include "autoware/trajectory/interpolator/linear.hpp"

#include <fmt/core.h>
#include <fmt/format.h>

#include <string>
#include <tuple>
#include <vector>

namespace autoware::speed_scale_corrector
{
using autoware::experimental::trajectory::interpolator::CubicSpline;
using autoware::experimental::trajectory::interpolator::Linear;

namespace
{

/**
 * @brief Extract time series data from pose with covariance messages
 * @param pose_with_covariance_buffer Buffer containing pose messages
 * @return Tuple of (timestamps, x_positions, y_positions)
 */
std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> extract_time_series_data(
  const std::vector<PoseWithCovarianceStamped> & pose_with_covariance_buffer)
{
  std::vector<double> time_pose;
  std::vector<double> x;
  std::vector<double> y;

  for (const auto & pose : pose_with_covariance_buffer) {
    time_pose.emplace_back(pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9);
    x.emplace_back(pose.pose.pose.position.x);
    y.emplace_back(pose.pose.pose.position.y);
  }

  return std::make_tuple(time_pose, x, y);
}

/**
 * @brief Extract time series data from velocity report messages
 * @param velocity_report_buffer Buffer containing velocity report messages
 * @return Tuple of (timestamps, velocities)
 */
std::tuple<std::vector<double>, std::vector<double>> extract_time_series_data(
  const std::vector<VelocityReport> & velocity_report_buffer)
{
  std::vector<double> time_velocity;
  std::vector<double> velocity;

  for (const auto & velocity_report : velocity_report_buffer) {
    time_velocity.emplace_back(
      velocity_report.header.stamp.sec + velocity_report.header.stamp.nanosec * 1e-9);
    velocity.emplace_back(velocity_report.longitudinal_velocity);
  }

  return std::make_tuple(time_velocity, velocity);
}

/**
 * @brief Extract time series data from IMU messages
 * @param imu_buffer Buffer containing IMU messages
 * @return Tuple of (timestamps, angular_velocities)
 */
std::tuple<std::vector<double>, std::vector<double>> extract_time_series_data(
  const std::vector<Imu> & imu_buffer)
{
  std::vector<double> time_imu;
  std::vector<double> angular_velocity;

  for (const auto & imu : imu_buffer) {
    time_imu.emplace_back(imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9);
    angular_velocity.emplace_back(imu.angular_velocity.z);
  }

  return std::make_tuple(time_imu, angular_velocity);
}
}  // namespace

std::vector<SpeedScaleEstimatorState> create_states(
  const InterpolatorInterface<double> & x_interpolator,
  const InterpolatorInterface<double> & y_interpolator,
  const InterpolatorInterface<double> & angular_velocity_interpolator,
  const InterpolatorInterface<double> & velocity_interpolator, const std::vector<double> & times)
{
  std::vector<SpeedScaleEstimatorState> states;
  states.reserve(times.size());

  for (size_t i = 0; i < times.size(); ++i) {
    const double t = times[i];
    SpeedScaleEstimatorState state;

    // Get interpolated values at current time
    const double x = x_interpolator.compute(t);
    const double y = y_interpolator.compute(t);
    const double angular_velocity = angular_velocity_interpolator.compute(t);
    const double velocity = velocity_interpolator.compute(t);

    // Set basic state values
    state.angular_velocity = angular_velocity;
    state.velocity_from_velocity_report = velocity;

    // Velocity from position derivatives
    const double dxdt = x_interpolator.compute_first_derivative(t);
    const double dydt = y_interpolator.compute_first_derivative(t);
    state.velocity_from_odometry = std::sqrt(dxdt * dxdt + dydt * dydt);

    // Calculate odometry-based distance and velocity for non-initial points
    if (i > 0) {
      const double prev_t = times[i - 1];
      const double prev_x = x_interpolator.compute(prev_t);
      const double prev_y = y_interpolator.compute(prev_t);

      // Distance between consecutive points
      const double dx = x - prev_x;
      const double dy = y - prev_y;
      state.distance = std::sqrt(dx * dx + dy * dy);

      // Distance from velocity report using trapezoidal integration
      const double dt = t - prev_t;
      const double prev_velocity = velocity_interpolator.compute(prev_t);
      state.distance_from_velocity_report = (velocity + prev_velocity) * dt * 0.5;
    }

    states.emplace_back(state);
  }

  return states;
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

/**
 * @brief Check speed change constraint
 * @param states Vector of estimator states to check
 * @param parameters Estimation parameters containing constraints
 * @param current_speed_scale_factor Current estimated speed scale factor
 * @return Expected result indicating success or failure with reason
 */
tl::expected<CheckConstraintSuccess, CheckConstraintFailure> check_speed_change_constraint(
  const std::vector<SpeedScaleEstimatorState> & states,
  const SpeedScaleEstimatorParameters & parameters, const double current_speed_scale_factor)
{
  auto speed_change = current_speed_scale_factor * (states.front().velocity_from_odometry -
                                                    states.back().velocity_from_odometry);
  if (std::abs(speed_change) > parameters.max_speed_change) {
    return tl::make_unexpected(
      CheckConstraintFailure{fmt::format(
        "Speed change is too high, speed change: {:.3f}, max speed change: {:.3f}", speed_change,
        parameters.max_speed_change)});
  }
  return CheckConstraintSuccess{};
}

SpeedScaleEstimator::SpeedScaleEstimator(const SpeedScaleEstimatorParameters & parameters)
: parameters_(parameters)
{
}

tl::expected<SpeedScaleEstimatorUpdated, SpeedScaleEstimatorNotUpdated> SpeedScaleEstimator::update(
  const PoseWithCovarianceStamped & pose_with_covariance, const std::vector<Imu> & imus,
  const std::vector<VelocityReport> & velocity_reports)
{
  // Update internal buffers with new sensor data
  pose_with_covariance_buffer_.emplace_back(pose_with_covariance);
  imu_buffer_.insert(imu_buffer_.end(), imus.begin(), imus.end());
  velocity_report_buffer_.insert(
    velocity_report_buffer_.end(), velocity_reports.begin(), velocity_reports.end());

  // Extract time series data from buffered sensor messages
  auto [time_pose, x, y] = extract_time_series_data(pose_with_covariance_buffer_);
  auto [time_imu, angular_velocity] = extract_time_series_data(imu_buffer_);
  auto [time_velocity, velocity] = extract_time_series_data(velocity_report_buffer_);

  // Early return if any data stream is empty
  if (time_pose.empty()) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{
        "Pose with covariance is empty", estimated_speed_scale_factor_});
  }

  if (time_imu.empty()) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{"IMU is empty", estimated_speed_scale_factor_});
  }

  if (time_velocity.empty()) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{"Velocity report is empty", estimated_speed_scale_factor_});
  }

  // Find common time interval across all data streams
  auto intersection = intersect_intervals(
    {{time_pose.front(), time_pose.back()},
     {time_imu.front(), time_imu.back()},
     {time_velocity.front(), time_velocity.back()}});

  if (!intersection) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{
        "No common time interval found.", estimated_speed_scale_factor_});
  }

  auto [start, end] = intersection.value();

  // Check if time window is sufficient for estimation
  if (end - start < parameters_.time_window) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{
        fmt::format(
          "Time window is too short, current time window: {:.3f}, required time window: {:.3f}",
          end - start, parameters_.time_window),
        estimated_speed_scale_factor_});
  }

  // Apply Gaussian smoothing to noisy sensor data
  auto smoothed_angular_velocity = smooth_gaussian(time_imu, angular_velocity, 0.2);
  auto smoothed_velocity = smooth_gaussian(time_velocity, velocity, 0.2);

  // Create interpolators for trajectory data
  auto x_interpolator = CubicSpline::Builder().set_bases(time_pose).set_values(x).build();
  if (!x_interpolator) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{
        "X interpolator (CubicSpline) build failed", estimated_speed_scale_factor_});
  }

  auto y_interpolator = CubicSpline::Builder().set_bases(time_pose).set_values(y).build();
  if (!y_interpolator) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{
        "Y interpolator (CubicSpline) build failed", estimated_speed_scale_factor_});
  }

  auto angular_velocity_interpolator =
    Linear::Builder().set_bases(time_imu).set_values(smoothed_angular_velocity).build();
  if (!angular_velocity_interpolator) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{
        "Angular velocity interpolator (Linear) build failed", estimated_speed_scale_factor_});
  }

  auto velocity_interpolator =
    Linear::Builder().set_bases(time_velocity).set_values(smoothed_velocity).build();
  if (!velocity_interpolator) {
    return tl::make_unexpected(
      SpeedScaleEstimatorNotUpdated{
        "Velocity interpolator (Linear) build failed", estimated_speed_scale_factor_});
  }

  // Generate uniform time samples within the intersection interval
  auto times = arange(end - parameters_.time_window, end, parameters_.time_interval);

  // Create state vector from interpolated data
  auto states = create_states(
    *x_interpolator, *y_interpolator, *angular_velocity_interpolator, *velocity_interpolator,
    times);

  {
    auto result = check_angular_velocity_constraint(states, parameters_);
    if (!result) {
      return tl::make_unexpected(
        SpeedScaleEstimatorNotUpdated{result.error().reason, estimated_speed_scale_factor_});
    }
  }

  {
    auto result = check_velocity_constraint(states, parameters_);
    if (!result) {
      return tl::make_unexpected(
        SpeedScaleEstimatorNotUpdated{result.error().reason, estimated_speed_scale_factor_});
    }
  }

  {
    auto result = check_speed_change_constraint(states, parameters_, estimated_speed_scale_factor_);
    if (!result) {
      return tl::make_unexpected(
        SpeedScaleEstimatorNotUpdated{result.error().reason, estimated_speed_scale_factor_});
    }
  }

  // clear buffer
  imu_buffer_.clear();
  velocity_report_buffer_.clear();
  pose_with_covariance_buffer_.clear();

  num_update_++;

  double current_speed_scale_factor =
    states.back().distance / states.back().distance_from_velocity_report;
  estimated_speed_scale_factor_ =
    (estimated_speed_scale_factor_ * num_update_ + current_speed_scale_factor) / (num_update_ + 1);

  return SpeedScaleEstimatorUpdated{estimated_speed_scale_factor_};
}

}  // namespace autoware::speed_scale_corrector
