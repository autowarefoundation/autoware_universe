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

#include "autoware/steer_offset_estimator/steer_offset_estimator.hpp"

#include "autoware/steer_offset_estimator/utils.hpp"

#include <rclcpp/rclcpp/duration.hpp>
#include <rclcpp/rclcpp/time.hpp>
#include <tl_expected/expected.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iterator>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
namespace autoware::steer_offset_estimator
{

SteerOffsetEstimator::SteerOffsetEstimator(const SteerOffsetEstimatorParameters & parameters)
: params_(parameters),
  estimated_offset_(parameters.initial_offset),
  covariance_(parameters.initial_covariance)
{
}

tl::expected<SteerOffsetEstimationUpdated, SteerOffsetEstimationNotUpdated>
SteerOffsetEstimator::update(
  const std::vector<PoseStamped> & poses, const std::vector<SteeringReport> & steers)
{
  const auto unexpected = [](const std::string & reason) {
    return tl::make_unexpected(SteerOffsetEstimationNotUpdated{reason});
  };

  if (poses.empty()) return unexpected("poses is empty");

  if (steers.empty()) return unexpected("steers is empty");

  const auto twist = calculate_twist(poses);
  if (!twist) return unexpected("failed to compute twist");

  const double velocity = twist->linear.x;
  const double angular_velocity = twist->angular.z;

  update_steering_buffer(steers);
  const auto steering_info = get_steering_at_timestamp(rclcpp::Time(previous_pose_->header.stamp));

  // Validate input data quality
  if (velocity < params_.min_velocity) return unexpected("velocity is too low");
  if (std::abs(angular_velocity) > params_.max_ang_velocity)
    return unexpected("angular velocity is too high");
  if (!steering_info) return unexpected("steering angle is not available");

  if (std::abs(steering_info.value().steering) > params_.max_steer) {
    previous_steering_ = steering_info;
    return unexpected("steering angle is too large");
  }

  const auto steering_rate = std::invoke([this, steering_info]() -> double {
    if (!previous_steering_) return 0.0;
    const auto steering_dt =
      rclcpp::Duration(rclcpp::Time(steering_info->stamp) - rclcpp::Time(previous_steering_->stamp))
        .seconds();
    return steering_dt > 1e-6
             ? std::abs(steering_info->steering - previous_steering_->steering) / steering_dt
             : 0.0;
  });
  previous_steering_ = steering_info;

  if (steering_rate > params_.max_steer_rate) return unexpected("steering rate is too large");

  return estimate_offset(velocity, angular_velocity, steering_info.value().steering);
}

std::optional<geometry_msgs::msg::Twist> SteerOffsetEstimator::calculate_twist(
  const std::vector<PoseStamped> & poses)
{
  std::optional<geometry_msgs::msg::Twist> twist;

  // if previous pose is too old, reset value
  const double dt =
    previous_pose_.has_value()
      ? (rclcpp::Time(poses.back().header.stamp) - rclcpp::Time(previous_pose_->header.stamp))
          .seconds()
      : 0.0;
  if (dt > params_.max_pose_lag) previous_pose_ = std::nullopt;

  // If previous pose exists, use with current pose
  if (previous_pose_) {
    twist = utils::calc_twist_from_pose(previous_pose_.value(), poses.back());
    previous_pose_ = poses.back();
    return twist;
  }

  // First call - need at least 2 poses to calculate motion
  if (poses.size() < 2) {
    previous_pose_ = poses.back();
    return std::nullopt;
  }
  // Use first pose as previous, last pose as current
  twist = utils::calc_twist_from_pose(poses[0], poses.back());
  previous_pose_ = poses.back();

  return twist;
}

void SteerOffsetEstimator::update_steering_buffer(const std::vector<SteeringReport> & steers)
{
  for (const auto & steer : steers) {
    if (
      !steering_buffer_.empty() &&
      rclcpp::Time(steer.stamp) < rclcpp::Time(steering_buffer_.back().stamp)) {
      continue;
    }
    steering_buffer_.emplace_back(steer);
  }

  // Keep buffer size manageable
  while (!steering_buffer_.empty() && (rclcpp::Time(steering_buffer_.back().stamp) -
                                       rclcpp::Time(steering_buffer_.front().stamp))
                                          .seconds() > params_.max_steer_buffer) {
    steering_buffer_.pop_front();
  }
}

std::optional<SteeringInfo> SteerOffsetEstimator::get_steering_at_timestamp(
  const rclcpp::Time & timestamp) const
{
  if (steering_buffer_.empty()) return std::nullopt;

  // ensure latest steering info is not too old
  constexpr double eps = 1e-3;
  if ((timestamp - rclcpp::Time(steering_buffer_.back().stamp)).seconds() > eps) {
    return std::nullopt;
  }

  const auto upper = std::find_if(
    steering_buffer_.begin(), steering_buffer_.end(),
    [&timestamp](const auto & steer_rep) { return rclcpp::Time(steer_rep.stamp) > timestamp; });

  const auto pivot = (upper == steering_buffer_.begin()) ? upper : std::prev(upper);

  const int window = 2;
  auto start = (std::distance(steering_buffer_.begin(), pivot) > window) ? pivot - window
                                                                         : steering_buffer_.begin();
  auto finish = (std::distance(pivot, steering_buffer_.end()) > window) ? pivot + window + 1
                                                                        : steering_buffer_.end();

  const auto count = std::distance(start, finish);

  if (count == 0) return std::nullopt;

  SteeringInfo steering_info;

  double steering_sum = std::accumulate(start, finish, 0.0, [](double sum, const auto & steer_rep) {
    return sum + steer_rep.steering_tire_angle;
  });

  steering_info.steering = steering_sum / static_cast<double>(count);
  steering_info.stamp = pivot->stamp;
  return steering_info;
}

SteerOffsetEstimationUpdated SteerOffsetEstimator::estimate_offset(
  const double velocity, const double angular_velocity, const double steering_angle)
{
  // 1) Regressor and measurement for the regression model y = phi * theta + noise
  const double phi = velocity / params_.wheel_base;          // regressor
  const double y = angular_velocity - phi * steering_angle;  // measurement for regression

  // 2) Time update (process model): theta_k = theta_{k-1} + w,  w ~ N(0, Q)
  //    This inflates the PRIOR covariance to allow parameter drift.
  const double Q = params_.process_noise_covariance;  // NOLINT
  const double P_prior = covariance_ + Q;             // prior covariance // NOLINT

  // 3) Measurement update: compute denominator with PRIOR covariance
  //    denom = R + phi^2 * P_prior
  const double denom =
    std::max(params_.measurement_noise_covariance + phi * phi * P_prior, params_.denominator_floor);

  // 4) Kalman-like gain using PRIOR covariance
  const double K = (P_prior * phi) / denom;  // NOLINT

  // 5) Innovation (residual) and parameter update
  const double residual = y - phi * estimated_offset_;   // r = y - phi * theta_{k-1}
  estimated_offset_ = estimated_offset_ + K * residual;  // theta_k

  // 6) Covariance update (scalar Joseph form)
  //    P_k = P_prior - (P_prior * phi^2 * P_prior) / denom = (1 - K*phi) * P_prior
  const double P_new =  // NOLINT
    std::max(P_prior - (P_prior * phi * phi * P_prior) / denom, params_.covariance_floor);
  covariance_ = P_new;

  SteerOffsetEstimationUpdated updated_result;
  updated_result.offset = estimated_offset_;
  updated_result.covariance = covariance_;
  updated_result.velocity = velocity;
  updated_result.angular_velocity = angular_velocity;
  updated_result.steering_angle = steering_angle;
  updated_result.kalman_gain = K;
  updated_result.residual = residual;
  return updated_result;
}

}  // namespace autoware::steer_offset_estimator
