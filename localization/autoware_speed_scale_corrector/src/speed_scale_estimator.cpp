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

#include "speed_scale_estimator.hpp"

#include "utils.hpp"

#include <cmath>
#include <vector>

namespace autoware::speed_scale_corrector
{

SpeedScaleEstimator::SpeedScaleEstimator(const SpeedScaleEstimatorParameters & parameters)
: parameters_(parameters),
  estimated_speed_scale_factor_(parameters.initial_speed_scale_factor),
  covariance_(parameters.initial_speed_scale_factor_covariance)
{
}

double SpeedScaleEstimator::get_update_interval_sec() const
{
  return parameters_.update_interval;
}

tl::expected<SpeedScaleEstimatorUpdated, SpeedScaleEstimatorNotUpdated> SpeedScaleEstimator::update(
  const std::vector<TimestampedPose> & poses, const std::vector<TimestampedImu> & imus,
  const std::vector<TimestampedVelocity> & velocity_reports)
{
  if (poses.empty()) {
    return tl::make_unexpected(SpeedScaleEstimatorNotUpdated{
      UpdateFailureReason::PoseEmpty, {}, estimated_speed_scale_factor_});
  }

  if (imus.empty()) {
    return tl::make_unexpected(SpeedScaleEstimatorNotUpdated{
      UpdateFailureReason::ImuEmpty, {}, estimated_speed_scale_factor_});
  }

  if (velocity_reports.empty()) {
    return tl::make_unexpected(SpeedScaleEstimatorNotUpdated{
      UpdateFailureReason::VelocityReportEmpty, {}, estimated_speed_scale_factor_});
  }

  const auto & pose_curr = poses.back();

  if (!previous_pose_) {
    previous_pose_ = pose_curr;
    return tl::make_unexpected(SpeedScaleEstimatorNotUpdated{
      UpdateFailureReason::WaitingForNextPose, {}, estimated_speed_scale_factor_});
  }

  const auto & pose_prev = previous_pose_.value();

  const double time_diff = calc_time_diff(pose_prev, pose_curr);
  if (time_diff >= parameters_.update_interval * 2.0) {
    previous_pose_ = pose_curr;
    UpdateFailureContext context;
    context.time_diff = time_diff;
    context.time_diff_threshold = parameters_.update_interval * 2.0;
    return tl::make_unexpected(SpeedScaleEstimatorNotUpdated{
      UpdateFailureReason::TimeDifferenceTooLarge, context, estimated_speed_scale_factor_});
  }

  const double v_odometry = calc_odometry_velocity(pose_prev, pose_curr);

  const auto nearest_velocity_report =
    find_nearest_velocity_report(velocity_reports, pose_curr.time_sec);
  if (!nearest_velocity_report) {
    return tl::make_unexpected(SpeedScaleEstimatorNotUpdated{
      UpdateFailureReason::VelocityReportEmpty, {}, estimated_speed_scale_factor_});
  }

  if (nearest_velocity_report->stamp_diff > parameters_.update_interval) {
    UpdateFailureContext context;
    context.stamp_diff = nearest_velocity_report->stamp_diff;
    context.stamp_diff_threshold = parameters_.update_interval;
    return tl::make_unexpected(SpeedScaleEstimatorNotUpdated{
      UpdateFailureReason::VelocityReportTimestampMismatch, context,
      estimated_speed_scale_factor_});
  }

  const double v_report = nearest_velocity_report->longitudinal_velocity;

  const auto nearest_imu = find_nearest_imu(imus, pose_curr.time_sec);
  if (!nearest_imu) {
    return tl::make_unexpected(SpeedScaleEstimatorNotUpdated{
      UpdateFailureReason::ImuEmpty, {}, estimated_speed_scale_factor_});
  }

  if (nearest_imu->stamp_diff > parameters_.update_interval) {
    UpdateFailureContext context;
    context.stamp_diff = nearest_imu->stamp_diff;
    context.stamp_diff_threshold = parameters_.update_interval;
    return tl::make_unexpected(SpeedScaleEstimatorNotUpdated{
      UpdateFailureReason::ImuTimestampMismatch, context, estimated_speed_scale_factor_});
  }

  const double angular_velocity = nearest_imu->angular_velocity_z;
  if (std::abs(angular_velocity) > parameters_.max_angular_velocity) {
    UpdateFailureContext context;
    context.angular_velocity = angular_velocity;
    context.max_angular_velocity = parameters_.max_angular_velocity;
    return tl::make_unexpected(SpeedScaleEstimatorNotUpdated{
      UpdateFailureReason::AngularVelocityTooHigh, context, estimated_speed_scale_factor_});
  }

  if (v_odometry > parameters_.max_speed) {
    UpdateFailureContext context;
    context.velocity = v_odometry;
    context.velocity_threshold = parameters_.max_speed;
    return tl::make_unexpected(SpeedScaleEstimatorNotUpdated{
      UpdateFailureReason::VelocityTooHigh, context, estimated_speed_scale_factor_});
  }

  if (v_odometry < parameters_.min_speed) {
    UpdateFailureContext context;
    context.velocity = v_odometry;
    context.velocity_threshold = parameters_.min_speed;
    return tl::make_unexpected(SpeedScaleEstimatorNotUpdated{
      UpdateFailureReason::VelocityTooLow, context, estimated_speed_scale_factor_});
  }

  if (std::abs(v_report) < 1e-6) {
    UpdateFailureContext context;
    context.velocity = v_report;
    return tl::make_unexpected(SpeedScaleEstimatorNotUpdated{
      UpdateFailureReason::VelocityReportTooSmall, context, estimated_speed_scale_factor_});
  }

  const double z = v_odometry;

  const double x_pred = estimated_speed_scale_factor_;
  const double P_pred = covariance_ + parameters_.process_noise_covariance;  // NOLINT

  const double H = v_report;  // NOLINT
  const double innovation = z - H * x_pred;
  const double S = H * H * P_pred + parameters_.measurement_noise_covariance;  // NOLINT
  const double K = (P_pred * H) / S;                                           // NOLINT

  estimated_speed_scale_factor_ = x_pred + K * innovation;
  covariance_ = (1.0 - K * H) * P_pred;

  previous_pose_ = pose_curr;

  SpeedScaleEstimatorUpdated result;
  result.estimated_speed_scale_factor = estimated_speed_scale_factor_;
  result.covariance = covariance_;
  result.velocity_from_odometry = v_odometry;
  result.velocity_from_velocity_report = v_report;
  result.kalman_gain = K;
  result.time_diff = time_diff;

  return result;
}

}  // namespace autoware::speed_scale_corrector
