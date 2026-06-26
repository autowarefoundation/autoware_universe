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

#include <rclcpp/time.hpp>

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

SpeedScaleEstimatorNotUpdated SpeedScaleEstimator::make_not_updated(
  const UpdateFailureReason reason, const UpdateFailureContext & context) const
{
  return {reason, context, estimated_speed_scale_factor_};
}

void SpeedScaleEstimator::update_imu_buffer(const std::vector<Imu> & imus)
{
  for (const auto & imu : imus) {
    if (
      !imu_buffer_.empty() &&
      rclcpp::Time(imu.header.stamp) < rclcpp::Time(imu_buffer_.back().header.stamp)) {
      continue;
    }
    imu_buffer_.emplace_back(imu);
  }

  while (!imu_buffer_.empty() && (rclcpp::Time(imu_buffer_.back().header.stamp) -
                                  rclcpp::Time(imu_buffer_.front().header.stamp))
                                     .seconds() > parameters_.sensor_buffer_duration) {
    imu_buffer_.pop_front();
  }
}

void SpeedScaleEstimator::update_velocity_report_buffer(
  const std::vector<VelocityReport> & velocity_reports)
{
  for (const auto & velocity_report : velocity_reports) {
    if (
      !velocity_report_buffer_.empty() &&
      rclcpp::Time(velocity_report.header.stamp) <
        rclcpp::Time(velocity_report_buffer_.back().header.stamp)) {
      continue;
    }
    velocity_report_buffer_.emplace_back(velocity_report);
  }

  while (!velocity_report_buffer_.empty() &&
         (rclcpp::Time(velocity_report_buffer_.back().header.stamp) -
          rclcpp::Time(velocity_report_buffer_.front().header.stamp))
             .seconds() > parameters_.sensor_buffer_duration) {
    velocity_report_buffer_.pop_front();
  }
}

tl::expected<SpeedScaleEstimatorUpdated, SpeedScaleEstimatorNotUpdated> SpeedScaleEstimator::update(
  const std::vector<PoseStamped> & poses, const std::vector<Imu> & imus,
  const std::vector<VelocityReport> & velocity_reports)
{
  update_imu_buffer(imus);
  update_velocity_report_buffer(velocity_reports);

  if (poses.empty()) {
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::PoseEmpty));
  }

  if (imu_buffer_.empty()) {
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::ImuEmpty));
  }

  if (velocity_report_buffer_.empty()) {
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::VelocityReportEmpty));
  }

  const auto & pose_curr = poses.back();

  if (!previous_pose_) {
    previous_pose_ = pose_curr;
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::WaitingForNextPose));
  }

  const auto & pose_prev = previous_pose_.value();

  const double time_diff = calc_time_diff(pose_prev, pose_curr);
  if (time_diff > parameters_.max_pose_lag) {
    previous_pose_ = pose_curr;
    UpdateFailureContext context;
    context.time_diff = time_diff;
    context.time_diff_threshold = parameters_.max_pose_lag;
    return tl::make_unexpected(
      make_not_updated(UpdateFailureReason::TimeDifferenceTooLarge, context));
  }

  const double v_pose = calc_pose_velocity(pose_prev, pose_curr);

  const rclcpp::Time pose_time(pose_curr.header.stamp);

  const std::vector<VelocityReport> buffered_velocity_reports(
    velocity_report_buffer_.begin(), velocity_report_buffer_.end());
  const auto nearest_velocity_report =
    find_nearest_velocity_report(buffered_velocity_reports, pose_time);
  if (!nearest_velocity_report) {
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::VelocityReportEmpty));
  }

  if (nearest_velocity_report->stamp_diff > parameters_.max_stamp_lag) {
    UpdateFailureContext context;
    context.stamp_diff = nearest_velocity_report->stamp_diff;
    context.stamp_diff_threshold = parameters_.max_stamp_lag;
    return tl::make_unexpected(
      make_not_updated(UpdateFailureReason::VelocityReportTimestampMismatch, context));
  }

  const double v_wheel = nearest_velocity_report->longitudinal_velocity;

  const std::vector<Imu> buffered_imus(imu_buffer_.begin(), imu_buffer_.end());
  const auto nearest_imu = find_nearest_imu(buffered_imus, pose_time);
  if (!nearest_imu) {
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::ImuEmpty));
  }

  if (nearest_imu->stamp_diff > parameters_.max_stamp_lag) {
    UpdateFailureContext context;
    context.stamp_diff = nearest_imu->stamp_diff;
    context.stamp_diff_threshold = parameters_.max_stamp_lag;
    return tl::make_unexpected(
      make_not_updated(UpdateFailureReason::ImuTimestampMismatch, context));
  }

  const double angular_velocity = nearest_imu->angular_velocity_z;
  if (std::abs(angular_velocity) > parameters_.max_angular_velocity) {
    UpdateFailureContext context;
    context.angular_velocity = angular_velocity;
    context.max_angular_velocity = parameters_.max_angular_velocity;
    return tl::make_unexpected(
      make_not_updated(UpdateFailureReason::AngularVelocityTooHigh, context));
  }

  if (v_pose > parameters_.max_speed) {
    UpdateFailureContext context;
    context.velocity = v_pose;
    context.velocity_threshold = parameters_.max_speed;
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::VelocityTooHigh, context));
  }

  if (v_pose < parameters_.min_speed) {
    UpdateFailureContext context;
    context.velocity = v_pose;
    context.velocity_threshold = parameters_.min_speed;
    return tl::make_unexpected(make_not_updated(UpdateFailureReason::VelocityTooLow, context));
  }

  if (std::abs(v_wheel) < 1e-6) {
    UpdateFailureContext context;
    context.velocity = v_wheel;
    return tl::make_unexpected(
      make_not_updated(UpdateFailureReason::VelocityReportTooSmall, context));
  }

  const double z = v_pose;

  const double x_pred = estimated_speed_scale_factor_;
  const double P_pred = covariance_ + parameters_.process_noise_covariance;  // NOLINT

  const double H = v_wheel;  // NOLINT
  const double innovation = z - H * x_pred;
  const double S = H * H * P_pred + parameters_.measurement_noise_covariance;  // NOLINT
  const double K = (P_pred * H) / S;                                           // NOLINT

  estimated_speed_scale_factor_ = x_pred + K * innovation;
  covariance_ = (1.0 - K * H) * P_pred;

  previous_pose_ = pose_curr;

  SpeedScaleEstimatorUpdated result;
  result.estimated_speed_scale_factor = estimated_speed_scale_factor_;
  result.covariance = covariance_;
  result.velocity_from_pose = v_pose;
  result.velocity_from_wheel = v_wheel;
  result.kalman_gain = K;
  result.time_diff = time_diff;

  return result;
}

}  // namespace autoware::speed_scale_corrector
