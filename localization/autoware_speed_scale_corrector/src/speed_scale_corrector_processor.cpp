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

#include "speed_scale_corrector_processor.hpp"

#include <autoware/speed_scale_corrector/types.hpp>

#include <fmt/format.h>

#include <cmath>
#include <string>

namespace autoware::speed_scale_corrector
{
namespace
{

TimestampedPose to_domain(const PoseStamped & pose)
{
  return {
    rclcpp::Time(pose.header.stamp).seconds(), pose.pose.position.x, pose.pose.position.y,
    pose.pose.position.z};
}

std::vector<TimestampedImu> to_domain_imus(
  const std::vector<Imu::ConstSharedPtr> & imu_ptrs)
{
  std::vector<TimestampedImu> domain_imus;
  domain_imus.reserve(imu_ptrs.size());
  for (const auto & imu_ptr : imu_ptrs) {
    domain_imus.push_back(
      {rclcpp::Time(imu_ptr->header.stamp).seconds(), imu_ptr->angular_velocity.z});
  }
  return domain_imus;
}

std::vector<TimestampedVelocity> to_domain_velocities(
  const std::vector<VelocityReport::ConstSharedPtr> & velocity_report_ptrs)
{
  std::vector<TimestampedVelocity> domain_velocities;
  domain_velocities.reserve(velocity_report_ptrs.size());
  for (const auto & velocity_report_ptr : velocity_report_ptrs) {
    domain_velocities.push_back(
      {rclcpp::Time(velocity_report_ptr->header.stamp).seconds(),
       velocity_report_ptr->longitudinal_velocity});
  }
  return domain_velocities;
}

std::string failure_reason_to_string(
  const UpdateFailureReason reason, const UpdateFailureContext & context)
{
  switch (reason) {
    case UpdateFailureReason::PoseEmpty:
      return "Pose is empty";
    case UpdateFailureReason::ImuEmpty:
      return "IMU is empty";
    case UpdateFailureReason::VelocityReportEmpty:
      return "Velocity report is empty";
    case UpdateFailureReason::WaitingForNextPose:
      return "Waiting for next pose";
    case UpdateFailureReason::TimeDifferenceTooLarge:
      return fmt::format(
        "Time difference is too large, time_diff: {:.3f}, threshold: {:.3f}",
        context.time_diff, context.time_diff_threshold);
    case UpdateFailureReason::VelocityReportTimestampMismatch:
      return fmt::format(
        "Velocity report timestamp mismatch, stamp_diff: {:.3f}, threshold: {:.3f}",
        context.stamp_diff, context.stamp_diff_threshold);
    case UpdateFailureReason::ImuTimestampMismatch:
      return fmt::format(
        "IMU timestamp mismatch, stamp_diff: {:.3f}, threshold: {:.3f}", context.stamp_diff,
        context.stamp_diff_threshold);
    case UpdateFailureReason::AngularVelocityTooHigh:
      return fmt::format(
        "Angular velocity is too high (IMU), angular velocity: {:.3f}, max angular velocity: "
        "{:.3f}",
        context.angular_velocity, context.max_angular_velocity);
    case UpdateFailureReason::VelocityTooHigh:
      return fmt::format(
        "Velocity is too high, velocity: {:.3f}, max velocity: {:.3f}", context.velocity,
        context.velocity_threshold);
    case UpdateFailureReason::VelocityTooLow:
      return fmt::format(
        "Velocity is too low, velocity: {:.3f}, min velocity: {:.3f}", context.velocity,
        context.velocity_threshold);
    case UpdateFailureReason::VelocityReportTooSmall:
      return fmt::format("Velocity report is too small: {:.3f}", context.velocity);
  }
  return "Unknown failure reason";
}

}  // namespace

SpeedScaleCorrectorProcessor::SpeedScaleCorrectorProcessor(
  const SpeedScaleEstimatorParameters & parameters)
: speed_scale_estimator_(parameters)
{
}

double SpeedScaleCorrectorProcessor::get_update_interval_sec() const
{
  return speed_scale_estimator_.get_update_interval_sec();
}

SpeedScaleCorrectorProcessResult SpeedScaleCorrectorProcessor::process(
  const std::vector<PoseStamped::ConstSharedPtr> & pose_ptrs,
  const std::vector<Imu::ConstSharedPtr> & imu_ptrs,
  const std::vector<VelocityReport::ConstSharedPtr> & velocity_report_ptrs)
{
  std::vector<TimestampedPose> domain_poses;
  domain_poses.reserve(pose_ptrs.size());
  for (const auto & pose_ptr : pose_ptrs) {
    domain_poses.push_back(to_domain(*pose_ptr));
  }

  SpeedScaleCorrectorProcessResult result;
  result.estimation_result = speed_scale_estimator_.update(
    domain_poses, to_domain_imus(imu_ptrs), to_domain_velocities(velocity_report_ptrs));
  result.updated = result.estimation_result.has_value();
  return result;
}

StringStamped SpeedScaleCorrectorProcessor::make_debug_info(
  const SpeedScaleCorrectorProcessResult & result, const rclcpp::Time & stamp) const
{
  StringStamped debug_info;
  debug_info.stamp = stamp;

  if (!result.estimation_result) {
    const auto & error = result.estimation_result.error();
    debug_info.data = fmt::format(
      "Not updated: {}\nEstimated speed scale factor: {}",
      failure_reason_to_string(error.reason, error.context),
      error.last_estimated_speed_scale_factor);
  } else {
    const auto & updated = result.estimation_result.value();
    debug_info.data = fmt::format(
      "Updated:\nEstimated speed scale factor: {}\nStd. dev.: {}\nVelocity from odometry: "
      "{}\nVelocity from velocity report: {}\nKalman gain: {}",
      updated.estimated_speed_scale_factor, std::sqrt(updated.covariance),
      updated.velocity_from_odometry, updated.velocity_from_velocity_report, updated.kalman_gain);
  }

  return debug_info;
}

Float32Stamped SpeedScaleCorrectorProcessor::make_scale_factor_msg(
  const SpeedScaleEstimatorUpdated & updated, const rclcpp::Time & stamp) const
{
  Float32Stamped msg;
  msg.stamp = stamp;
  msg.data = static_cast<float>(updated.estimated_speed_scale_factor);
  return msg;
}

}  // namespace autoware::speed_scale_corrector
