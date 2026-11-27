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

#ifndef STRUCTS_HPP_
#define STRUCTS_HPP_

namespace autoware::speed_scale_estimator
{

struct SpeedScaleEstimatorParameters
{
  double update_interval{};
  double max_pose_lag{};
  double max_stamp_lag{};
  double sensor_buffer_duration{};
  double initial_speed_scale_factor{};
  double initial_speed_scale_factor_covariance{};
  double process_noise_covariance{};
  double measurement_noise_covariance{};
  double max_angular_velocity{};
  double max_speed{};
  double min_speed{};
};

enum class UpdateFailureReason {
  PoseEmpty,
  ImuEmpty,
  VelocityReportEmpty,
  WaitingForNextPose,
  TimeDifferenceTooLarge,
  VelocityReportTimestampMismatch,
  ImuTimestampMismatch,
  AngularVelocityTooHigh,
  VelocityTooHigh,
  VelocityTooLow,
  VelocityReportTooSmall,
};

struct UpdateFailureContext
{
  double time_diff = 0.0;
  double time_diff_threshold = 0.0;
  double stamp_diff = 0.0;
  double stamp_diff_threshold = 0.0;
  double angular_velocity = 0.0;
  double max_angular_velocity = 0.0;
  double velocity = 0.0;
  double velocity_threshold = 0.0;
};

struct SpeedScaleEstimatorUpdated
{
  double estimated_speed_scale_factor = 0.0;
  double covariance = 0.0;
  double velocity_from_pose = 0.0;
  double velocity_from_wheel = 0.0;
  double kalman_gain = 0.0;
  double time_diff = 0.0;
};

struct SpeedScaleEstimatorNotUpdated
{
  UpdateFailureReason reason{};
  UpdateFailureContext context{};
  double last_estimated_speed_scale_factor{};
};

}  // namespace autoware::speed_scale_estimator

#endif  // STRUCTS_HPP_
