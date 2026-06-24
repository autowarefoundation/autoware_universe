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

#ifndef SPEED_SCALE_ESTIMATOR_HPP_
#define SPEED_SCALE_ESTIMATOR_HPP_

#include "tl_expected/expected.hpp"

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <optional>
#include <vector>

/**
 * @brief Speed scale corrector namespace
 */
namespace autoware::speed_scale_corrector
{

using autoware_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::PoseStamped;
using sensor_msgs::msg::Imu;

struct SpeedScaleEstimatorParameters
{
  double update_interval{};
  double initial_speed_scale_factor{};
  double initial_speed_scale_factor_covariance{};
  double process_noise_covariance{};
  double measurement_noise_covariance{};
  double max_angular_velocity{};
  double max_speed{};
  double min_speed{};
};

enum class UpdateFailureReason
{
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
  double velocity_from_odometry = 0.0;
  double velocity_from_velocity_report = 0.0;
  double kalman_gain = 0.0;
  double time_diff = 0.0;
};

struct SpeedScaleEstimatorNotUpdated
{
  UpdateFailureReason reason{};
  UpdateFailureContext context{};
  double last_estimated_speed_scale_factor{};
};

class SpeedScaleEstimator
{
public:
  explicit SpeedScaleEstimator(const SpeedScaleEstimatorParameters & parameters);

  [[nodiscard]] double get_update_interval_sec() const;

  tl::expected<SpeedScaleEstimatorUpdated, SpeedScaleEstimatorNotUpdated> update(
    const std::vector<PoseStamped> & poses, const std::vector<Imu> & imus,
    const std::vector<VelocityReport> & velocity_reports);

private:
  SpeedScaleEstimatorParameters parameters_;
  std::optional<PoseStamped> previous_pose_;
  double estimated_speed_scale_factor_ = 1.0;
  double covariance_ = 1.0;
};

}  // namespace autoware::speed_scale_corrector

#endif  // SPEED_SCALE_ESTIMATOR_HPP_
