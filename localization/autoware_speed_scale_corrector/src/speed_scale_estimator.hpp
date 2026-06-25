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

#include "structs.hpp"

#include <rcpputils/tl_expected/expected.hpp>

#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <deque>
#include <optional>
#include <vector>

namespace autoware::speed_scale_corrector
{

using autoware_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::PoseStamped;
using sensor_msgs::msg::Imu;

class SpeedScaleEstimator
{
public:
  explicit SpeedScaleEstimator(const SpeedScaleEstimatorParameters & parameters);

  [[nodiscard]] double get_update_interval_sec() const;

  [[nodiscard]] tl::expected<SpeedScaleEstimatorUpdated, SpeedScaleEstimatorNotUpdated> update(
    const std::vector<PoseStamped> & poses, const std::vector<Imu> & imus,
    const std::vector<VelocityReport> & velocity_reports);

private:
  [[nodiscard]] SpeedScaleEstimatorNotUpdated make_not_updated(
    UpdateFailureReason reason, const UpdateFailureContext & context = {}) const;

  void update_imu_buffer(const std::vector<Imu> & imus);
  void update_velocity_report_buffer(const std::vector<VelocityReport> & velocity_reports);

  SpeedScaleEstimatorParameters parameters_;
  std::optional<PoseStamped> previous_pose_;
  std::deque<Imu> imu_buffer_;
  std::deque<VelocityReport> velocity_report_buffer_;
  double estimated_speed_scale_factor_ = 1.0;
  double covariance_ = 1.0;
};

}  // namespace autoware::speed_scale_corrector

#endif  // SPEED_SCALE_ESTIMATOR_HPP_
