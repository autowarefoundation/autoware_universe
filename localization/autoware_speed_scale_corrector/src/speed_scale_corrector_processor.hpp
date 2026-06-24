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

#ifndef SPEED_SCALE_CORRECTOR_PROCESSOR_HPP_
#define SPEED_SCALE_CORRECTOR_PROCESSOR_HPP_

#include "speed_scale_estimator.hpp"

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <vector>

namespace autoware::speed_scale_corrector
{

using autoware_internal_debug_msgs::msg::Float32Stamped;
using autoware_internal_debug_msgs::msg::StringStamped;
using autoware_vehicle_msgs::msg::VelocityReport;
using geometry_msgs::msg::PoseStamped;
using sensor_msgs::msg::Imu;

struct SpeedScaleCorrectorProcessResult
{
  bool updated = false;
  tl::expected<SpeedScaleEstimatorUpdated, SpeedScaleEstimatorNotUpdated> estimation_result;
};

class SpeedScaleCorrectorProcessor
{
public:
  explicit SpeedScaleCorrectorProcessor(const SpeedScaleEstimatorParameters & parameters);

  [[nodiscard]] double get_update_interval_sec() const;

  [[nodiscard]] SpeedScaleCorrectorProcessResult process(
    const std::vector<PoseStamped::ConstSharedPtr> & pose_ptrs,
    const std::vector<Imu::ConstSharedPtr> & imu_ptrs,
    const std::vector<VelocityReport::ConstSharedPtr> & velocity_report_ptrs);

  [[nodiscard]] StringStamped make_debug_info(
    const SpeedScaleCorrectorProcessResult & result, const rclcpp::Time & stamp) const;

  [[nodiscard]] Float32Stamped make_scale_factor_msg(
    const SpeedScaleEstimatorUpdated & updated, const rclcpp::Time & stamp) const;

private:
  SpeedScaleEstimator speed_scale_estimator_;
};

}  // namespace autoware::speed_scale_corrector

#endif  // SPEED_SCALE_CORRECTOR_PROCESSOR_HPP_
