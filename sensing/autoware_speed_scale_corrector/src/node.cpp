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

#include "node.hpp"

#include "autoware/speed_scale_corrector/speed_scale_estimator.hpp"

#include <fmt/format.h>

#include <cmath>
#include <functional>
#include <vector>

namespace autoware::speed_scale_corrector
{

SpeedScaleCorrectorNode::SpeedScaleCorrectorNode(const rclcpp::NodeOptions & node_options)
: Node("speed_scale_corrector", node_options),
  speed_scale_estimator_(SpeedScaleEstimatorParameters::load_parameters(this))
{
  using std::placeholders::_1;

  // Publishers
  pub_estimated_speed_scale_factor_ = create_publisher<Float32Stamped>("~/output/scale_factor", 1);

  // Subscriber
  sub_pose_ = PollingSubscriber<PoseStamped, All>::create_subscription(this, "~/input/pose", 1);
  sub_velocity_report_ =
    PollingSubscriber<VelocityReport, All>::create_subscription(this, "~/input/velocity_report", 1);
  sub_imu_ = PollingSubscriber<Imu, All>::create_subscription(this, "~/input/imu", 1);

  // Debug Publishers
  pub_debug_info_ = create_publisher<StringStamped>("~/output/debug_info", 1);

  // Timer
  timer_ = rclcpp::create_timer(
    this, get_clock(), speed_scale_estimator_.get_update_interval(),
    std::bind(&SpeedScaleCorrectorNode::on_timer, this));
}

void SpeedScaleCorrectorNode::on_timer()
{
  auto pose_ptrs = sub_pose_->take_data();
  auto velocity_report_ptrs = sub_velocity_report_->take_data();
  auto imu_ptrs = sub_imu_->take_data();

  std::vector<PoseStamped> poses(pose_ptrs.size());
  std::transform(
    pose_ptrs.begin(), pose_ptrs.end(), poses.begin(),
    [](const PoseStamped::ConstSharedPtr & ptr) { return *ptr; });
  std::vector<VelocityReport> velocity_reports(velocity_report_ptrs.size());
  std::transform(
    velocity_report_ptrs.begin(), velocity_report_ptrs.end(), velocity_reports.begin(),
    [](const VelocityReport::ConstSharedPtr & ptr) { return *ptr; });
  std::vector<Imu> imus(imu_ptrs.size());
  std::transform(
    imu_ptrs.begin(), imu_ptrs.end(), imus.begin(),
    [](const Imu::ConstSharedPtr & ptr) { return *ptr; });

  auto result = speed_scale_estimator_.update(poses, imus, velocity_reports);
  StringStamped debug_info;
  debug_info.stamp = get_clock()->now();
  if (!result) {
    debug_info.data = fmt::format(
      "Not updated: {}\nEstimated speed scale factor: {}", result.error().reason,
      result.error().last_estimated_speed_scale_factor);
  } else {
    debug_info.data = fmt::format(
      "Updated:\nEstimated speed scale factor: {}\nStd. dev.: {}\nVelocity from odometry: "
      "{}\nVelocity from velocity report: {}\nKalman gain: {}",
      result.value().estimated_speed_scale_factor, std::sqrt(result.value().covariance),
      result.value().velocity_from_odometry, result.value().velocity_from_velocity_report,
      result.value().kalman_gain);
    Float32Stamped estimated_speed_scale_factor;
    estimated_speed_scale_factor.stamp = get_clock()->now();
    estimated_speed_scale_factor.data =
      static_cast<float>(result.value().estimated_speed_scale_factor);
    pub_estimated_speed_scale_factor_->publish(estimated_speed_scale_factor);
  }
  pub_debug_info_->publish(debug_info);
}

}  // namespace autoware::speed_scale_corrector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::speed_scale_corrector::SpeedScaleCorrectorNode)
