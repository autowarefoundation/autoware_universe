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

#include "autoware/speed_scale_corrector/node.hpp"

#include "autoware/speed_scale_corrector/speed_scale_estimator.hpp"

#include <fmt/format.h>

#include <cmath>
#include <functional>
#include <vector>

namespace autoware::speed_scale_corrector
{

/**
 * @brief Load parameters from ROS parameter server
 * @param node Pointer to ROS node for parameter access
 * @return SpeedScaleEstimatorParameters Loaded parameters with default values
 */
SpeedScaleEstimatorParameters load_parameters(rclcpp::Node * node)
{
  SpeedScaleEstimatorParameters parameters;
  parameters.time_window = node->declare_parameter<double>("time_window");
  parameters.time_interval = node->declare_parameter<double>("time_interval");
  parameters.initial_speed_scale_factor =
    node->declare_parameter<double>("initial_speed_scale_factor");
  parameters.max_angular_velocity = node->declare_parameter<double>("max_angular_velocity");
  parameters.max_speed = node->declare_parameter<double>("max_speed");
  parameters.min_speed = node->declare_parameter<double>("min_speed");
  parameters.max_speed_change = node->declare_parameter<double>("max_speed_change");
  return parameters;
}

SpeedScaleCorrectorNode::SpeedScaleCorrectorNode(const rclcpp::NodeOptions & node_options)
: Node("speed_scale_corrector", node_options), speed_scale_estimator_(load_parameters(this))
{
  using std::placeholders::_1;

  // Publishers
  pub_estimated_speed_scale_factor_ = create_publisher<Float32Stamped>("~/output/scale_factor", 1);

  // Subscriber
  sub_pose_with_covariance_ = this->create_subscription<PoseWithCovarianceStamped>(
    "~/input/pose_with_covariance", 1,
    std::bind(&SpeedScaleCorrectorNode::on_pose_with_covariance, this, _1));
  sub_velocity_report_ =
    PollingSubscriber<VelocityReport, All>::create_subscription(this, "~/input/velocity_report", 1);
  sub_imu_ = PollingSubscriber<Imu, All>::create_subscription(this, "~/input/imu", 1);

  // Debug Publishers
  pub_debug_info_ = create_publisher<StringStamped>("~/output/debug_info", 1);
}

void SpeedScaleCorrectorNode::on_pose_with_covariance(
  const PoseWithCovarianceStamped::ConstSharedPtr pose_with_covariance)
{
  if (pose_with_covariance == nullptr) {
    return;
  }

  auto velocity_report_ptrs = sub_velocity_report_->take_data();
  auto imu_ptrs = sub_imu_->take_data();

  std::vector<VelocityReport> velocity_reports(velocity_report_ptrs.size());
  std::transform(
    velocity_report_ptrs.begin(), velocity_report_ptrs.end(), velocity_reports.begin(),
    [](const VelocityReport::ConstSharedPtr & ptr) { return *ptr; });
  std::vector<Imu> imus(imu_ptrs.size());
  std::transform(
    imu_ptrs.begin(), imu_ptrs.end(), imus.begin(),
    [](const Imu::ConstSharedPtr & ptr) { return *ptr; });

  auto result = speed_scale_estimator_.update(*pose_with_covariance, imus, velocity_reports);
  StringStamped debug_info;
  debug_info.stamp = get_clock()->now();
  if (!result) {
    debug_info.data = fmt::format(
      "Not updated: {}\nEstimated speed scale factor: {}", result.error().reason,
      result.error().last_estimated_speed_scale_factor);
  } else {
    debug_info.data = fmt::format(
      "Updated:\nEstimated speed scale factor: {}", result.value().estimated_speed_scale_factor);
    Float32Stamped estimated_speed_scale_factor;
    estimated_speed_scale_factor.stamp = get_clock()->now();
    estimated_speed_scale_factor.data =
      static_cast<float>(result.value().estimated_speed_scale_factor);
    pub_estimated_speed_scale_factor_->publish(estimated_speed_scale_factor);
  }
  pub_debug_info_->publish(debug_info);
}

}  // namespace autoware::speed_scale_corrector
