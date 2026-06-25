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

#include <functional>

namespace autoware::speed_scale_corrector
{
namespace
{

SpeedScaleEstimatorParameters load_parameters(rclcpp::Node * node)
{
  SpeedScaleEstimatorParameters parameters;
  parameters.update_interval = node->declare_parameter<double>("update_interval");
  parameters.max_pose_lag = node->declare_parameter<double>("max_pose_lag");
  parameters.max_stamp_lag = node->declare_parameter<double>("max_stamp_lag");
  parameters.sensor_buffer_duration = node->declare_parameter<double>("sensor_buffer_duration");
  parameters.initial_speed_scale_factor =
    node->declare_parameter<double>("initial_speed_scale_factor");
  parameters.initial_speed_scale_factor_covariance =
    node->declare_parameter<double>("initial_speed_scale_factor_covariance");
  parameters.process_noise_covariance = node->declare_parameter<double>("process_noise_covariance");
  parameters.measurement_noise_covariance =
    node->declare_parameter<double>("measurement_noise_covariance");
  parameters.max_angular_velocity = node->declare_parameter<double>("max_angular_velocity");
  parameters.max_speed = node->declare_parameter<double>("max_speed");
  parameters.min_speed = node->declare_parameter<double>("min_speed");
  return parameters;
}

}  // namespace

SpeedScaleCorrectorNode::SpeedScaleCorrectorNode(const rclcpp::NodeOptions & node_options)
: Node("speed_scale_corrector", node_options), processor_(load_parameters(this))
{
  pub_estimated_speed_scale_factor_ = create_publisher<Float32Stamped>("~/output/scale_factor", 1);
  pub_debug_info_ = create_publisher<StringStamped>("~/output/debug_info", 1);

  sub_pose_ =
    PollingSubscriber<PoseStamped, All>::create_subscription(this, "~/input/pose", rclcpp::QoS{10});
  sub_velocity_report_ = PollingSubscriber<VelocityReport, All>::create_subscription(
    this, "~/input/velocity_report", rclcpp::QoS{10});
  sub_imu_ = PollingSubscriber<Imu, All>::create_subscription(this, "~/input/imu", rclcpp::QoS{10});

  timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::duration<double>(processor_.get_update_interval_sec()),
    std::bind(&SpeedScaleCorrectorNode::on_timer, this));
}

void SpeedScaleCorrectorNode::on_timer()
{
  const auto result = processor_.process(
    sub_pose_->take_data(), sub_imu_->take_data(), sub_velocity_report_->take_data());

  const auto now = get_clock()->now();
  pub_debug_info_->publish(processor_.make_debug_info(result, now));

  if (result.updated) {
    pub_estimated_speed_scale_factor_->publish(
      processor_.make_scale_factor_msg(result.estimation_result.value(), now));
  }
}

}  // namespace autoware::speed_scale_corrector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::speed_scale_corrector::SpeedScaleCorrectorNode)
