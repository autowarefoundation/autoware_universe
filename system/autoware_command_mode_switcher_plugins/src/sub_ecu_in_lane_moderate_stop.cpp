//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "sub_ecu_in_lane_moderate_stop.hpp"

namespace autoware::command_mode_switcher
{

void SubEcuInLaneModerateStopSwitcher::initialize()
{
  pub_trigger_ = node_->create_publisher<JerkConstantDecelerationTrigger>(
    "/control/jerk_constant_deceleration_trigger", rclcpp::QoS{1});
  sub_odom_ =
    std::make_unique<autoware_utils_rclcpp::InterProcessPollingSubscriber<nav_msgs::msg::Odometry>>(
      node_, "/localization/kinematic_state");
  client_relay_trajectory_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_relay_trajectory_ = node_->create_client<tier4_system_msgs::srv::ChangeTopicRelayControl>(
    "/system/topic_relay_controller_trajectory/operate", rmw_qos_profile_services_default,
    client_relay_trajectory_group_);
  client_relay_pose_with_covariance_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  client_relay_pose_with_covariance_ =
    node_->create_client<tier4_system_msgs::srv::ChangeTopicRelayControl>(
      "/system/topic_relay_controller_pose_with_covariance/operate",
      rmw_qos_profile_services_default, client_relay_pose_with_covariance_group_);

  params_.target_acceleration =
    node_->declare_parameter<double>(expand_param("target_acceleration"));
  params_.target_jerk = node_->declare_parameter<double>(expand_param("target_jerk"));
  params_.enable_trajectory_relay =
    node_->declare_parameter<bool>(expand_param("enable_trajectory_relay"));
  params_.enable_pose_with_covariance_relay =
    node_->declare_parameter<bool>(expand_param("enable_pose_with_covariance_relay"));
  params_.service_timeout_ms =
    node_->declare_parameter<int64_t>(expand_param("service_timeout_ms"));
  mrm_state_ = MrmState::Normal;
}

SourceState SubEcuInLaneModerateStopSwitcher::update_source_state(bool request)
{
  if (request && mrm_state_ == MrmState::Operating) return SourceState{true, false};
  if (request && mrm_state_ == MrmState::Succeeded) return SourceState{true, false};
  if (!request && mrm_state_ == MrmState::Normal) return SourceState{false, true};

  if (request) {
    publish_jerk_constant_deceleration_trigger(request);
    // TODO(TetsuKawa): When request_topic_relay_control fails (timeout or service error), we
    // currently still return {true, false} and proceed with in_lane_stop. We need to decide
    // whether relay failure should block the state transition (i.e., return {false, false} or
    // retry) and how that interacts with autoware_command_mode_switcher's rollback mechanism,
    // which cancels the mode transition after transition_timeout_ if is_completed() stays false.
    if (params_.enable_trajectory_relay)
      request_topic_relay_control(false, client_relay_trajectory_, "trajectory");
    if (params_.enable_pose_with_covariance_relay)
      request_topic_relay_control(
        false, client_relay_pose_with_covariance_, "pose_with_covariance");
    mrm_state_ = MrmState::Operating;
    return SourceState{true, false};
  } else {
    publish_jerk_constant_deceleration_trigger(request);
    if (params_.enable_trajectory_relay)
      request_topic_relay_control(true, client_relay_trajectory_, "trajectory");
    if (params_.enable_pose_with_covariance_relay)
      request_topic_relay_control(true, client_relay_pose_with_covariance_, "pose_with_covariance");
    mrm_state_ = MrmState::Normal;
    return SourceState{false, true};
  }
}

MrmState SubEcuInLaneModerateStopSwitcher::update_mrm_state()
{
  if (mrm_state_ != MrmState::Operating) {
    return mrm_state_;
  }

  if (is_stopped()) mrm_state_ = MrmState::Succeeded;
  return mrm_state_;
}

void SubEcuInLaneModerateStopSwitcher::publish_jerk_constant_deceleration_trigger(bool turn_on)
{
  auto trigger = JerkConstantDecelerationTrigger();
  trigger.stamp = node_->now();
  trigger.trigger = turn_on;
  trigger.target_acceleration = params_.target_acceleration;
  trigger.target_jerk = params_.target_jerk;

  pub_trigger_->publish(trigger);
}

void SubEcuInLaneModerateStopSwitcher::request_topic_relay_control(
  const bool relay_on,
  rclcpp::Client<tier4_system_msgs::srv::ChangeTopicRelayControl>::SharedPtr client,
  const std::string & srv_name)
{
  if (!client->service_is_ready()) {
    RCLCPP_WARN(
      node_->get_logger(), "Service unavailable %s relay control: %s", srv_name.c_str(),
      relay_on ? "ON" : "OFF");
    return;
  }

  auto request = std::make_shared<tier4_system_msgs::srv::ChangeTopicRelayControl::Request>();
  request->relay_on = relay_on;

  auto future = client->async_send_request(request);
  if (
    future.wait_for(std::chrono::milliseconds(params_.service_timeout_ms)) !=
    std::future_status::ready) {
    RCLCPP_WARN(
      node_->get_logger(), "Timeout waiting for %s relay control: %s", srv_name.c_str(),
      relay_on ? "ON" : "OFF");
    return;
  }

  const auto & response = future.get();
  if (response->status.success) {
    RCLCPP_INFO(
      node_->get_logger(), "Changed %s relay control: %s", srv_name.c_str(),
      relay_on ? "ON" : "OFF");
  } else {
    RCLCPP_WARN(
      node_->get_logger(), "Failed to change %s relay control: %s", srv_name.c_str(),
      relay_on ? "ON" : "OFF");
  }
}

bool SubEcuInLaneModerateStopSwitcher::is_stopped()
{
  auto odom = sub_odom_->take_data();
  if (odom == nullptr) return false;
  constexpr auto th_stopped_velocity = 0.001;
  return (std::abs(odom->twist.twist.linear.x) < th_stopped_velocity);
}

}  // namespace autoware::command_mode_switcher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::command_mode_switcher::SubEcuInLaneModerateStopSwitcher,
  autoware::command_mode_switcher::CommandPlugin)
