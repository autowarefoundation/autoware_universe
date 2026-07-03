// Copyright 2026 The Autoware Contributors
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

#include "mrm_in_lane_stop_operator.hpp"

#include <cmath>
#include <chrono>

namespace autoware::mrm_in_lane_stop_operator
{

MrmInLaneStopOperator::MrmInLaneStopOperator(const rclcpp::NodeOptions & node_options)
: Node("mrm_in_lane_stop_operator", node_options)
{
  // parameter
  service_timeout_ms_ = declare_parameter<int64_t>("service_timeout_ms");
  const auto mode_names = declare_parameter<std::vector<std::string>>("mode_names");

  for (const auto & name : mode_names) {
    ModeConfig mode;
    mode.name = name;
    mode.target_acceleration = declare_parameter<double>(name + ".target_acceleration");
    mode.target_jerk = declare_parameter<double>(name + ".target_jerk");

    modes_.push_back(std::move(mode));
  }

  // client
  relay_service_name_ = "~/input/relay_service";
  relay_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  relay_client_ = create_client<ChangeTopicRelayControl>(
    relay_service_name_, rmw_qos_profile_services_default, relay_group_);

  // publisher
  pub_trigger_ =
    create_publisher<ConstantJerkDecelerationTrigger>("~/output/jerk_deceleration_trigger", rclcpp::QoS{1});
  pub_mrm_state_ = create_publisher<DrivingModeMrmState>("~/output/mrm_state", 1);
  pub_driving_mode_active_ = create_publisher<DrivingModeFlag>("~/output/driving_mode_active", 1);
  sub_request_ = create_subscription<DrivingModeRequest>(
    "~/input/driving_mode_request", 1,
    std::bind(&MrmInLaneStopOperator::on_request, this, std::placeholders::_1));

  // subscription
  sub_info_ = create_subscription<DrivingModeInfo>(
    "~/input/driving_mode_info", rclcpp::QoS(1).transient_local(),
    std::bind(&MrmInLaneStopOperator::on_info, this, std::placeholders::_1));
  sub_kinematic_state_ =
    std::make_unique<autoware_utils_rclcpp::InterProcessPollingSubscriber<Odometry>>(
      this, "/localization/kinematic_state");

  // timer
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(10).period(),
    std::bind(&MrmInLaneStopOperator::on_timer, this));
}

void MrmInLaneStopOperator::on_info(DrivingModeInfo::ConstSharedPtr msg)
{
  for (const auto & item : msg->items) {
    for (auto & mode : modes_) {
      if (mode.name == item.name) {
        mode.mode_id = item.mode;
      }
    }
  }
}

void MrmInLaneStopOperator::on_request(DrivingModeRequest::ConstSharedPtr msg)
{
  const auto find_mode_by_id = [this](const uint32_t id) -> ModeConfig * {
    for (auto & mode : modes_) {
      if (mode.mode_id == id) return &mode;
    }
    return nullptr;
  };

  const auto requested_id = msg->mode;
  auto * requested = find_mode_by_id(requested_id);

  if (requested) {
    // Request is for one of our configured modes.
    if (active_mode_id_ == requested_id) {
      return;  // Already running this exact mode; do nothing.
    }
    if (active_mode_id_.has_value()) {
      // A different one of our modes is running; cancel it first.
      auto * current = find_mode_by_id(active_mode_id_.value());
      if (current) cancel(*current);
    }
    if (execute(*requested)) {
      active_mode_id_ = requested_id;
      // For real-time safety, we should only publish the active flag
      // After switching the active_mode_id_ to the new mode, so that the published flag is consistent with the internal state.
      publishDrivingModeActive();
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Failed to enable one or more relay services for mode=%u. Keep active_mode_id_ unchanged.",
        requested_id);
    }
  } else {
    // Request is not for any of our modes; cancel if we are active.
    if (active_mode_id_.has_value()) {
      auto * current = find_mode_by_id(active_mode_id_.value());
      if (current) cancel(*current);
      active_mode_id_ = std::nullopt;
    }
  }
}

bool MrmInLaneStopOperator::execute(ModeConfig & mode)
{
  publish_trigger(true, mode.target_acceleration, mode.target_jerk);
  const bool relay_success = call_relay(false);

  if (relay_success) {
    RCLCPP_INFO(get_logger(), "Execute MRM: %s", mode.name.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Execute MRM with relay failures: %s", mode.name.c_str());
  }

  return relay_success;
}

void MrmInLaneStopOperator::cancel(ModeConfig & mode)
{
  publish_trigger(false, mode.target_acceleration, mode.target_jerk);
  call_relay(true);
  RCLCPP_INFO(get_logger(), "Cancel MRM: %s", mode.name.c_str());
}

void MrmInLaneStopOperator::publish_trigger(
  bool turn_on, double target_acceleration, double target_jerk)
{
  ConstantJerkDecelerationTrigger msg;
  msg.stamp = now();
  msg.trigger = turn_on;
  msg.target_acceleration = static_cast<float>(target_acceleration);
  msg.target_jerk = static_cast<float>(target_jerk);
  pub_trigger_->publish(msg);
}

bool MrmInLaneStopOperator::call_relay(bool relay_on)
{
  if (!relay_client_->service_is_ready()) {
    RCLCPP_WARN(
      get_logger(), "Service unavailable: %s (relay_on=%s)",
      relay_service_name_.c_str(), relay_on ? "true" : "false");
    return false;
  }

  auto request = std::make_shared<ChangeTopicRelayControl::Request>();
  request->relay_on = relay_on;

  auto future = relay_client_->async_send_request(request);
  if (
    future.wait_for(std::chrono::milliseconds(service_timeout_ms_)) !=
    std::future_status::ready) {
    RCLCPP_WARN(
      get_logger(), "Timeout waiting for relay service: %s", relay_service_name_.c_str());
    return false;
  }

  const auto & response = future.get();
  if (!response->status.success) {
    RCLCPP_WARN(
      get_logger(), "Failed to change relay control: %s (relay_on=%s)",
      relay_service_name_.c_str(), relay_on ? "true" : "false");
    return false;
  }

  return true;
}

void MrmInLaneStopOperator::publishDrivingModeActive() const
{
  DrivingModeFlag msg;
  msg.stamp = now();

  for (const auto & mode : modes_) {
    if (!mode.mode_id) continue;
    tier4_system_msgs::msg::DrivingModeFlagItem item;
    item.mode = mode.mode_id.value();
    item.flag = (active_mode_id_ == mode.mode_id);
    msg.items.push_back(item);
  }

  pub_driving_mode_active_->publish(msg);
}

void MrmInLaneStopOperator::publish_mrm_state() const
{
  using tier4_system_msgs::msg::DrivingModeMrmStateItem;

  DrivingModeMrmState msg;
  msg.stamp = now();

  for (const auto & mode : modes_) {
    if (!mode.mode_id) continue;
    DrivingModeMrmStateItem item;
    item.mode = mode.mode_id.value();

    item.state = DrivingModeMrmStateItem::UNKNOWN;
    if (active_mode_id_ != mode.mode_id) {
      item.state = DrivingModeMrmStateItem::NORMAL;
    } else if (is_vehicle_stopped()) {
      item.state = DrivingModeMrmStateItem::SUCCEEDED;
    } else {
      item.state = DrivingModeMrmStateItem::OPERATING;
    }

    msg.items.push_back(item);
  }

  pub_mrm_state_->publish(msg);
}

bool MrmInLaneStopOperator::is_vehicle_stopped() const
{
  auto odom = sub_kinematic_state_->take_data();
  if (!odom) {
    return false;
  }

  // Match in_lane_stop switcher criteria.
  constexpr double th_stopped_velocity = 0.001;
  return std::abs(odom->twist.twist.linear.x) < th_stopped_velocity;
}

void MrmInLaneStopOperator::on_timer()
{
  publish_mrm_state();
  publishDrivingModeActive();
}

}  // namespace autoware::mrm_in_lane_stop_operator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mrm_in_lane_stop_operator::MrmInLaneStopOperator)
