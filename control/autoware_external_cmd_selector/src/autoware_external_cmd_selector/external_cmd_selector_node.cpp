// Copyright 2020 Tier IV, Inc.
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

#include "autoware/external_cmd_selector/external_cmd_selector_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace autoware::external_cmd_selector
{

ExternalCmdSelector::ExternalCmdSelector(const rclcpp::NodeOptions & node_options)
: Node("external_cmd_selector", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Parameter
  double update_rate = declare_parameter<double>("update_rate");
  std::string initial_selector_mode = declare_parameter<std::string>("initial_selector_mode");

  // Publisher
  pub_current_selector_mode_ =
    create_publisher<CommandSourceMode>("~/output/current_selector_mode", 1);
  pub_pedals_cmd_ = create_publisher<PedalsCommand>("~/output/pedals_cmd", 1);
  pub_steering_cmd_ = create_publisher<SteeringCommand>("~/output/steering_cmd", 1);
  pub_heartbeat_ = create_publisher<OperatorHeartbeat>("~/output/heartbeat", 1);
  pub_gear_cmd_ = create_publisher<GearCommand>("~/output/gear_cmd", 1);
  pub_turn_indicators_cmd_ =
    create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators_cmd", 1);
  pub_hazard_lights_cmd_ = create_publisher<HazardLightsCommand>("~/output/hazard_lights_cmd", 1);

  // Callback Groups
  callback_group_subscribers_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_services_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = callback_group_subscribers_;

  // Subscriber
  sub_local_pedals_cmd_ = create_subscription<PedalsCommand>(
    "~/input/local/pedals_cmd", 1,
    bind_on_cmd<PedalsCommand>(&ExternalCmdSelector::on_pedals_cmd, CommandSourceMode::LOCAL),
    subscriber_option);
  sub_local_steering_cmd_ = create_subscription<SteeringCommand>(
    "~/input/local/steering_cmd", 1,
    bind_on_cmd<SteeringCommand>(&ExternalCmdSelector::on_steering_cmd, CommandSourceMode::LOCAL),
    subscriber_option);
  sub_local_heartbeat_ = create_subscription<OperatorHeartbeat>(
    "~/input/local/heartbeat", 1,
    bind_on_cmd<OperatorHeartbeat>(&ExternalCmdSelector::on_heartbeat, CommandSourceMode::LOCAL),
    subscriber_option);
  sub_local_gear_cmd_ = create_subscription<GearCommand>(
    "~/input/local/gear_cmd", 1,
    bind_on_cmd<GearCommand>(&ExternalCmdSelector::on_gear_cmd, CommandSourceMode::LOCAL),
    subscriber_option);
  sub_local_turn_indicators_cmd_ = create_subscription<TurnIndicatorsCommand>(
    "~/input/local/turn_indicators_cmd", 1,
    bind_on_cmd<TurnIndicatorsCommand>(
      &ExternalCmdSelector::on_turn_indicators_cmd, CommandSourceMode::LOCAL),
    subscriber_option);
  sub_local_hazard_lights_cmd_ = create_subscription<HazardLightsCommand>(
    "~/input/local/hazard_lights_cmd", 1,
    bind_on_cmd<HazardLightsCommand>(
      &ExternalCmdSelector::on_hazard_lights_cmd, CommandSourceMode::LOCAL),
    subscriber_option);

  sub_remote_pedals_cmd_ = create_subscription<PedalsCommand>(
    "~/input/remote/pedals_cmd", 1,
    bind_on_cmd<PedalsCommand>(&ExternalCmdSelector::on_pedals_cmd, CommandSourceMode::REMOTE),
    subscriber_option);
  sub_remote_steering_cmd_ = create_subscription<SteeringCommand>(
    "~/input/remote/steering_cmd", 1,
    bind_on_cmd<SteeringCommand>(&ExternalCmdSelector::on_steering_cmd, CommandSourceMode::REMOTE),
    subscriber_option);
  sub_remote_heartbeat_ = create_subscription<OperatorHeartbeat>(
    "~/input/remote/heartbeat", 1,
    bind_on_cmd<OperatorHeartbeat>(&ExternalCmdSelector::on_heartbeat, CommandSourceMode::REMOTE),
    subscriber_option);
  sub_remote_gear_cmd_ = create_subscription<GearCommand>(
    "~/input/remote/gear_cmd", 1,
    bind_on_cmd<GearCommand>(&ExternalCmdSelector::on_gear_cmd, CommandSourceMode::REMOTE),
    subscriber_option);
  sub_remote_turn_indicators_cmd_ = create_subscription<TurnIndicatorsCommand>(
    "~/input/remote/turn_indicators_cmd", 1,
    bind_on_cmd<TurnIndicatorsCommand>(
      &ExternalCmdSelector::on_turn_indicators_cmd, CommandSourceMode::REMOTE),
    subscriber_option);
  sub_remote_hazard_lights_cmd_ = create_subscription<HazardLightsCommand>(
    "~/input/remote/hazard_lights_cmd", 1,
    bind_on_cmd<HazardLightsCommand>(
      &ExternalCmdSelector::on_hazard_lights_cmd, CommandSourceMode::REMOTE),
    subscriber_option);

  // Service
  srv_select_external_command_ = create_service<CommandSourceSelect>(
    "~/service/select_external_command",
    std::bind(&ExternalCmdSelector::on_select_external_command, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_services_);

  // Initialize mode
  auto convert_selector_mode = [](const std::string & mode_text) {
    if (mode_text == "local") {
      return CommandSourceMode::LOCAL;
    }
    if (mode_text == "remote") {
      return CommandSourceMode::REMOTE;
    }
    throw std::invalid_argument("unknown selector mode");
  };
  current_selector_mode_.data = convert_selector_mode(initial_selector_mode);

  // Diagnostics Updater
  updater_.setHardwareID("external_cmd_selector");
  updater_.add("heartbeat", [](auto & stat) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Alive");
  });

  // Timer
  const auto period_ns = rclcpp::Rate(update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&ExternalCmdSelector::on_timer, this),
    callback_group_subscribers_);
}

void ExternalCmdSelector::on_pedals_cmd(const PedalsCommand & msg, uint8_t mode)
{
  if (current_selector_mode_.data != mode) return;
  pub_pedals_cmd_->publish(msg);
}

void ExternalCmdSelector::on_steering_cmd(const SteeringCommand & msg, uint8_t mode)
{
  if (current_selector_mode_.data != mode) return;
  pub_steering_cmd_->publish(msg);
}

void ExternalCmdSelector::on_heartbeat(const OperatorHeartbeat & msg, uint8_t mode)
{
  if (current_selector_mode_.data != mode) return;
  pub_heartbeat_->publish(msg);
}

void ExternalCmdSelector::on_gear_cmd(const GearCommand & msg, uint8_t mode)
{
  if (current_selector_mode_.data != mode) return;
  pub_gear_cmd_->publish(msg);
}

void ExternalCmdSelector::on_turn_indicators_cmd(const TurnIndicatorsCommand & msg, uint8_t mode)
{
  if (current_selector_mode_.data != mode) return;
  pub_turn_indicators_cmd_->publish(msg);
}

void ExternalCmdSelector::on_hazard_lights_cmd(const HazardLightsCommand & msg, uint8_t mode)
{
  if (current_selector_mode_.data != mode) return;
  pub_hazard_lights_cmd_->publish(msg);
}

bool ExternalCmdSelector::on_select_external_command(
  const CommandSourceSelect::Request::SharedPtr req,
  const CommandSourceSelect::Response::SharedPtr res)
{
  current_selector_mode_.data = req->mode.data;
  res->success = true;
  res->message = "Success.";
  return true;
}

void ExternalCmdSelector::on_timer()
{
  pub_current_selector_mode_->publish(current_selector_mode_);
  updater_.force_update();
}

}  // namespace autoware::external_cmd_selector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::external_cmd_selector::ExternalCmdSelector)
