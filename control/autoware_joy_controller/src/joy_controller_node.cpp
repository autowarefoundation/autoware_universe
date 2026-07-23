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

#include "autoware/joy_controller/joy_controller.hpp"
#include "autoware/joy_controller/joy_converter/ds4_joy_converter.hpp"
#include "autoware/joy_controller/joy_converter/g29_joy_converter.hpp"
#include "autoware/joy_controller/joy_converter/p65_joy_converter.hpp"
#include "autoware/joy_controller/joy_converter/xbox_joy_converter.hpp"

#include <autoware/qos_utils/qos_compatibility.hpp>
#include <tier4_api_utils/tier4_api_utils.hpp>

#include <algorithm>
#include <memory>
#include <string>

namespace
{
using autoware::joy_controller::GateModeType;
using autoware::joy_controller::GearCommandType;
using autoware::joy_controller::TurnIndicatorsCommandType;
using GateMode = tier4_control_msgs::msg::GateMode;
using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
using HazardLightsCommand = autoware_vehicle_msgs::msg::HazardLightsCommand;
using TurnIndicatorsCommand = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

GearCommandType getUpperShift(const GearCommandType & shift)
{
  if (shift == GearCommand::NONE) {
    return GearCommand::PARK;
  }
  if (shift == GearCommand::PARK) {
    return GearCommand::REVERSE;
  }
  if (shift == GearCommand::REVERSE) {
    return GearCommand::NEUTRAL;
  }
  if (shift == GearCommand::NEUTRAL) {
    return GearCommand::DRIVE;
  }
  if (shift == GearCommand::DRIVE) {
    return GearCommand::LOW;
  }
  if (shift == GearCommand::LOW) {
    return GearCommand::LOW;
  }

  return GearCommand::NONE;
}

GearCommandType getLowerShift(const GearCommandType & shift)
{
  if (shift == GearCommand::NONE) {
    return GearCommand::PARK;
  }
  if (shift == GearCommand::PARK) {
    return GearCommand::PARK;
  }
  if (shift == GearCommand::REVERSE) {
    return GearCommand::PARK;
  }
  if (shift == GearCommand::NEUTRAL) {
    return GearCommand::REVERSE;
  }
  if (shift == GearCommand::DRIVE) {
    return GearCommand::NEUTRAL;
  }
  if (shift == GearCommand::LOW) {
    return GearCommand::DRIVE;
  }

  return GearCommand::NONE;
}

const char * getShiftName(const GearCommandType & shift)
{
  if (shift == GearCommand::NONE) {
    return "NONE";
  }
  if (shift == GearCommand::PARK) {
    return "PARK";
  }
  if (shift == GearCommand::REVERSE) {
    return "REVERSE";
  }
  if (shift == GearCommand::NEUTRAL) {
    return "NEUTRAL";
  }
  if (shift == GearCommand::DRIVE) {
    return "DRIVE";
  }
  if (shift == GearCommand::LOW) {
    return "LOW";
  }

  return "NOT_SUPPORTED";
}

const char * getTurnSignalName(const TurnIndicatorsCommandType & turn_signal)
{
  if (turn_signal == TurnIndicatorsCommand::NO_COMMAND) {
    return "NO_COMMAND";
  }
  if (turn_signal == TurnIndicatorsCommand::DISABLE) {
    return "DISABLE";
  }
  if (turn_signal == TurnIndicatorsCommand::ENABLE_LEFT) {
    return "ENABLE_LEFT";
  }
  if (turn_signal == TurnIndicatorsCommand::ENABLE_RIGHT) {
    return "ENABLE_RIGHT";
  }

  return "NOT_SUPPORTED";
}

const char * getGateModeName(const GateModeType & gate_mode)
{
  using tier4_control_msgs::msg::GateMode;

  if (gate_mode == GateMode::AUTO) {
    return "AUTO";
  }
  if (gate_mode == GateMode::EXTERNAL) {
    return "EXTERNAL";
  }

  return "NOT_SUPPORTED";
}

double calcMapping(const double input, const double sensitivity)
{
  const double exponent = 1.0 / (std::max(0.001, std::min(1.0, sensitivity)));
  return std::pow(input, exponent);
}

}  // namespace

namespace autoware::joy_controller
{
void AutowareJoyControllerNode::onJoy()
{
  const auto msg = sub_joy_.take_data();
  if (!msg) {
    return;
  }

  last_joy_received_time_ = msg->header.stamp;
  if (joy_type_ == "G29") {
    joy_ = std::make_shared<const G29JoyConverter>(*msg);
  } else if (joy_type_ == "DS4") {
    joy_ = std::make_shared<const DS4JoyConverter>(*msg);
  } else if (joy_type_ == "XBOX") {
    joy_ = std::make_shared<const XBOXJoyConverter>(*msg);
  } else {
    joy_ = std::make_shared<const P65JoyConverter>(*msg);
  }

  if (joy_->shift_up() || joy_->shift_down() || joy_->shift_drive() || joy_->shift_reverse()) {
    publishGearCommand();
  }

  if (joy_->turn_signal_left() || joy_->turn_signal_right() || joy_->clear_turn_signal()) {
    publishTurnIndicatorsCommand();
  }

  if (joy_->gate_mode()) {
    publishGateMode();
  }

  if (joy_->autoware_engage() || joy_->autoware_disengage()) {
    publishAutowareEngage();
  }

  if (joy_->vehicle_engage() || joy_->vehicle_disengage()) {
    publishVehicleEngage();
  }

  if (joy_->emergency_stop()) {
    sendEmergencyRequest(true);
  }

  if (joy_->clear_emergency_stop()) {
    sendEmergencyRequest(false);
  }
}

void AutowareJoyControllerNode::onOdometry()
{
  if (raw_control_) {
    return;
  }

  const auto msg = sub_odom_.take_data();
  if (!msg) {
    return;
  }

  auto twist = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist->header = msg->header;
  twist->twist = msg->twist.twist;

  twist_ = twist;
}

bool AutowareJoyControllerNode::isDataReady()
{
  // Joy
  {
    if (!joy_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for joy msg...");
      return false;
    }

    constexpr auto timeout = 2.0;
    const auto time_diff = this->now() - last_joy_received_time_;
    if (time_diff.seconds() > timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(), "joy msg is timeout");
      return false;
    }
  }

  // Twist
  if (!raw_control_) {
    if (!twist_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for twist msg...");
      return false;
    }

    constexpr auto timeout = 0.5;
    const auto time_diff = this->now() - twist_->header.stamp;
    if (time_diff.seconds() > timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
        "twist msg is timeout");
      return false;
    }
  }

  return true;
}

void AutowareJoyControllerNode::onTimer()
{
  onOdometry();
  onJoy();

  if (!isDataReady()) {
    return;
  }

  publishControlCommand();
  publishPedalsCommand();
  publishSteeringCommand();
  publishHeartbeat();
}

void AutowareJoyControllerNode::publishControlCommand()
{
  autoware_control_msgs::msg::Control cmd;
  cmd.stamp = this->now();
  {
    cmd.lateral.steering_tire_angle = steer_ratio_ * joy_->steer();
    cmd.lateral.steering_tire_rotation_rate = steering_angle_velocity_;

    if (joy_->accel()) {
      cmd.longitudinal.acceleration = accel_ratio_ * joy_->accel();
      cmd.longitudinal.velocity =
        twist_->twist.linear.x + velocity_gain_ * cmd.longitudinal.acceleration;
      cmd.longitudinal.velocity =
        std::min(cmd.longitudinal.velocity, static_cast<float>(max_forward_velocity_));
    }

    if (joy_->brake()) {
      cmd.longitudinal.velocity = 0.0;
      cmd.longitudinal.acceleration = -brake_ratio_ * joy_->brake();
    }

    // Backward
    if (joy_->accel() && joy_->brake()) {
      cmd.longitudinal.acceleration = backward_accel_ratio_ * joy_->accel();
      cmd.longitudinal.velocity =
        twist_->twist.linear.x - velocity_gain_ * cmd.longitudinal.acceleration;
      cmd.longitudinal.velocity =
        std::max(cmd.longitudinal.velocity, static_cast<float>(-max_backward_velocity_));
    }
  }

  pub_control_command_->publish(cmd);
  prev_control_command_ = cmd;
}

void AutowareJoyControllerNode::publishPedalsCommand()
{
  autoware_adapi_v1_msgs::msg::PedalsCommand cmd;
  cmd.stamp = this->now();
  cmd.throttle = accel_ratio_ * calcMapping(static_cast<double>(joy_->accel()), accel_sensitivity_);
  cmd.brake = brake_ratio_ * calcMapping(static_cast<double>(joy_->brake()), brake_sensitivity_);
  pub_pedals_command_->publish(cmd);
}

void AutowareJoyControllerNode::publishSteeringCommand()
{
  autoware_adapi_v1_msgs::msg::SteeringCommand cmd;
  cmd.stamp = this->now();
  cmd.steering_tire_angle = steer_ratio_ * joy_->steer();
  cmd.steering_tire_velocity = steering_angle_velocity_;
  pub_steering_command_->publish(cmd);
}

void AutowareJoyControllerNode::publishGearCommand()
{
  autoware_vehicle_msgs::msg::GearCommand gear_shift;
  gear_shift.stamp = this->now();

  if (joy_->shift_up()) {
    gear_shift.command = getUpperShift(prev_shift_);
  }

  if (joy_->shift_down()) {
    gear_shift.command = getLowerShift(prev_shift_);
  }

  if (joy_->shift_drive()) {
    gear_shift.command = GearCommand::DRIVE;
  }

  if (joy_->shift_reverse()) {
    gear_shift.command = GearCommand::REVERSE;
  }

  RCLCPP_INFO(get_logger(), "GearCommand::%s", getShiftName(gear_shift.command));

  pub_gear_cmd_->publish(gear_shift);
  prev_shift_ = gear_shift.command;
}

void AutowareJoyControllerNode::publishTurnIndicatorsCommand()
{
  autoware_vehicle_msgs::msg::TurnIndicatorsCommand turn_signal;
  turn_signal.stamp = this->now();
  turn_signal.command = TurnIndicatorsCommand::DISABLE;

  if (joy_->turn_signal_left() && joy_->turn_signal_right()) {
    publishHazardLights(true);
  } else if (joy_->turn_signal_left()) {
    turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
    publishHazardLights(false);
  } else if (joy_->turn_signal_right()) {
    turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
    publishHazardLights(false);
  }

  if (joy_->clear_turn_signal()) {
    turn_signal.command = TurnIndicatorsCommand::DISABLE;
    publishHazardLights(false);
  }

  RCLCPP_INFO(get_logger(), "TurnIndicatorsCommand::%s", getTurnSignalName(turn_signal.command));

  pub_turn_indicators_cmd_->publish(turn_signal);
}

void AutowareJoyControllerNode::publishHazardLights(const bool enable)
{
  autoware_vehicle_msgs::msg::HazardLightsCommand hazard_lights;
  hazard_lights.stamp = this->now();
  hazard_lights.command = enable ? HazardLightsCommand::ENABLE : HazardLightsCommand::DISABLE;
  pub_hazard_lights_->publish(hazard_lights);
}

void AutowareJoyControllerNode::publishGateMode()
{
  tier4_control_msgs::msg::GateMode gate_mode;

  if (prev_gate_mode_ == GateMode::AUTO) {
    gate_mode.data = GateMode::EXTERNAL;
  }

  if (prev_gate_mode_ == GateMode::EXTERNAL) {
    gate_mode.data = GateMode::AUTO;
  }

  RCLCPP_INFO(get_logger(), "GateMode::%s", getGateModeName(gate_mode.data));

  pub_gate_mode_->publish(gate_mode);
  prev_gate_mode_ = gate_mode.data;
}

void AutowareJoyControllerNode::publishHeartbeat()
{
  autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat operator_heartbeat;
  operator_heartbeat.stamp = this->now();
  operator_heartbeat.ready = true;
  pub_operator_heartbeat_->publish(operator_heartbeat);
}

void AutowareJoyControllerNode::sendEmergencyRequest(bool emergency)
{
  RCLCPP_INFO(get_logger(), "%s emergency stop", emergency ? "Set" : "Clear");

  auto request = std::make_shared<tier4_external_api_msgs::srv::SetEmergency::Request>();
  request->emergency = emergency;

  client_emergency_stop_->async_send_request(
    request,
    [this](rclcpp::Client<tier4_external_api_msgs::srv::SetEmergency>::SharedFuture result) {
      auto response = result.get();
      if (tier4_api_utils::is_success(response->status)) {
        RCLCPP_INFO(get_logger(), "service succeeded");
      } else {
        RCLCPP_WARN(get_logger(), "service failed: %s", response->status.message.c_str());
      }
    });
}

void AutowareJoyControllerNode::publishAutowareEngage()
{
  auto req = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
  if (joy_->autoware_engage()) {
    req->engage = true;
    RCLCPP_INFO(get_logger(), "Autoware Engage");
  }

  if (joy_->autoware_disengage()) {
    req->engage = false;
    RCLCPP_INFO(get_logger(), "Autoware Disengage");
  }

  if (!client_autoware_engage_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "%s is unavailable", client_autoware_engage_->get_service_name());
    return;
  }

  client_autoware_engage_->async_send_request(
    req, [this](rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture result) {
      RCLCPP_INFO(
        get_logger(), "%s: %d, %s", client_autoware_engage_->get_service_name(),
        result.get()->status.code, result.get()->status.message.c_str());
    });
}

void AutowareJoyControllerNode::publishVehicleEngage()
{
  autoware_vehicle_msgs::msg::Engage engage;

  if (joy_->vehicle_engage()) {
    engage.engage = true;
    RCLCPP_INFO(get_logger(), "Vehicle Engage");
  }

  if (joy_->vehicle_disengage()) {
    engage.engage = false;
    RCLCPP_INFO(get_logger(), "Vehicle Disengage");
  }

  pub_vehicle_engage_->publish(engage);
}

void AutowareJoyControllerNode::initTimer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&AutowareJoyControllerNode::onTimer, this));
}

AutowareJoyControllerNode::AutowareJoyControllerNode(const rclcpp::NodeOptions & node_options)
: Node("autoware_joy_controller", node_options)
{
  // Parameter
  joy_type_ = declare_parameter<std::string>("joy_type");
  update_rate_ = declare_parameter<double>("update_rate");
  accel_ratio_ = declare_parameter<double>("accel_ratio");
  brake_ratio_ = declare_parameter<double>("brake_ratio");
  steer_ratio_ = declare_parameter<double>("steer_ratio");
  steering_angle_velocity_ = declare_parameter<double>("steering_angle_velocity");
  accel_sensitivity_ = declare_parameter<double>("accel_sensitivity");
  brake_sensitivity_ = declare_parameter<double>("brake_sensitivity");
  raw_control_ = declare_parameter<bool>("control_command.raw_control");
  velocity_gain_ = declare_parameter<double>("control_command.velocity_gain");
  max_forward_velocity_ = declare_parameter<double>("control_command.max_forward_velocity");
  max_backward_velocity_ = declare_parameter<double>("control_command.max_backward_velocity");
  backward_accel_ratio_ = declare_parameter<double>("control_command.backward_accel_ratio");

  RCLCPP_INFO(get_logger(), "Joy type: %s", joy_type_.c_str());

  // Callback Groups
  callback_group_services_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Subscriber
  if (raw_control_) {
    twist_ = std::make_shared<geometry_msgs::msg::TwistStamped>();
  }

  // Publisher
  pub_control_command_ =
    this->create_publisher<autoware_control_msgs::msg::Control>("output/control_command", 1);
  pub_pedals_command_ =
    this->create_publisher<autoware_adapi_v1_msgs::msg::PedalsCommand>("output/pedals_command", 1);
  pub_steering_command_ = this->create_publisher<autoware_adapi_v1_msgs::msg::SteeringCommand>(
    "output/steering_command", 1);
  pub_gear_cmd_ =
    this->create_publisher<autoware_vehicle_msgs::msg::GearCommand>("output/gear_cmd", 1);
  pub_turn_indicators_cmd_ =
    this->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "output/turn_indicators_cmd", 1);
  pub_hazard_lights_ = this->create_publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>(
    "output/hazard_lights_cmd", 1);
  pub_gate_mode_ = this->create_publisher<tier4_control_msgs::msg::GateMode>("output/gate_mode", 1);
  pub_operator_heartbeat_ =
    this->create_publisher<autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat>(
      "output/operator_heartbeat", 1);
  pub_vehicle_engage_ =
    this->create_publisher<autoware_vehicle_msgs::msg::Engage>("output/vehicle_engage", 1);

  // Service Client
  client_emergency_stop_ = this->create_client<tier4_external_api_msgs::srv::SetEmergency>(
    "service/emergency_stop", AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE(), callback_group_services_);
  while (!client_emergency_stop_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for emergency_stop service connection...");
  }

  client_autoware_engage_ =
    this->create_client<tier4_external_api_msgs::srv::Engage>("service/autoware_engage");

  // Timer
  initTimer(1.0 / update_rate_);
}
}  // namespace autoware::joy_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::joy_controller::AutowareJoyControllerNode)
