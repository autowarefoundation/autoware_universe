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

#ifndef AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_
#define AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_

#include "autoware_utils/ros/polling_subscriber.hpp"

#include <autoware_raw_vehicle_cmd_converter/accel_map.hpp>
#include <autoware_raw_vehicle_cmd_converter/brake_map.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/manual_operator_heartbeat.hpp>
#include <autoware_adapi_v1_msgs/msg/pedals_command.hpp>
#include <autoware_adapi_v1_msgs/msg/steering_command.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>

#include <memory>
#include <string>

class TestExternalCmdConverter;

namespace autoware::external_cmd_converter
{

using autoware::raw_vehicle_cmd_converter::AccelMap;
using autoware::raw_vehicle_cmd_converter::BrakeMap;
using autoware_adapi_v1_msgs::msg::ManualOperatorHeartbeat;
using autoware_adapi_v1_msgs::msg::PedalsCommand;
using autoware_adapi_v1_msgs::msg::SteeringCommand;
using autoware_control_msgs::msg::Control;
using autoware_vehicle_msgs::msg::GearCommand;
using nav_msgs::msg::Odometry;
using tier4_control_msgs::msg::GateMode;

class ExternalCmdConverterNode : public rclcpp::Node
{
public:
  explicit ExternalCmdConverterNode(const rclcpp::NodeOptions & node_options);

private:
  // Publisher
  rclcpp::Publisher<Control>::SharedPtr cmd_pub_;

  // Subscriber
  rclcpp::Subscription<PedalsCommand>::SharedPtr pedals_cmd_sub_;
  rclcpp::Subscription<ManualOperatorHeartbeat>::SharedPtr heartbeat_sub_;

  // Polling Subscriber
  template <typename T>
  using PollingSubscriber = autoware_utils::InterProcessPollingSubscriber<T>;
  PollingSubscriber<SteeringCommand> steering_cmd_sub_{this, "in/steering_cmd"};
  PollingSubscriber<Odometry> velocity_sub_{this, "in/odometry"};
  PollingSubscriber<GearCommand> gear_cmd_sub_{this, "in/gear_cmd"};
  PollingSubscriber<GateMode> gate_mode_sub_{this, "in/current_gate_mode"};

  void on_pedals_cmd(const PedalsCommand::ConstSharedPtr cmd_ptr);
  void on_heartbeat(const ManualOperatorHeartbeat::ConstSharedPtr msg);

  Odometry::ConstSharedPtr current_velocity_ptr_{nullptr};  // [m/s]
  GearCommand::ConstSharedPtr current_gear_cmd_{nullptr};
  GateMode::ConstSharedPtr current_gate_mode_{nullptr};

  std::shared_ptr<rclcpp::Time> latest_heartbeat_received_time_;
  std::shared_ptr<rclcpp::Time> latest_cmd_received_time_;

  // Timer
  void on_timer();
  rclcpp::TimerBase::SharedPtr rate_check_timer_;

  // Parameter
  double ref_vel_gain_;  // reference velocity = current velocity + desired acceleration * gain
  bool wait_for_first_topic_;
  double control_command_timeout_;
  double emergency_stop_timeout_;

  // Diagnostics
  diagnostic_updater::Updater updater_{this};

  void check_topic_status(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void check_emergency_stop(diagnostic_updater::DiagnosticStatusWrapper & stat);
  bool check_emergency_stop_topic_timeout();
  bool check_remote_topic_rate();

  // Algorithm
  AccelMap accel_map_;
  BrakeMap brake_map_;
  bool acc_map_initialized_;

  double calculate_acc(const PedalsCommand & cmd, const double vel);
  double get_gear_velocity_sign(const GearCommand & cmd);

  friend class ::TestExternalCmdConverter;
};

}  // namespace autoware::external_cmd_converter

#endif  // AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_
