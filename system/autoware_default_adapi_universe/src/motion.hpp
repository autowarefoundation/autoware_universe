// Copyright 2022 TIER IV, Inc.
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

#ifndef MOTION_HPP_
#define MOTION_HPP_

#include <autoware/adapi_specs/motion.hpp>
#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/component_interface_specs_universe/control.hpp>
#include <autoware/component_interface_specs_universe/localization.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <autoware/component_interface_utils/status.hpp>
#include <autoware/motion_utils/vehicle/vehicle_state_checker.hpp>
#include <rclcpp/rclcpp.hpp>

// This file should be included after messages.
#include "utils/types.hpp"

namespace autoware::default_adapi
{

class MotionNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit MotionNode(const rclcpp::NodeOptions & options);

private:
  // VehicleStopChecker owns an rclcpp subscription, so only its node-agnostic base is used here
  // and the odometry is fed from the subscription below, which covers the same topic and QoS.
  autoware::motion_utils::VehicleStopCheckerBase vehicle_stop_checker_;
  AUTOWARE_TIMER_PTR timer_;
  rclcpp::CallbackGroup::SharedPtr group_cli_;
  Srv<autoware::adapi_specs::motion::AcceptStart, NodeT> srv_accept_;
  Pub<autoware::adapi_specs::motion::State, NodeT> pub_state_;
  Cli<autoware::component_interface_specs_universe::control::SetPause, NodeT> cli_set_pause_;
  Sub<autoware::component_interface_specs_universe::control::IsPaused, NodeT> sub_is_paused_;
  Sub<autoware::component_interface_specs_universe::control::IsStartRequested, NodeT>
    sub_is_start_requested_;
  Sub<autoware::component_interface_specs_universe::localization::KinematicState, NodeT>
    sub_kinematic_state_;

  enum class State { Unknown, Pausing, Paused, Starting, Resuming, Resumed, Moving };
  State state_;
  std::optional<bool> is_paused_;
  std::optional<bool> is_start_requested_;

  double stop_check_duration_;
  bool require_accept_start_;
  bool is_calling_set_pause_;

  void update_state();
  void change_state(const State state);
  void update_pause(const State state);
  void change_pause(bool pause);
  void on_timer();
  void on_is_paused(
    const autoware::component_interface_specs_universe::control::IsPaused::Message::ConstSharedPtr
      msg);
  void on_is_start_requested(
    const autoware::component_interface_specs_universe::control::IsStartRequested::Message::
      ConstSharedPtr msg);
  void on_kinematic_state(
    const autoware::component_interface_specs_universe::localization::KinematicState::Message::
      ConstSharedPtr msg);
  void on_accept(
    const autoware::adapi_specs::motion::AcceptStart::Service::Request::SharedPtr req,
    const autoware::adapi_specs::motion::AcceptStart::Service::Response::SharedPtr res);
};

}  // namespace autoware::default_adapi

#endif  // MOTION_HPP_
