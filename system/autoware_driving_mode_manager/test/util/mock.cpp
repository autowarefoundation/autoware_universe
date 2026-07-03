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

#include "mock.hpp"

#include <iostream>
#include <string>

void MockInterface::init(MainLogic * logic)
{
  logic_ = logic;
}

bool MockInterface::get_enable_debug_topics() const
{
  return true;
}

rclcpp::Time MockInterface::now() const
{
  return rclcpp::Time{};
}

void MockInterface::change_trajectory_source(const TrajectorySource & source)
{
  trajectory_source = source;
  response_trajectory_source = source;
}

void MockInterface::change_command_source(const CommandSource & source)
{
  command_source = source;
  response_command_source = source;
}

void MockInterface::change_command_filter(const CommandFilter & filter)
{
  command_filter = filter;
  response_command_filter = filter;
}

void MockInterface::change_platform_mode(const PlatformMode & mode)
{
  platform_mode = mode;
  response_platform_mode = mode;
}

void MockInterface::publish_operation_mode(const OperationModeState & state)
{
  operation_mode_state = state;
}

void MockInterface::publish_mrm_state(const MrmState & state)
{
  (void)state;
}

void MockInterface::publish_driving_mode_request(const ModeRequest & request)
{
  (void)request;
}

void MockInterface::publish_driving_mode_sync(const AutowareModeSet & modes)
{
  (void)modes;
}

void MockInterface::publish_driving_mode_info(const ModeInfo & info)
{
  (void)info;
}

void MockInterface::publish_debug_flags(const DebugFlags & flags)
{
  (void)flags;
}

void MockInterface::publish_debug_request(const RequestModes & request)
{
  (void)request;
}

void MockInterface::log_info(const std::string & message)
{
  (void)message;
}

void MockInterface::log_warn(const std::string & message)
{
  (void)message;
}

void MockInterface::log_error(const std::string & message)
{
  (void)message;
}

void MockInterface::log_debug(const std::string & message)
{
  (void)message;
}

void MockInterface::update()
{
  if (response_trajectory_source) {
    logic_->on_trajectory_source(response_trajectory_source.value());
    response_trajectory_source.reset();
  }
  if (response_command_source) {
    logic_->on_command_source(response_command_source.value());
    response_command_source.reset();
  }
  if (response_command_filter) {
    logic_->on_command_filter(response_command_filter.value());
    response_command_filter.reset();
  }
  if (response_platform_mode) {
    logic_->on_vehicle_control_mode(response_platform_mode.value());
    response_platform_mode.reset();
  }
}
