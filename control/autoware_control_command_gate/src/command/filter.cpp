// Copyright 2025 The Autoware Contributors
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

#include "filter.hpp"

#include <utility>

namespace autoware::control_command_gate
{

CommandFilter::CommandFilter(std::unique_ptr<CommandOutput> && output, rclcpp::Node & node)
: CommandBridge(std::move(output)), node_(node), vehicle_status_(node)
{
  enable_command_limit_filter_ = node_.declare_parameter<bool>("enable_command_limit_filter");
  transition_flag_ = false;
}

void CommandFilter::set_nominal_filter_params(const VehicleCmdFilterParam & p)
{
  nominal_filter_.setParam(p);
}

void CommandFilter::set_transition_filter_params(const VehicleCmdFilterParam & p)
{
  transition_filter_.setParam(p);
}

void CommandFilter::set_transition_flag(bool flag)
{
  transition_flag_ = flag;
}

double CommandFilter::get_delta_time()
{
  const auto curr_time = node_.now();
  if (!prev_time_) {
    prev_time_ = curr_time;
    return 0.0;
  }
  const auto delta_time = (curr_time - *prev_time_).seconds();
  prev_time_ = curr_time;
  return delta_time;
}

Control CommandFilter::filter_command(const Control & msg)
{
  const auto dt = get_delta_time();
  const auto current_steering = vehicle_status_.get_current_steering();
  const auto current_velocity = vehicle_status_.get_current_velocity();

  IsFilterActivated is_filter_activated;
  Control out = msg;

  nominal_filter_.setCurrentSpeed(current_velocity);
  transition_filter_.setCurrentSpeed(current_velocity);

  const auto & filter = transition_flag_ ? transition_filter_ : nominal_filter_;
  filter.filterAll(dt, current_steering, out, is_filter_activated);

  // set prev value for both to keep consistency over switching:
  // Actual steer, vel, acc should be considered in manual mode to prevent sudden motion when
  // switching from manual to autonomous
  const auto is_autoware_control_enabled = vehicle_status_.is_autoware_control_enabled();
  const auto is_vehicle_stopped = vehicle_status_.is_vehicle_stopped();
  const auto current_status_command = vehicle_status_.get_actual_status_as_command();
  Control prev_command = is_autoware_control_enabled ? out : current_status_command;
  if (is_vehicle_stopped) {
    prev_command.longitudinal = out.longitudinal;
  }

  // TODO(Horibe): To prevent sudden acceleration/deceleration when switching from manual to
  // autonomous, the filter should be applied for actual speed and acceleration during manual
  // driving. However, this means that the output command from Gate will always be close to the
  // driving state during manual driving. Here, let autoware publish the stop command when the ego
  // is stopped to intend the autoware is trying to keep stopping.
  nominal_filter_.setPrevCmd(prev_command);
  transition_filter_.setPrevCmd(prev_command);

  // TODO(Takagi, Isamu): Publish debug information.
  // is_filter_activated.stamp = node_.now();
  // is_filter_activated_pub_->publish(is_filter_activated);
  // publishMarkers(is_filter_activated);

  return out;
}

void CommandFilter::on_control(const Control & msg)
{
  const auto out = enable_command_limit_filter_ ? filter_command(msg) : msg;
  CommandBridge::on_control(out);
}

}  // namespace autoware::control_command_gate
