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

#include "init.hpp"

#include <memory>
#include <utility>

namespace autoware::driving_mode_manager
{

ManagerInit::ManagerInit(std::unique_ptr<Interface> && interface, std::shared_ptr<Plugin> plugin)
{
  interface_ = std::move(interface);
  interface_->init(this);

  config_ = std::make_unique<DrivingModeConfig>();
  plugin_ = plugin;
  plugin_->setup(*config_);
  config_->validate();
  status_ = std::make_unique<DrivingModeStatus>(config_->autoware_modes());

  publish_driving_mode_info();
}

bool ManagerInit::is_ready() const
{
  if (!trajectory_source) return false;
  if (!command_source) return false;
  if (!command_filter) return false;
  if (!platform_mode) return false;
  if (!status_->is_ready()) return false;
  // TODO(isamu-takagi): check all mrm state
  return true;
}

GateStatusItem ManagerInit::gates() const
{
  GateStatusItem gates;
  gates.trajectory_source = trajectory_source.value();
  gates.command_source = command_source.value();
  gates.command_filter = command_filter.value();
  gates.platform_mode = platform_mode.value();
  return gates;
}

void ManagerInit::update()
{
  status_->update(interface_->now(), 1.0);
  publish_debug();
}

void ManagerInit::on_trajectory_source(const TrajectorySource & source)
{
  trajectory_source = source;
}

void ManagerInit::on_command_source(const CommandSource & source)
{
  command_source = source;
}

void ManagerInit::on_command_filter(const CommandFilter & filter)
{
  command_filter = filter;
}

void ManagerInit::on_vehicle_control_mode(const PlatformMode & mode)
{
  platform_mode = mode;
}

void ManagerInit::on_available_flag(const AutowareMode & mode, bool flag)
{
  if (const auto & data = status_->data(mode)) {
    data->available.update(interface_->now(), flag);
  }
}

void ManagerInit::on_stable_flag(const AutowareMode & mode, bool flag)
{
  if (const auto & data = status_->data(mode)) {
    data->stable.update(interface_->now(), flag);
  }
}

void ManagerInit::on_continuable_flag(const AutowareMode & mode, bool flag)
{
  if (const auto & data = status_->data(mode)) {
    data->continuable.update(interface_->now(), flag);
  }
}

void ManagerInit::on_mrm_state(const AutowareMode & mode, const MrmState::State & state)
{
  mrm_states_[mode] = state;
}

ServiceResponse ManagerInit::change_operation_mode(const OperationMode &)
{
  return ServiceResponse{false, "driving mode manager is not ready"};
}

ServiceResponse ManagerInit::change_autoware_control(const AutowareControl &)
{
  return ServiceResponse{false, "driving mode manager is not ready"};
}

ServiceResponse ManagerInit::change_mrm_request(const MrmRequest &)
{
  return ServiceResponse{false, "driving mode manager is not ready"};
}

void ManagerInit::publish_driving_mode_info() const
{
  ModeInfo info;
  for (const auto & mode : config_->autoware_modes()) {
    const auto name = config_->name(mode);
    if (!name.empty()) {
      info.names[mode] = name;
    }
  }
  interface_->publish_driving_mode_info(info);
}

void ManagerInit::publish_debug() const
{
  DebugStatus debug;
  for (const auto & mode : config_->autoware_modes()) {
    DebugStatus::Flag flag;
    flag.available = status_->is_available(mode);
    flag.stable = status_->is_stable(mode);
    flag.continuable = status_->is_continuable(mode);
    debug.flags[mode] = flag;
  }
  interface_->publish_debug(debug);
}

}  // namespace autoware::driving_mode_manager
