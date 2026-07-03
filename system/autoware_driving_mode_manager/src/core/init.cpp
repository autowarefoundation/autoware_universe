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
  config_->finalize();
  status_ = std::make_unique<DrivingModeStatus>(config_->autoware_modes());

  publish_driving_mode_info();
}

bool ManagerInit::is_ready() const
{
  if (!trajectory_source_) return false;
  if (!command_source_) return false;
  if (!command_filter_) return false;
  if (!platform_mode_) return false;
  if (!status_->is_ready()) return false;
  return true;
}

GateStatusItem ManagerInit::gates() const
{
  GateStatusItem gates;
  gates.trajectory_source = trajectory_source_.value();
  gates.command_source = command_source_.value();
  gates.command_filter = command_filter_.value();
  gates.platform_mode = platform_mode_.value();
  return gates;
}

void ManagerInit::update()
{
  // Set true for ignored flags;
  const auto stamp = interface_->now();
  for (const auto & mode : config_->autoware_modes()) {
    const auto & ignore = config_->ignore_flags(mode);
    const auto & data = status_->data(mode);
    if (ignore.available) data->available.update(stamp, true);
    if (ignore.active) data->active.update(stamp, true);
    if (ignore.stable) data->stable.update(stamp, true);
    if (ignore.continuable) data->continuable.update(stamp, true);
  }

  // Detect status timeout.
  status_->update(interface_->now(), 1.0);

  // Publish debug topics.
  if (interface_->get_enable_debug_topics()) {
    publish_debug_flags();
  }
}

void ManagerInit::on_trajectory_source(const TrajectorySource & source)
{
  trajectory_source_ = source;
}

void ManagerInit::on_command_source(const CommandSource & source)
{
  command_source_ = source;
}

void ManagerInit::on_command_filter(const CommandFilter & filter)
{
  command_filter_ = filter;
}

void ManagerInit::on_vehicle_control_mode(const PlatformMode & mode)
{
  platform_mode_ = mode;
}

void ManagerInit::on_available_flag(const AutowareMode & mode, bool flag)
{
  if (const auto & data = status_->data(mode)) {
    data->available.update(interface_->now(), flag);
  }
}

void ManagerInit::on_active_flag(const AutowareMode & mode, bool flag)
{
  if (const auto & data = status_->data(mode)) {
    data->active.update(interface_->now(), flag);
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

void ManagerInit::on_driving_mode_sync(const AutowareMode &, bool)
{
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

void ManagerInit::publish_debug_flags() const
{
  DebugFlags flags;
  for (const auto & mode : config_->autoware_modes()) {
    DebugFlags::Item item;
    item.available = status_->is_available(mode);
    item.active = status_->is_active(mode);
    item.stable = status_->is_stable(mode);
    item.continuable = status_->is_continuable(mode);
    flags.items[mode] = item;
  }
  interface_->publish_debug_flags(flags);
}

}  // namespace autoware::driving_mode_manager
