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

#include "config.hpp"

#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::driving_mode_manager
{

void DrivingModeConfig::define_autoware_mode(
  const AutowareMode & mode, const OperationMode & opmode, uint16_t priority)
{
  if (autoware_modes_.count(mode)) {
    throw std::invalid_argument("this mode is already defined: " + std::to_string(mode.id));
  }
  autoware_modes_[mode].priority = priority;
  autoware_modes_[mode].operation_mode = opmode;
  operation_to_autoware_[opmode] = mode;
}

void DrivingModeConfig::define_autoware_mode(
  const AutowareMode & mode, const MrmBehavior & behavior, uint16_t priority)
{
  if (autoware_modes_.count(mode)) {
    throw std::invalid_argument("this mode is already defined: " + std::to_string(mode.id));
  }
  autoware_modes_[mode].priority = priority;
  autoware_modes_[mode].mrm_behavior = behavior;
  mrm_to_autoware_[behavior] = mode;
}

void DrivingModeConfig::define_trajectory_source(const TrajectorySource & source)
{
  trajectory_sources_.insert(source);
}

void DrivingModeConfig::define_command_source(const CommandSource & source)
{
  command_sources_.insert(source);
}

void DrivingModeConfig::bind_name(const AutowareMode & mode, const std::string & name)
{
  if (autoware_modes_.count(mode) == 0) {
    throw std::invalid_argument("unknown autoware mode: " + std::to_string(mode.id));
  }
  autoware_modes_.at(mode).name = name;
}

void DrivingModeConfig::bind_gates(const AutowareMode & mode, const Gates & gates)
{
  if (autoware_modes_.count(mode) == 0) {
    const auto id = std::to_string(mode.id);
    throw std::invalid_argument("unknown autoware mode: " + id);
  }
  if (gates.trajectory && trajectory_sources_.count(*gates.trajectory) == 0) {
    const auto id = std::to_string(gates.trajectory->id);
    throw std::invalid_argument("unknown trajectory source: " + id);
  }
  if (gates.command && command_sources_.count(*gates.command) == 0) {
    const auto id = std::to_string(gates.command->id);
    throw std::invalid_argument("unknown command source: " + id);
  }
  autoware_modes_.at(mode).gates = gates;
}

void DrivingModeConfig::finalize()
{
  for (const auto & [mode, config] : autoware_modes_) {
    autoware_modes_list_.push_back(mode);
  }
}

std::vector<AutowareMode> DrivingModeConfig::autoware_modes() const
{
  return autoware_modes_list_;
}

bool DrivingModeConfig::exists(const AutowareMode & mode) const
{
  return autoware_modes_.count(mode) != 0;
}

std::string DrivingModeConfig::name(const AutowareMode & mode) const
{
  const auto iter = autoware_modes_.find(mode);
  return iter == autoware_modes_.end() ? "" : iter->second.name;
}

DrivingModeConfig::Gates DrivingModeConfig::gates(const AutowareMode & mode) const
{
  return autoware_modes_.at(mode).gates;
}

AutowareMode DrivingModeConfig::to_autoware_mode(const OperationMode & opmode) const
{
  return operation_to_autoware_.at(opmode);
}

std::optional<AutowareMode> DrivingModeConfig::to_autoware_mode(const MrmBehavior & behavior) const
{
  const auto iter = mrm_to_autoware_.find(behavior);
  return iter == mrm_to_autoware_.end() ? std::nullopt : std::optional(iter->second);
}

OperationMode DrivingModeConfig::to_operation_mode(const AutowareMode & mode) const
{
  return autoware_modes_.at(mode).operation_mode.value();
}

std::optional<MrmBehavior> DrivingModeConfig::to_mrm_behavior(const AutowareMode & mode) const
{
  const auto iter = autoware_modes_.find(mode);
  return iter == autoware_modes_.end() ? std::nullopt : iter->second.mrm_behavior;
}

uint16_t DrivingModeConfig::priority(const AutowareMode & mode) const
{
  const auto iter = autoware_modes_.find(mode);
  return iter == autoware_modes_.end() ? 0 : iter->second.priority;
}

}  // namespace autoware::driving_mode_manager
