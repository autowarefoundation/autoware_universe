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

template <typename T>
std::invalid_argument invalid_argument_with_id(const std::string & message, const T & data)
{
  return std::invalid_argument(message + ": " + std::to_string(data.id));
}

void DrivingModeConfig::define_autoware_mode(
  const AutowareMode & autoware_mode, const OperationMode & operation_mode, uint16_t priority)
{
  if (autoware_modes_.count(autoware_mode)) {
    throw invalid_argument_with_id("this mode is already defined", autoware_mode);
  }
  autoware_modes_[autoware_mode].priority = priority;
  autoware_modes_[autoware_mode].operation_mode = operation_mode;
  operation_to_autoware_[operation_mode] = autoware_mode;
}

void DrivingModeConfig::define_autoware_mode(
  const AutowareMode & autoware_mode, const MrmBehavior & mrm_behavior, uint16_t priority)
{
  if (autoware_modes_.count(autoware_mode)) {
    throw invalid_argument_with_id("this mode is already defined", autoware_mode);
  }
  autoware_modes_[autoware_mode].priority = priority;
  autoware_modes_[autoware_mode].mrm_behavior = mrm_behavior;
  mrm_to_autoware_[mrm_behavior] = autoware_mode;
}

void DrivingModeConfig::define_trajectory_source(const TrajectorySource & source)
{
  trajectory_sources_.insert(source);
}

void DrivingModeConfig::define_command_source(const CommandSource & source)
{
  command_sources_.insert(source);
}

void DrivingModeConfig::bind_name(const AutowareMode & autoware_mode, const std::string & name)
{
  if (autoware_modes_.count(autoware_mode) == 0) {
    throw invalid_argument_with_id("unknown autoware mode", autoware_mode);
  }
  autoware_modes_.at(autoware_mode).name = name;
}

void DrivingModeConfig::bind_gates(const AutowareMode & autoware_mode, const Gates & gates)
{
  if (autoware_modes_.count(autoware_mode) == 0) {
    throw invalid_argument_with_id("unknown autoware mode", autoware_mode);
  }
  if (gates.trajectory && trajectory_sources_.count(*gates.trajectory) == 0) {
    throw invalid_argument_with_id("unknown trajectory source", *gates.trajectory);
  }
  if (gates.command && command_sources_.count(*gates.command) == 0) {
    throw invalid_argument_with_id("unknown command source", *gates.command);
  }
  autoware_modes_.at(autoware_mode).gates = gates;
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

bool DrivingModeConfig::exists(const AutowareMode & autoware_mode) const
{
  return autoware_modes_.count(autoware_mode) != 0;
}

std::string DrivingModeConfig::name(const AutowareMode & autoware_mode) const
{
  const auto iter = autoware_modes_.find(autoware_mode);
  return iter == autoware_modes_.end() ? "" : iter->second.name;
}

DrivingModeConfig::Gates DrivingModeConfig::gates(const AutowareMode & autoware_mode) const
{
  return autoware_modes_.at(autoware_mode).gates;
}

AutowareMode DrivingModeConfig::to_autoware_mode(const OperationMode & operation_mode) const
{
  return operation_to_autoware_.at(operation_mode);
}

std::optional<AutowareMode> DrivingModeConfig::to_autoware_mode(
  const MrmBehavior & mrm_behavior) const
{
  const auto iter = mrm_to_autoware_.find(mrm_behavior);
  return iter == mrm_to_autoware_.end() ? std::nullopt : std::optional(iter->second);
}

OperationMode DrivingModeConfig::to_operation_mode(const AutowareMode & autoware_mode) const
{
  return autoware_modes_.at(autoware_mode).operation_mode.value();
}

std::optional<MrmBehavior> DrivingModeConfig::to_mrm_behavior(
  const AutowareMode & autoware_mode) const
{
  const auto iter = autoware_modes_.find(autoware_mode);
  return iter == autoware_modes_.end() ? std::nullopt : iter->second.mrm_behavior;
}

uint16_t DrivingModeConfig::priority(const AutowareMode & autoware_mode) const
{
  const auto iter = autoware_modes_.find(autoware_mode);
  return iter == autoware_modes_.end() ? 0 : iter->second.priority;
}

}  // namespace autoware::driving_mode_manager
