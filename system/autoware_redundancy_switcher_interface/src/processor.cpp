//  Copyright 2025 The Autoware Contributors
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
#include "redundancy_switcher_interface/core_logic/processor.hpp"

#include <redundancy_switcher_interface/detail/overloaded.hpp>

#include <string>
#include <variant>
#include <vector>

namespace autoware::redundancy_switcher
{

Processor::Processor() = default;

std::vector<OutputCommand> Processor::handle(const InputEvent & event)
{
  return std::visit(
    overloaded{
      [this](const SelfInterruptionEvent & e) { return self_interruption(e); },
      [this](const ResetEvent & e) { return reset(e); },
      [this](const SetAutowareReadyEvent & e) { return set_autoware_ready(e); },
      [this](const SetVelocityStatusEvent & e) { return set_velocity_status(e); },
      [this](const SetControlModeEvent & e) { return set_control_mode(e); },
      [this](const SetSwitcherSignalsEvent & e) { return set_switcher_signals(e); },
      [this](const SetActiveControlUnitEvent & e) { return set_active_control_unit(e); },
      [this](const SetAnotherEcuAvailabilityTimeoutEvent & e) {
        return set_another_ecu_availability_timeout(e);
      },
      [this](const SetPriorityEvent & e) { return set_priority(e); }},
    event);
}

std::vector<OutputCommand> Processor::self_interruption(const SelfInterruptionEvent &)
{
  if (!state_.autoware_ready.has_value() || state_.autoware_ready->value != AutowareReady::True) {
    return {LogCommand{LogLevel::Debug, "Self-interruption rejected: Autoware is not ready."}};
  }
  if (!state_.control_mode.has_value() || state_.control_mode->value != ControlMode::Auto) {
    return {LogCommand{LogLevel::Debug, "Self-interruption rejected: Autoware is not in control."}};
  }
  if (!state_.switcher.has_value()) {
    return {LogCommand{
      LogLevel::Warn, "Self-interruption rejected: startup not yet complete (no switcher data)."}};
  }

  const auto & sw = *state_.switcher;
  if (sw.value.is_self_interrupted) {
    return {LogCommand{
      LogLevel::Debug, "Self-interruption rejected: already interrupted. " + sw.annotation}};
  }
  if (sw.value.is_faulted) {
    return {
      LogCommand{LogLevel::Error, "Self-interruption rejected: switcher fault. " + sw.annotation}};
  }
  if (sw.value.is_stable) {
    return {
      SelfInterruptionCommand{},
      LogCommand{LogLevel::Info, "Self-interruption accepted. " + sw.annotation}};
  }
  // Transitional state (startup or state change in progress)
  return {LogCommand{
    LogLevel::Info,
    "Self-interruption rejected: switcher in transitional state. " + sw.annotation}};
}

std::vector<OutputCommand> Processor::reset(const ResetEvent &)
{
  // Reject only when velocity is known and the vehicle is moving; unknown (nullopt) is treated as
  // stopped.
  if (
    state_.velocity_status.has_value() &&
    state_.velocity_status->value != VelocityStatus::Stopped) {
    const std::string msg = "Reset rejected: vehicle is not stopped.";
    return {
      ResetResultCommand{false, ResetRejectedReason::Ignored, msg},
      LogCommand{LogLevel::Warn, msg}};
  }

  // Always accept reset when Autoware is not ready (including nullopt).
  if (!state_.autoware_ready.has_value() || state_.autoware_ready->value != AutowareReady::True) {
    const std::string msg = "Reset accepted (initializing).";
    return {ResetCommand{}, ResetResultCommand{true, {}, msg}, LogCommand{LogLevel::Info, msg}};
  }

  // Switcher data not yet received (startup not yet complete).
  if (!state_.switcher.has_value()) {
    const std::string msg = "Reset rejected: startup not yet complete (no switcher data).";
    return {
      ResetResultCommand{false, ResetRejectedReason::Error, msg}, LogCommand{LogLevel::Error, msg}};
  }

  const auto & sw = *state_.switcher;
  if (sw.value.is_self_interrupted) {
    const std::string msg = "Reset accepted. " + sw.annotation;
    return {ResetCommand{}, ResetResultCommand{true, {}, msg}, LogCommand{LogLevel::Info, msg}};
  }
  if (sw.value.is_stable) {
    const std::string msg = "Reset not necessary: switcher already stable. " + sw.annotation;
    return {
      ResetResultCommand{false, ResetRejectedReason::NotNecessary, msg},
      LogCommand{LogLevel::Info, msg}};
  }
  if (sw.value.is_faulted) {
    const std::string msg = "Reset rejected: switcher fault. " + sw.annotation;
    return {
      ResetResultCommand{false, ResetRejectedReason::Error, msg}, LogCommand{LogLevel::Error, msg}};
  }
  // Transitional state (startup or state change in progress)
  const std::string msg = "Reset rejected: switcher in transitional state. " + sw.annotation;
  return {
    ResetResultCommand{false, ResetRejectedReason::Error, msg}, LogCommand{LogLevel::Warn, msg}};
}

std::vector<OutputCommand> Processor::set_autoware_ready(const SetAutowareReadyEvent & e)
{
  const bool changed =
    !state_.autoware_ready.has_value() || state_.autoware_ready->value != e.value.value;
  state_.autoware_ready = e.value;

  std::vector<OutputCommand> commands;
  // Notify SwitcherAdapter to update its local cache of autoware_ready.
  commands.push_back(UpdateAutowareReadyCommand{e.value.value});
  if (changed) {
    commands.push_back(UpdateStatusDiagCommand{state_});
  }
  commands.push_back(
    LogCommand{
      LogLevel::Info, std::string("Autoware ready: ") +
                        (e.value.value == AutowareReady::True ? "true" : "false") +
                        (e.value.annotation.empty() ? "" : " (" + e.value.annotation + ")")});
  return commands;
}

std::vector<OutputCommand> Processor::set_velocity_status(const SetVelocityStatusEvent & e)
{
  const bool changed = !state_.velocity_status.has_value() ||
                       state_.velocity_status->value != e.value.value ||
                       state_.velocity_status->annotation != e.value.annotation;
  state_.velocity_status = e.value;
  if (!changed) {
    return {};
  }
  return {
    UpdateStatusDiagCommand{state_},
    LogCommand{
      LogLevel::Info, std::string("Vehicle stopped: ") +
                        (e.value.value == VelocityStatus::Stopped ? "true" : "false") +
                        (e.value.annotation.empty() ? "" : " (" + e.value.annotation + ")")}};
}

std::vector<OutputCommand> Processor::set_control_mode(const SetControlModeEvent & e)
{
  const bool changed = !state_.control_mode.has_value() ||
                       state_.control_mode->value != e.value.value ||
                       state_.control_mode->annotation != e.value.annotation;
  state_.control_mode = e.value;
  if (!changed) {
    return {};
  }
  return {
    UpdateStatusDiagCommand{state_},
    LogCommand{
      LogLevel::Info, std::string("Autoware control: ") +
                        (e.value.value == ControlMode::Auto ? "true" : "false") +
                        (e.value.annotation.empty() ? "" : " (" + e.value.annotation + ")")}};
}

std::vector<OutputCommand> Processor::set_switcher_signals(const SetSwitcherSignalsEvent & e)
{
  const bool changed =
    !state_.switcher.has_value() || state_.switcher->value.is_stable != e.value.value.is_stable ||
    state_.switcher->value.is_self_interrupted != e.value.value.is_self_interrupted ||
    state_.switcher->value.is_faulted != e.value.value.is_faulted ||
    state_.switcher->annotation != e.value.annotation;

  state_.switcher = e.value;

  std::vector<OutputCommand> commands;
  if (e.value.value.is_self_interrupted || e.value.value.is_faulted) {
    // While self-interrupted/faulted, active control unit must be represented as empty.
    commands.push_back(UpdateActiveControlUnitCommand{ActiveControlUnit{}});
  }
  if (changed) {
    commands.push_back(UpdateStatusDiagCommand{state_});
    commands.push_back(LogCommand{LogLevel::Info, "Switcher state updated: " + e.value.annotation});
  }
  return commands;
}

std::vector<OutputCommand> Processor::set_active_control_unit(const SetActiveControlUnitEvent & e)
{
  if (
    state_.switcher.has_value() &&
    (state_.switcher->value.is_self_interrupted || state_.switcher->value.is_faulted)) {
    return {UpdateActiveControlUnitCommand{ActiveControlUnit{}}};
  }
  return {UpdateActiveControlUnitCommand{e.value.value}};
}

std::vector<OutputCommand> Processor::set_another_ecu_availability_timeout(
  const SetAnotherEcuAvailabilityTimeoutEvent & e)
{
  state_.another_ecu_availability_timeout = e.timed_out.value;
  const std::string msg =
    std::string("Another ECU availability timeout: ") + (e.timed_out.value ? "true" : "false") +
    (e.timed_out.annotation.empty() ? "" : " (" + e.timed_out.annotation + ")");
  return {
    UpdateAnotherEcuAvailabilityTimeoutCommand{e.timed_out.value},
    LogCommand{e.timed_out.value ? LogLevel::Warn : LogLevel::Info, msg}};
}

std::vector<OutputCommand> Processor::set_priority(const SetPriorityEvent & e)
{
  const bool changed = !state_.priority.has_value() || *state_.priority != e.value.value;
  state_.priority = e.value.value;
  if (!changed) {
    return {};
  }
  return {
    UpdatePriorityCommand{e.value.value},
    LogCommand{
      LogLevel::Info, "Priority updated: " + std::to_string(e.value.value) +
                        (e.value.annotation.empty() ? "" : " (" + e.value.annotation + ")")}};
}

}  // namespace autoware::redundancy_switcher
