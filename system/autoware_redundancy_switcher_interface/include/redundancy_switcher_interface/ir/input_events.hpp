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
#ifndef REDUNDANCY_SWITCHER_INTERFACE__IR__INPUT_EVENTS_HPP_
#define REDUNDANCY_SWITCHER_INTERFACE__IR__INPUT_EVENTS_HPP_

#include "redundancy_switcher_interface/ir/domain_types.hpp"

#include <string>
#include <variant>
#include <vector>

namespace autoware::redundancy_switcher
{

/// Fired when this ECU detects an error and must notify the upper system.
struct SelfInterruptionEvent
{
  Annotated<std::monostate> value{{}, ""};
};

/// A reset request from an operator or equivalent source.
struct ResetEvent
{
  Annotated<std::monostate> value{{}, ""};
};

struct SetAutowareReadyEvent
{
  Annotated<AutowareReady> value;
};
struct SetVelocityStatusEvent
{
  Annotated<VelocityStatus> value;
};
struct SetControlModeEvent
{
  Annotated<ControlMode> value;
};

/// Timeout notification fired when CommandModeAvailability from the peer ECU
/// has not arrived within the configured interval.
struct SetAnotherEcuAvailabilityTimeoutEvent
{
  Annotated<bool> timed_out;
};

/// Fired when the switching state has been updated by the SwitcherAdapter.
/// Timeouts and disconnections are represented as SwitcherSignals with is_faulted=true.
struct SetSwitcherSignalsEvent
{
  Annotated<SwitcherSignals> value;
};
struct SetActiveControlUnitEvent
{
  Annotated<ActiveControlUnit> value;
};

using InputEvent = std::variant<
  SelfInterruptionEvent, ResetEvent, SetAutowareReadyEvent, SetVelocityStatusEvent,
  SetControlModeEvent, SetSwitcherSignalsEvent, SetActiveControlUnitEvent,
  SetAnotherEcuAvailabilityTimeoutEvent>;

}  // namespace autoware::redundancy_switcher
#endif  // REDUNDANCY_SWITCHER_INTERFACE__IR__INPUT_EVENTS_HPP_
