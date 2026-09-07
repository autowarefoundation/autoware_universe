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
#ifndef REDUNDANCY_SWITCHER_INTERFACE__CORE_LOGIC__PROCESSOR_HPP_
#define REDUNDANCY_SWITCHER_INTERFACE__CORE_LOGIC__PROCESSOR_HPP_

#include "redundancy_switcher_interface/core_logic/i_processor.hpp"

#include <vector>

namespace autoware::redundancy_switcher
{

class Processor : public IProcessor
{
public:
  Processor();

  std::vector<OutputCommand> handle(const InputEvent & event) override;
  DomainSnapshot snapshot() const override { return state_; }

private:
  DomainSnapshot state_;

  std::vector<OutputCommand> self_interruption(const SelfInterruptionEvent & e);
  std::vector<OutputCommand> reset(const ResetEvent & e);
  std::vector<OutputCommand> set_autoware_ready(const SetAutowareReadyEvent & e);
  std::vector<OutputCommand> set_velocity_status(const SetVelocityStatusEvent & e);
  std::vector<OutputCommand> set_control_mode(const SetControlModeEvent & e);
  std::vector<OutputCommand> set_switcher_signals(const SetSwitcherSignalsEvent & e);
  std::vector<OutputCommand> set_active_control_unit(const SetActiveControlUnitEvent & e);
  std::vector<OutputCommand> set_another_ecu_availability_timeout(
    const SetAnotherEcuAvailabilityTimeoutEvent & e);
  std::vector<OutputCommand> set_priority(const SetPriorityEvent & e);
};

}  // namespace autoware::redundancy_switcher
#endif  // REDUNDANCY_SWITCHER_INTERFACE__CORE_LOGIC__PROCESSOR_HPP_
