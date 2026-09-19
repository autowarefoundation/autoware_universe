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
#ifndef REDUNDANCY_SWITCHER_INTERFACE__PLUGIN__EVENT_GATEWAY_HPP_
#define REDUNDANCY_SWITCHER_INTERFACE__PLUGIN__EVENT_GATEWAY_HPP_

#include "redundancy_switcher_interface/core_logic/i_processor.hpp"
#include "redundancy_switcher_interface/ir/input_events.hpp"
#include "redundancy_switcher_interface/ir/output_commands.hpp"
#include "redundancy_switcher_interface/plugin/command_bus.hpp"

#include <memory>
#include <mutex>
#include <vector>

namespace autoware::redundancy_switcher
{

// dispatch(commands) runs outside the lock, so long I/O in execute() does not block other threads.
// Do not call submit() synchronously from within execute() — deadlock risk.
class EventGateway
{
public:
  EventGateway(std::shared_ptr<IProcessor> processor, std::shared_ptr<CommandBus> command_bus)
  : processor_(processor), command_bus_(command_bus)
  {
  }

  void submit(const InputEvent & event)
  {
    std::vector<OutputCommand> commands;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      commands = processor_->handle(event);
    }
    if (!commands.empty()) command_bus_->dispatch(commands);
  }

  // Returns after dispatch() completes; caller can extract ResetResultCommand from the result.
  std::vector<OutputCommand> submit_request(const InputEvent & event)
  {
    std::vector<OutputCommand> commands;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      commands = processor_->handle(event);
    }
    if (!commands.empty()) command_bus_->dispatch(commands);
    return commands;
  }

  DomainSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return processor_->snapshot();
  }

private:
  std::shared_ptr<IProcessor> processor_;
  std::shared_ptr<CommandBus> command_bus_;
  mutable std::mutex mutex_;
};

}  // namespace autoware::redundancy_switcher
#endif  // REDUNDANCY_SWITCHER_INTERFACE__PLUGIN__EVENT_GATEWAY_HPP_
