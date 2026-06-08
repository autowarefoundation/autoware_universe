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

#include "task.hpp"

#include "values.hpp"

#include <string>

namespace autoware::driving_mode_manager
{

TaskResult TrajectorySourceTask::execute(Interface & interface, GateStatus & gates)
{
  if (gates.status.trajectory_source == target_) {
    return TaskResult::kFinished;
  }
  if (stamp_) {
    const auto duration = (interface.now() - stamp_.value()).seconds();
    return timeout < duration ? TaskResult::kTimeout : TaskResult::kRunning;
  }
  stamp_ = interface.now();
  gates.expect.trajectory_source = target_;
  interface.change_trajectory_source(target_);
  return TaskResult::kRunning;
}

std::string TrajectorySourceTask::describe() const
{
  return "TrajectorySourceTask[" + std::to_string(target_.id) + "]";
}

TaskResult CommandSourceTask::execute(Interface & interface, GateStatus & gates)
{
  if (gates.status.command_source == target_) {
    return TaskResult::kFinished;
  }
  if (stamp_) {
    const auto duration = (interface.now() - stamp_.value()).seconds();
    return timeout < duration ? TaskResult::kTimeout : TaskResult::kRunning;
  }
  stamp_ = interface.now();
  gates.expect.command_source = target_;
  interface.change_command_source(target_);
  return TaskResult::kRunning;
}

std::string CommandSourceTask::describe() const
{
  return "CommandSourceTask[" + std::to_string(target_.id) + "]";
}

TaskResult PlatformModeTask::execute(Interface & interface, GateStatus & gates)
{
  if (gates.status.platform_mode == target_) {
    return TaskResult::kFinished;
  }
  if (stamp_) {
    const auto duration = (interface.now() - stamp_.value()).seconds();
    return timeout < duration ? TaskResult::kTimeout : TaskResult::kRunning;
  }
  stamp_ = interface.now();
  gates.expect.platform_mode = target_;
  interface.change_platform_mode(target_);
  return TaskResult::kRunning;
}

std::string PlatformModeTask::describe() const
{
  return "PlatformModeTask[" + to_string(target_) + "]";
}

TaskResult TransitionFilterTask::execute(Interface & interface, GateStatus & gates)
{
  if (gates.status.command_filter == target_) {
    return TaskResult::kFinished;
  }
  if (stamp_) {
    const auto duration = (interface.now() - stamp_.value()).seconds();
    return timeout < duration ? TaskResult::kTimeout : TaskResult::kRunning;
  }
  stamp_ = interface.now();
  gates.expect.command_filter = target_;
  interface.change_command_filter(target_);
  return TaskResult::kRunning;
}

std::string TransitionFilterTask::describe() const
{
  return "TransitionFilterTask[" + std::string(target_.flag ? "true" : "false") + "]";
}

TaskResult WaitModeReadyTask::execute(Interface & interface, GateStatus & gates)
{
  (void)interface;
  (void)gates;
  return TaskResult::kFinished;
}

std::string WaitModeReadyTask::describe() const
{
  return "WaitModeReadyTask";
}

TaskResult WaitModeStableTask::execute(Interface & interface, GateStatus & gates)
{
  (void)interface;
  (void)gates;
  return TaskResult::kFinished;
}

std::string WaitModeStableTask::describe() const
{
  return "WaitModeStableTask";
}

}  // namespace autoware::driving_mode_manager
