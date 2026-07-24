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

#include "debug.hpp"

#include <string>

namespace autoware::driving_mode_manager
{

Task * TaskList::get() const
{
  if (!platform_.empty()) return platform_.front().get();
  if (!autoware_.empty()) return autoware_.front().get();
  if (!finalize_.empty()) return finalize_.front().get();
  return nullptr;
}

void TaskList::pop()
{
  if (!platform_.empty()) return platform_.pop();
  if (!autoware_.empty()) return autoware_.pop();
  if (!finalize_.empty()) return finalize_.pop();
}

bool TaskList::interruptible() const
{
  // Finalize tasks are interruptible.
  if (!platform_.empty()) return false;
  if (!autoware_.empty()) return false;
  return true;
}

bool TaskList::empty() const
{
  if (!platform_.empty()) return false;
  if (!autoware_.empty()) return false;
  if (!finalize_.empty()) return false;
  return true;
}

TaskResult PlatformModeTask::execute(
  Interface & interface, GateStatus & gates, const DrivingModeStatus &)
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

TaskResult TrajectorySourceTask::execute(
  Interface & interface, GateStatus & gates, const DrivingModeStatus &)
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

TaskResult CommandSourceTask::execute(
  Interface & interface, GateStatus & gates, const DrivingModeStatus &)
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

TaskResult CommandFilterTask::execute(
  Interface & interface, GateStatus & gates, const DrivingModeStatus &)
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

std::string CommandFilterTask::describe() const
{
  return "CommandFilterTask[" + std::string(target_.flag ? "true" : "false") + "]";
}

TaskResult WaitModeActiveTask::execute(
  Interface & interface, GateStatus &, const DrivingModeStatus & status)
{
  if (status.is_active(mode_)) {
    return TaskResult::kFinished;
  }
  if (stamp_) {
    const auto duration = (interface.now() - stamp_.value()).seconds();
    return timeout < duration ? TaskResult::kTimeout : TaskResult::kRunning;
  }
  stamp_ = interface.now();
  return TaskResult::kRunning;
}

std::string WaitModeActiveTask::describe() const
{
  return "WaitModeActiveTask[" + std::to_string(mode_.id) + "]";
}

TaskResult WaitModeStableTask::execute(
  Interface & interface, GateStatus &, const DrivingModeStatus & status)
{
  if (status.is_stable(mode_)) {
    return TaskResult::kFinished;
  }
  if (stamp_) {
    const auto duration = (interface.now() - stamp_.value()).seconds();
    return timeout < duration ? TaskResult::kTimeout : TaskResult::kRunning;
  }
  stamp_ = interface.now();
  return TaskResult::kRunning;
}

std::string WaitModeStableTask::describe() const
{
  return "WaitModeStableTask[" + std::to_string(mode_.id) + "]";
}

}  // namespace autoware::driving_mode_manager
