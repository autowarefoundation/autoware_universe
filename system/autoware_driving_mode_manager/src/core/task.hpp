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

#ifndef CORE__TASK_HPP_
#define CORE__TASK_HPP_

#include "type/data.hpp"
#include "type/interface.hpp"

#include <rclcpp/time.hpp>

#include <optional>
#include <string>

namespace autoware::driving_mode_manager
{

enum class TaskPhase {
  kAutowareMode,
  kPlatformMode,
  kWaitStable,
  kOverridden,
  kAborted,
  kCompleted,
};

enum class TaskResult {
  kFinished,
  kRunning,
  kTimeout,
};

class Task
{
public:
  virtual ~Task() = default;
  virtual TaskResult execute(Interface & interface, GateStatus & gates) = 0;
  virtual std::string describe() const = 0;
};

class TrajectorySourceTask : public Task
{
public:
  explicit TrajectorySourceTask(const TrajectorySource & target) : target_(target) {}
  TaskResult execute(Interface & interface, GateStatus & gates) override;
  std::string describe() const override;

private:
  static constexpr double timeout = 3.0;
  const TrajectorySource target_;
  std::optional<rclcpp::Time> stamp_;
};

class CommandSourceTask : public Task
{
public:
  explicit CommandSourceTask(const CommandSource & target) : target_(target) {}
  TaskResult execute(Interface & interface, GateStatus & gates) override;
  std::string describe() const override;

private:
  static constexpr double timeout = 3.0;
  const CommandSource target_;
  std::optional<rclcpp::Time> stamp_;
};

class PlatformModeTask : public Task
{
public:
  explicit PlatformModeTask(const PlatformMode & target) : target_(target) {}
  TaskResult execute(Interface & interface, GateStatus & gates) override;
  std::string describe() const override;

private:
  static constexpr double timeout = 3.0;
  const PlatformMode target_;
  std::optional<rclcpp::Time> stamp_;
};

class TransitionFilterTask : public Task
{
public:
  explicit TransitionFilterTask(const CommandFilter & target) : target_(target) {}
  TaskResult execute(Interface & interface, GateStatus & gates) override;
  std::string describe() const override;

private:
  static constexpr double timeout = 3.0;
  const CommandFilter target_;
  std::optional<rclcpp::Time> stamp_;
};

class WaitModeReadyTask : public Task
{
public:
  explicit WaitModeReadyTask(const AutowareMode & mode) : mode_(mode) {}
  TaskResult execute(Interface & interface, GateStatus & gates) override;
  std::string describe() const override;

private:
  const AutowareMode mode_;
};

class WaitModeStableTask : public Task
{
public:
  explicit WaitModeStableTask(const AutowareMode & mode) : mode_(mode) {}
  TaskResult execute(Interface & interface, GateStatus & gates) override;
  std::string describe() const override;

private:
  const AutowareMode mode_;
};

}  // namespace autoware::driving_mode_manager

#endif  // CORE__TASK_HPP_
