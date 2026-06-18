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

#include "core/status.hpp"
#include "type/interface.hpp"

#include <rclcpp/time.hpp>

#include <optional>
#include <queue>
#include <string>
#include <utility>

namespace autoware::driving_mode_manager
{

enum class TaskResult {
  kFinished,
  kRunning,
  kTimeout,
};

class Task
{
public:
  virtual ~Task() = default;
  virtual std::string describe() const = 0;
  virtual TaskResult execute(
    Interface & interface, GateStatus & gates, const DrivingModeStatus & status) = 0;
};

class TaskList
{
public:
  Task * get() const;
  void pop();
  bool interruptible() const;
  bool empty() const;

  void clear_platform_tasks() { platform_ = std::queue<std::unique_ptr<Task>>(); }
  void clear_autoware_tasks() { autoware_ = std::queue<std::unique_ptr<Task>>(); }
  void clear_finalize_tasks() { finalize_ = std::queue<std::unique_ptr<Task>>(); }
  void add_platform_tasks(std::unique_ptr<Task> && task) { platform_.push(std::move(task)); }
  void add_autoware_tasks(std::unique_ptr<Task> && task) { autoware_.push(std::move(task)); }
  void add_finalize_tasks(std::unique_ptr<Task> && task) { finalize_.push(std::move(task)); }

private:
  std::queue<std::unique_ptr<Task>> platform_;
  std::queue<std::unique_ptr<Task>> autoware_;
  std::queue<std::unique_ptr<Task>> finalize_;
};

class PlatformModeTask : public Task
{
public:
  explicit PlatformModeTask(const PlatformMode & target) : target_(target) {}
  std::string describe() const override;
  TaskResult execute(
    Interface & interface, GateStatus & gates, const DrivingModeStatus & status) override;

private:
  static constexpr double timeout = 1.0;
  const PlatformMode target_;
  std::optional<rclcpp::Time> stamp_;
};

class TrajectorySourceTask : public Task
{
public:
  explicit TrajectorySourceTask(const TrajectorySource & target) : target_(target) {}
  std::string describe() const override;
  TaskResult execute(
    Interface & interface, GateStatus & gates, const DrivingModeStatus & status) override;

private:
  static constexpr double timeout = 1.0;
  const TrajectorySource target_;
  std::optional<rclcpp::Time> stamp_;
};

class CommandSourceTask : public Task
{
public:
  explicit CommandSourceTask(const CommandSource & target) : target_(target) {}
  std::string describe() const override;
  TaskResult execute(
    Interface & interface, GateStatus & gates, const DrivingModeStatus & status) override;

private:
  static constexpr double timeout = 1.0;
  const CommandSource target_;
  std::optional<rclcpp::Time> stamp_;
};

class CommandFilterTask : public Task
{
public:
  explicit CommandFilterTask(const CommandFilter & target) : target_(target) {}
  std::string describe() const override;
  TaskResult execute(
    Interface & interface, GateStatus & gates, const DrivingModeStatus & status) override;

private:
  static constexpr double timeout = 3.0;
  const CommandFilter target_;
  std::optional<rclcpp::Time> stamp_;
};

class WaitModeActiveTask : public Task
{
public:
  explicit WaitModeActiveTask(const AutowareMode & mode) : mode_(mode) {}
  std::string describe() const override;
  TaskResult execute(
    Interface & interface, GateStatus & gates, const DrivingModeStatus & status) override;

private:
  static constexpr double timeout = 1.0;
  const AutowareMode mode_;
  std::optional<rclcpp::Time> stamp_;
};

class WaitModeStableTask : public Task
{
public:
  explicit WaitModeStableTask(const AutowareMode & mode) : mode_(mode) {}
  std::string describe() const override;
  TaskResult execute(
    Interface & interface, GateStatus & gates, const DrivingModeStatus & status) override;

private:
  static constexpr double timeout = 5.0;
  const AutowareMode mode_;
  std::optional<rclcpp::Time> stamp_;
};

}  // namespace autoware::driving_mode_manager

#endif  // CORE__TASK_HPP_
