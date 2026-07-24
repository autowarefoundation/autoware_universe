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

#include "core/task.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::driving_mode_manager
{

namespace
{

rclcpp::Time seconds_to_time(double seconds)
{
  const auto nanoseconds = static_cast<int64_t>(seconds * 1e9);
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

class FakeInterface : public Interface
{
public:
  // This fake captures side effects of task execution without ROS node setup.
  void init(MainLogic *) override {}
  bool get_enable_debug_topics() const override { return false; }

  rclcpp::Time now() const override { return now_; }
  void set_now(double seconds) { now_ = seconds_to_time(seconds); }

  void change_trajectory_source(const TrajectorySource & source) override
  {
    changed_trajectory_source = source;
    ++trajectory_source_call_count;
  }

  void change_command_source(const CommandSource & source) override
  {
    changed_command_source = source;
    ++command_source_call_count;
  }

  void change_command_filter(const CommandFilter & filter) override
  {
    changed_command_filter = filter;
    ++command_filter_call_count;
  }

  void change_platform_mode(const PlatformMode & mode) override
  {
    changed_platform_mode = mode;
    ++platform_mode_call_count;
  }

  void publish_operation_mode(const OperationModeState &) const override {}
  void publish_mrm_state(const MrmState &) const override {}
  void publish_driving_mode_request(const ModeRequest &) const override {}
  void publish_driving_mode_info(const ModeInfo &) const override {}
  void publish_debug_flags(const DebugFlags &) const override {}
  void publish_debug_request(const RequestModes &) const override {}

  void log_info(const std::string &) const override {}
  void log_warn(const std::string &) const override {}
  void log_error(const std::string &) const override {}
  void log_debug(const std::string &) const override {}

  rclcpp::Time now_{seconds_to_time(0.0)};
  int trajectory_source_call_count{0};
  int command_source_call_count{0};
  int command_filter_call_count{0};
  int platform_mode_call_count{0};
  TrajectorySource changed_trajectory_source{0};
  CommandSource changed_command_source{0};
  CommandFilter changed_command_filter{false};
  PlatformMode changed_platform_mode{PlatformMode::kUnknown};
};

class DummyTask : public Task
{
public:
  explicit DummyTask(std::string description) : description_(std::move(description)) {}

  std::string describe() const override { return description_; }

  TaskResult execute(Interface &, GateStatus &, const DrivingModeStatus &) override
  {
    return TaskResult::kFinished;
  }

private:
  std::string description_;
};

}  // namespace

TEST(TaskListTest, queue_priority_is_platform_then_autoware_then_finalize)
{
  // TaskList always prioritizes platform tasks, then autoware, then finalize.
  TaskList tasks;

  tasks.add_finalize_tasks(std::make_unique<DummyTask>("finalize"));
  tasks.add_autoware_tasks(std::make_unique<DummyTask>("autoware"));
  tasks.add_platform_tasks(std::make_unique<DummyTask>("platform"));

  ASSERT_NE(tasks.get(), nullptr);
  EXPECT_EQ(tasks.get()->describe(), "platform");
  tasks.pop();
  ASSERT_NE(tasks.get(), nullptr);
  EXPECT_EQ(tasks.get()->describe(), "autoware");
  tasks.pop();
  ASSERT_NE(tasks.get(), nullptr);
  EXPECT_EQ(tasks.get()->describe(), "finalize");
  tasks.pop();
  EXPECT_TRUE(tasks.empty());
}

TEST(TaskListTest, interruptible_only_when_no_platform_or_autoware_tasks)
{
  // Finalize-only queue is interruptible; platform/autoware tasks are not.
  TaskList tasks;

  EXPECT_TRUE(tasks.interruptible());

  tasks.add_finalize_tasks(std::make_unique<DummyTask>("finalize"));
  EXPECT_TRUE(tasks.interruptible());

  tasks.add_autoware_tasks(std::make_unique<DummyTask>("autoware"));
  EXPECT_FALSE(tasks.interruptible());

  // Clear the autoware task added above so this next check isolates
  // the "platform task only" non-interruptible case.
  tasks.clear_autoware_tasks();
  tasks.add_platform_tasks(std::make_unique<DummyTask>("platform"));
  EXPECT_FALSE(tasks.interruptible());
}

TEST(PlatformModeTaskTest, requests_change_once_and_times_out)
{
  // First execution issues the mode-change request and starts timeout tracking.
  FakeInterface interface;
  interface.set_now(0.0);

  GateStatus gates;
  gates.status.platform_mode = PlatformMode::kManual;
  gates.expect.platform_mode = PlatformMode::kManual;

  DrivingModeStatus status({});
  PlatformModeTask task(PlatformMode::kAutoware);

  EXPECT_EQ(task.execute(interface, gates, status), TaskResult::kRunning);
  EXPECT_EQ(gates.expect.platform_mode, PlatformMode::kAutoware);
  EXPECT_EQ(interface.platform_mode_call_count, 1);
  EXPECT_EQ(interface.changed_platform_mode, PlatformMode::kAutoware);

  // Subsequent calls while waiting must not re-send the same request.
  interface.set_now(0.5);
  EXPECT_EQ(task.execute(interface, gates, status), TaskResult::kRunning);
  EXPECT_EQ(interface.platform_mode_call_count, 1);

  // Exceeding task timeout transitions to timeout result.
  interface.set_now(1.1);
  EXPECT_EQ(task.execute(interface, gates, status), TaskResult::kTimeout);
}

TEST(PlatformModeTaskTest, finishes_when_status_matches_target)
{
  // If gate status already matches target, task completes immediately.
  FakeInterface interface;
  GateStatus gates;
  gates.status.platform_mode = PlatformMode::kAutoware;
  gates.expect.platform_mode = PlatformMode::kAutoware;

  DrivingModeStatus status({});
  PlatformModeTask task(PlatformMode::kAutoware);

  EXPECT_EQ(task.execute(interface, gates, status), TaskResult::kFinished);
  EXPECT_EQ(interface.platform_mode_call_count, 0);
}

TEST(WaitModeActiveTaskTest, finishes_when_mode_is_active)
{
  // Wait task should finish immediately when status already satisfies condition.
  const AutowareMode mode{1001};
  DrivingModeStatus status({mode});
  auto * data = status.data(mode);
  ASSERT_NE(data, nullptr);
  data->active.update(seconds_to_time(0.0), true);

  FakeInterface interface;
  GateStatus gates;
  WaitModeActiveTask task(mode);

  EXPECT_EQ(task.execute(interface, gates, status), TaskResult::kFinished);
}

TEST(WaitModeStableTaskTest, times_out_when_mode_does_not_become_stable)
{
  // Without stable flag update, wait task eventually times out.
  const AutowareMode mode{1001};
  DrivingModeStatus status({mode});

  FakeInterface interface;
  interface.set_now(0.0);

  GateStatus gates;
  WaitModeStableTask task(mode);

  EXPECT_EQ(task.execute(interface, gates, status), TaskResult::kRunning);

  interface.set_now(5.1);
  EXPECT_EQ(task.execute(interface, gates, status), TaskResult::kTimeout);
}

}  // namespace autoware::driving_mode_manager
