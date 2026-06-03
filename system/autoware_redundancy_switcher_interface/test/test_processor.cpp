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

#include <gtest/gtest.h>

#include <vector>

namespace autoware::redundancy_switcher
{

template <typename T>
const T * find_effect(const std::vector<OutputCommand> & commands)
{
  for (const auto & e : commands) {
    if (const auto * p = std::get_if<T>(&e)) return p;
  }
  return nullptr;
}

template <typename T>
bool has_effect(const std::vector<OutputCommand> & commands)
{
  return find_effect<T>(commands) != nullptr;
}

class ProcessorTest : public ::testing::Test
{
protected:
  Processor processor_;

  void set_ready()
  {
    processor_.handle(
      InputEvent{SetAutowareReadyEvent{Annotated<AutowareReady>{AutowareReady::True, "test"}}});
  }
  void set_stopped()
  {
    processor_.handle(
      InputEvent{
        SetVelocityStatusEvent{Annotated<VelocityStatus>{VelocityStatus::Stopped, "test"}}});
  }
  void set_auto_control()
  {
    processor_.handle(
      InputEvent{SetControlModeEvent{Annotated<ControlMode>{ControlMode::Auto, "test"}}});
  }
  void set_stable()
  {
    processor_.handle(
      InputEvent{SetSwitcherSignalsEvent{
        Annotated<SwitcherSignals>{SwitcherSignals{true, false, false}, "stable"}}});
  }
  void set_self_interrupted()
  {
    processor_.handle(
      InputEvent{SetSwitcherSignalsEvent{
        Annotated<SwitcherSignals>{SwitcherSignals{false, true, false}, "self_interrupted"}}});
  }
  void set_faulted()
  {
    processor_.handle(
      InputEvent{SetSwitcherSignalsEvent{
        Annotated<SwitcherSignals>{SwitcherSignals{false, false, true}, "faulted"}}});
  }
};

TEST_F(ProcessorTest, InitialState_IsNotReady)
{
  const auto snap = processor_.snapshot();
  EXPECT_FALSE(snap.switcher.has_value());
  EXPECT_FALSE(snap.autoware_ready.has_value());
  EXPECT_FALSE(snap.velocity_status.has_value());
  EXPECT_FALSE(snap.control_mode.has_value());
}

TEST_F(ProcessorTest, SetAutowareReady_TriggersDiagOnChange)
{
  const auto commands = processor_.handle(
    InputEvent{SetAutowareReadyEvent{Annotated<AutowareReady>{AutowareReady::True, "test"}}});
  EXPECT_TRUE(has_effect<UpdateStatusDiagCommand>(commands));
}

TEST_F(ProcessorTest, SetAutowareReady_DoesNotTriggerIfNoChange)
{
  processor_.handle(
    InputEvent{SetAutowareReadyEvent{Annotated<AutowareReady>{AutowareReady::False, "test"}}});
  const auto commands = processor_.handle(
    InputEvent{SetAutowareReadyEvent{Annotated<AutowareReady>{AutowareReady::False, "test"}}});
  EXPECT_FALSE(has_effect<UpdateStatusDiagCommand>(commands));
}

TEST_F(ProcessorTest, SetVelocityStatus_TriggersDiagOnChange)
{
  const auto commands = processor_.handle(
    InputEvent{SetVelocityStatusEvent{Annotated<VelocityStatus>{VelocityStatus::Stopped, "test"}}});
  EXPECT_TRUE(has_effect<UpdateStatusDiagCommand>(commands));
}

TEST_F(ProcessorTest, SetVelocityStatus_DoesNotTriggerDiagIfNoChange)
{
  processor_.handle(
    InputEvent{SetVelocityStatusEvent{Annotated<VelocityStatus>{VelocityStatus::Stopped, "test"}}});
  const auto commands = processor_.handle(
    InputEvent{SetVelocityStatusEvent{Annotated<VelocityStatus>{VelocityStatus::Stopped, "test"}}});
  EXPECT_FALSE(has_effect<UpdateStatusDiagCommand>(commands));
}

TEST_F(ProcessorTest, SetControlMode_TriggersDiagOnChange)
{
  const auto commands = processor_.handle(
    InputEvent{SetControlModeEvent{Annotated<ControlMode>{ControlMode::Auto, "test"}}});
  EXPECT_TRUE(has_effect<UpdateStatusDiagCommand>(commands));
}

TEST_F(ProcessorTest, SetControlMode_DoesNotTriggerDiagIfNoChange)
{
  processor_.handle(
    InputEvent{SetControlModeEvent{Annotated<ControlMode>{ControlMode::Auto, "test"}}});
  const auto commands = processor_.handle(
    InputEvent{SetControlModeEvent{Annotated<ControlMode>{ControlMode::Auto, "test"}}});
  EXPECT_FALSE(has_effect<UpdateStatusDiagCommand>(commands));
}

TEST_F(ProcessorTest, SetSwitcherSignals_Stable_TriggersDiagAndLog)
{
  const auto commands = processor_.handle(
    InputEvent{
      SetSwitcherSignalsEvent{Annotated<SwitcherSignals>{{true, false, false}, "stable"}}});
  EXPECT_TRUE(has_effect<LogCommand>(commands));
  EXPECT_TRUE(has_effect<UpdateStatusDiagCommand>(commands));
}

TEST_F(ProcessorTest, SetSwitcherSignals_Transitional_TriggersDiag)
{
  const SwitcherSignals transitional{false, false, false};
  const auto commands = processor_.handle(
    InputEvent{SetSwitcherSignalsEvent{Annotated<SwitcherSignals>{transitional, "transitional"}}});
  EXPECT_TRUE(has_effect<UpdateStatusDiagCommand>(commands));
}

TEST_F(ProcessorTest, SwitcherDataTimeout_SetsFaultedState)
{
  processor_.handle(
    InputEvent{SetSwitcherSignalsEvent{
      Annotated<SwitcherSignals>{SwitcherSignals{true, false, false}, "before timeout"}}});
  ASSERT_TRUE(processor_.snapshot().switcher.has_value());

  const auto commands = processor_.handle(
    InputEvent{SetSwitcherSignalsEvent{
      Annotated<SwitcherSignals>{{false, false, true}, "Switcher data timeout"}}});
  ASSERT_TRUE(processor_.snapshot().switcher.has_value());
  EXPECT_TRUE(processor_.snapshot().switcher->value.is_faulted);
  EXPECT_FALSE(processor_.snapshot().switcher->value.is_stable);
  EXPECT_FALSE(processor_.snapshot().switcher->value.is_self_interrupted);
  EXPECT_TRUE(has_effect<LogCommand>(commands));
  EXPECT_TRUE(has_effect<UpdateStatusDiagCommand>(commands));
}

TEST_F(ProcessorTest, SelfInterruption_Rejected_WhenAutowareNotReady)
{
  const auto commands =
    processor_.handle(InputEvent{SelfInterruptionEvent{Annotated<std::monostate>{{}, "test"}}});
  EXPECT_FALSE(has_effect<SelfInterruptionCommand>(commands));
  EXPECT_TRUE(has_effect<LogCommand>(commands));
}

TEST_F(ProcessorTest, SelfInterruption_Rejected_WhenNotInAutoControl)
{
  set_ready();
  const auto commands =
    processor_.handle(InputEvent{SelfInterruptionEvent{Annotated<std::monostate>{{}, "test"}}});
  EXPECT_FALSE(has_effect<SelfInterruptionCommand>(commands));
}

TEST_F(ProcessorTest, SelfInterruption_Accepted_WhenStable)
{
  set_ready();
  set_auto_control();
  set_stable();
  const auto commands =
    processor_.handle(InputEvent{SelfInterruptionEvent{Annotated<std::monostate>{{}, "test"}}});
  EXPECT_TRUE(has_effect<SelfInterruptionCommand>(commands));
}

TEST_F(ProcessorTest, SelfInterruption_Rejected_WhenSelfInterrupted)
{
  set_ready();
  set_auto_control();
  set_self_interrupted();
  const auto commands =
    processor_.handle(InputEvent{SelfInterruptionEvent{Annotated<std::monostate>{{}, "test"}}});
  EXPECT_FALSE(has_effect<SelfInterruptionCommand>(commands));
}

TEST_F(ProcessorTest, SelfInterruption_Rejected_WhenFaulted)
{
  set_ready();
  set_auto_control();
  set_faulted();
  const auto commands =
    processor_.handle(InputEvent{SelfInterruptionEvent{Annotated<std::monostate>{{}, "test"}}});
  EXPECT_FALSE(has_effect<SelfInterruptionCommand>(commands));
}

TEST_F(ProcessorTest, SelfInterruption_Rejected_WhenTransitional)
{
  set_ready();
  set_auto_control();
  processor_.handle(
    InputEvent{
      SetSwitcherSignalsEvent{Annotated<SwitcherSignals>{{false, false, false}, "transitional"}}});
  const auto commands =
    processor_.handle(InputEvent{SelfInterruptionEvent{Annotated<std::monostate>{{}, "test"}}});
  EXPECT_FALSE(has_effect<SelfInterruptionCommand>(commands));
}

TEST_F(ProcessorTest, Reset_Rejected_WhenVehicleNotStopped)
{
  processor_.handle(
    InputEvent{SetVelocityStatusEvent{Annotated<VelocityStatus>{VelocityStatus::Moving, "test"}}});
  const auto commands =
    processor_.handle(InputEvent{ResetEvent{Annotated<std::monostate>{{}, "test"}}});
  EXPECT_FALSE(has_effect<ResetCommand>(commands));
  EXPECT_TRUE(has_effect<LogCommand>(commands));
  const auto * result = find_effect<ResetResultCommand>(commands);
  ASSERT_NE(result, nullptr);
  EXPECT_FALSE(result->accepted);
  EXPECT_EQ(result->reason, ResetRejectedReason::Ignored);
}

TEST_F(ProcessorTest, Reset_Accepted_WhenStopped_AndAutowareNotReady)
{
  set_stopped();
  const auto commands =
    processor_.handle(InputEvent{ResetEvent{Annotated<std::monostate>{{}, "test"}}});
  EXPECT_TRUE(has_effect<ResetCommand>(commands));
  const auto * result = find_effect<ResetResultCommand>(commands);
  ASSERT_NE(result, nullptr);
  EXPECT_TRUE(result->accepted);
}

TEST_F(ProcessorTest, Reset_Accepted_WhenSelfInterrupted)
{
  set_stopped();
  set_ready();
  set_self_interrupted();
  const auto commands =
    processor_.handle(InputEvent{ResetEvent{Annotated<std::monostate>{{}, "test"}}});
  EXPECT_TRUE(has_effect<ResetCommand>(commands));
  const auto * result = find_effect<ResetResultCommand>(commands);
  ASSERT_NE(result, nullptr);
  EXPECT_TRUE(result->accepted);
}

TEST_F(ProcessorTest, Reset_Ignored_WhenStable)
{
  set_stopped();
  set_ready();
  set_stable();
  const auto commands =
    processor_.handle(InputEvent{ResetEvent{Annotated<std::monostate>{{}, "test"}}});
  EXPECT_FALSE(has_effect<ResetCommand>(commands));
  EXPECT_TRUE(has_effect<LogCommand>(commands));
  const auto * result = find_effect<ResetResultCommand>(commands);
  ASSERT_NE(result, nullptr);
  EXPECT_FALSE(result->accepted);
  EXPECT_EQ(result->reason, ResetRejectedReason::NotNecessary);
}

TEST_F(ProcessorTest, Reset_Rejected_WhenFaulted)
{
  set_stopped();
  set_ready();
  set_faulted();
  const auto commands =
    processor_.handle(InputEvent{ResetEvent{Annotated<std::monostate>{{}, "test"}}});
  EXPECT_FALSE(has_effect<ResetCommand>(commands));
  const auto * log = find_effect<LogCommand>(commands);
  ASSERT_NE(log, nullptr);
  EXPECT_EQ(log->level, LogLevel::Error);
  const auto * result = find_effect<ResetResultCommand>(commands);
  ASSERT_NE(result, nullptr);
  EXPECT_FALSE(result->accepted);
  EXPECT_EQ(result->reason, ResetRejectedReason::Error);
}

TEST_F(ProcessorTest, Reset_AlwaysContainsResetResultCommand)
{
  const auto commands =
    processor_.handle(InputEvent{ResetEvent{Annotated<std::monostate>{{}, "test"}}});
  EXPECT_TRUE(has_effect<ResetResultCommand>(commands));
}

TEST_F(ProcessorTest, SetSwitcherSignals_SelfInterrupted_EmitsEmptyActiveControlUnit)
{
  // Switching to self_interrupted must immediately clear active control unit
  const auto commands = processor_.handle(
    InputEvent{SetSwitcherSignalsEvent{
      Annotated<SwitcherSignals>{{false, true, false}, "self_interrupted"}}});
  const auto * cmd = find_effect<UpdateActiveControlUnitCommand>(commands);
  ASSERT_NE(cmd, nullptr);
  EXPECT_TRUE(cmd->value.unit_ids.empty());
}

TEST_F(ProcessorTest, SetSwitcherSignals_Faulted_EmitsEmptyActiveControlUnit)
{
  const auto commands = processor_.handle(
    InputEvent{
      SetSwitcherSignalsEvent{Annotated<SwitcherSignals>{{false, false, true}, "faulted"}}});
  const auto * cmd = find_effect<UpdateActiveControlUnitCommand>(commands);
  ASSERT_NE(cmd, nullptr);
  EXPECT_TRUE(cmd->value.unit_ids.empty());
}

TEST_F(ProcessorTest, SetActiveControlUnit_PassthroughWhenStable)
{
  // When stable, the submitted unit_ids must be forwarded unchanged
  set_stable();
  const ActiveControlUnit expected{{0, 2}};
  const auto commands = processor_.handle(
    InputEvent{SetActiveControlUnitEvent{Annotated<ActiveControlUnit>{expected, "test"}}});
  const auto * cmd = find_effect<UpdateActiveControlUnitCommand>(commands);
  ASSERT_NE(cmd, nullptr);
  EXPECT_EQ(cmd->value.unit_ids, expected.unit_ids);
}

TEST_F(ProcessorTest, SetActiveControlUnit_ForcedEmptyWhenSelfInterrupted)
{
  // Even if SetActiveControlUnitEvent carries non-empty IDs, they must be suppressed
  set_self_interrupted();
  const ActiveControlUnit non_empty{{0}};
  const auto commands = processor_.handle(
    InputEvent{SetActiveControlUnitEvent{Annotated<ActiveControlUnit>{non_empty, "test"}}});
  const auto * cmd = find_effect<UpdateActiveControlUnitCommand>(commands);
  ASSERT_NE(cmd, nullptr);
  EXPECT_TRUE(cmd->value.unit_ids.empty());
}

TEST_F(ProcessorTest, SetActiveControlUnit_ForcedEmptyWhenFaulted)
{
  set_faulted();
  const ActiveControlUnit non_empty{{0}};
  const auto commands = processor_.handle(
    InputEvent{SetActiveControlUnitEvent{Annotated<ActiveControlUnit>{non_empty, "test"}}});
  const auto * cmd = find_effect<UpdateActiveControlUnitCommand>(commands);
  ASSERT_NE(cmd, nullptr);
  EXPECT_TRUE(cmd->value.unit_ids.empty());
}

TEST_F(ProcessorTest, SameInputSequence_ProducesSameOutput)
{
  auto run_sequence = [](Processor & p) {
    p.handle(
      InputEvent{SetAutowareReadyEvent{Annotated<AutowareReady>{AutowareReady::True, "test"}}});
    p.handle(
      InputEvent{
        SetVelocityStatusEvent{Annotated<VelocityStatus>{VelocityStatus::Stopped, "test"}}});
    p.handle(InputEvent{SetControlModeEvent{Annotated<ControlMode>{ControlMode::Auto, "test"}}});
    p.handle(
      InputEvent{SetSwitcherSignalsEvent{
        Annotated<SwitcherSignals>{SwitcherSignals{true, false, false}, "stable"}}});
    return p.snapshot();
  };

  Processor p1, p2;
  const auto snap1 = run_sequence(p1);
  const auto snap2 = run_sequence(p2);

  EXPECT_EQ(snap1.autoware_ready->value, snap2.autoware_ready->value);
  EXPECT_EQ(snap1.velocity_status->value, snap2.velocity_status->value);
  EXPECT_EQ(snap1.control_mode->value, snap2.control_mode->value);
  ASSERT_TRUE(snap1.switcher.has_value());
  ASSERT_TRUE(snap2.switcher.has_value());
  EXPECT_EQ(snap1.switcher->value.is_stable, snap2.switcher->value.is_stable);
  EXPECT_EQ(snap1.switcher->value.is_self_interrupted, snap2.switcher->value.is_self_interrupted);
  EXPECT_EQ(snap1.switcher->value.is_faulted, snap2.switcher->value.is_faulted);
}

}  // namespace autoware::redundancy_switcher
