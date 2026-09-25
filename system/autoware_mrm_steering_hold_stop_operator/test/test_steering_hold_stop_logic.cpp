// Copyright 2026 TIER IV, Inc.
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

#include "steering_hold_stop_logic.hpp"

#include <gmock/gmock.h>

namespace autoware::mrm_steering_hold_stop_operator
{

namespace
{

constexpr double kDt = 0.03;

LogicParams defaultParams()
{
  LogicParams params;
  params.input_freshness_timeout = 0.5;
  params.low_speed_threshold = 1.0;
  return params;
}

MeasuredInputs freshInputs(
  const double steering = 0.1, const double velocity = 10.0, const double acceleration = -0.5)
{
  MeasuredInputs inputs;
  inputs.steering_age = 0.01;
  inputs.odom_age = 0.01;
  inputs.accel_age = 0.01;
  inputs.steering_tire_angle = steering;
  inputs.longitudinal_velocity = velocity;
  inputs.longitudinal_acceleration = acceleration;
  return inputs;
}

TriggerState triggerOff()
{
  return TriggerState{false, 0.0, 0.0};
}

TriggerState triggerOn(const double target_acceleration = -3.0, const double target_jerk = -5.0)
{
  return TriggerState{true, target_acceleration, target_jerk};
}

}  // namespace

TEST(SteeringHoldStopLogic, MirrorOutputsMeasuredState)
{
  SteeringHoldStopLogic logic(defaultParams());
  const auto result = logic.update(triggerOff(), freshInputs(0.2, 8.0, -0.3), kDt);

  EXPECT_EQ(result.state, State::kMirror);
  ASSERT_TRUE(result.command.has_value());
  EXPECT_DOUBLE_EQ(result.command->steering_tire_angle, 0.2);
  EXPECT_DOUBLE_EQ(result.command->velocity, 8.0);
  EXPECT_DOUBLE_EQ(result.command->acceleration, -0.3);
  EXPECT_DOUBLE_EQ(result.command->jerk, 0.0);
}

TEST(SteeringHoldStopLogic, MirrorClampsPositiveAcceleration)
{
  SteeringHoldStopLogic logic(defaultParams());
  const auto result = logic.update(triggerOff(), freshInputs(0.0, 8.0, 0.8), kDt);

  ASSERT_TRUE(result.command.has_value());
  EXPECT_DOUBLE_EQ(result.command->acceleration, 0.0);
}

TEST(SteeringHoldStopLogic, MirrorAtLowSpeedIgnoresStaleAcceleration)
{
  SteeringHoldStopLogic logic(defaultParams());
  auto inputs = freshInputs(0.05, 0.5, -0.2);
  inputs.accel_age = std::nullopt;  // acceleration never received

  const auto result = logic.update(triggerOff(), inputs, kDt);

  EXPECT_EQ(result.state, State::kMirror);
  ASSERT_TRUE(result.command.has_value());
  EXPECT_DOUBLE_EQ(result.command->acceleration, 0.0);  // not used at low speed
}

TEST(SteeringHoldStopLogic, WaitingInputWhenRequiredInputStaleWhileTriggerOff)
{
  SteeringHoldStopLogic logic(defaultParams());

  auto stale_steering = freshInputs();
  stale_steering.steering_age = 1.0;
  auto result = logic.update(triggerOff(), stale_steering, kDt);
  EXPECT_EQ(result.state, State::kWaitingInput);
  EXPECT_FALSE(result.command.has_value());

  auto stale_odom = freshInputs();
  stale_odom.odom_age = std::nullopt;
  result = logic.update(triggerOff(), stale_odom, kDt);
  EXPECT_EQ(result.state, State::kWaitingInput);
  EXPECT_FALSE(result.command.has_value());

  auto stale_accel = freshInputs(0.1, 10.0, -0.5);  // high speed: acceleration required
  stale_accel.accel_age = 1.0;
  result = logic.update(triggerOff(), stale_accel, kDt);
  EXPECT_EQ(result.state, State::kWaitingInput);
  EXPECT_FALSE(result.command.has_value());
}

TEST(SteeringHoldStopLogic, WaitingInputRecoversToMirror)
{
  SteeringHoldStopLogic logic(defaultParams());

  auto stale = freshInputs();
  stale.steering_age = 1.0;
  EXPECT_EQ(logic.update(triggerOff(), stale, kDt).state, State::kWaitingInput);

  const auto result = logic.update(triggerOff(), freshInputs(), kDt);
  EXPECT_EQ(result.state, State::kMirror);
  EXPECT_TRUE(result.command.has_value());
}

TEST(SteeringHoldStopLogic, TriggerOnInitializesFromMeasurements)
{
  SteeringHoldStopLogic logic(defaultParams());
  logic.update(triggerOff(), freshInputs(), kDt);

  const auto init = logic.update(triggerOn(-3.0, -5.0), freshInputs(0.15, 10.0, -0.5), kDt);
  EXPECT_EQ(init.state, State::kDecelerating);
  ASSERT_TRUE(init.command.has_value());
  EXPECT_DOUBLE_EQ(init.command->steering_tire_angle, 0.15);
  EXPECT_DOUBLE_EQ(init.command->velocity, 10.0);
  EXPECT_DOUBLE_EQ(init.command->acceleration, -0.5);
  EXPECT_DOUBLE_EQ(init.command->jerk, -5.0);

  // Steering stays held even if the measured steering changes afterwards.
  const auto step = logic.update(triggerOn(-3.0, -5.0), freshInputs(0.5, 9.0, -1.0), 0.1);
  ASSERT_TRUE(step.command.has_value());
  EXPECT_DOUBLE_EQ(step.command->steering_tire_angle, 0.15);
  EXPECT_NEAR(step.command->velocity, 10.0 - 0.5 * 0.1, 1e-9);
  EXPECT_NEAR(step.command->acceleration, -0.5 - 5.0 * 0.1, 1e-9);
  EXPECT_DOUBLE_EQ(step.command->jerk, -5.0);
}

TEST(SteeringHoldStopLogic, InitialAccelerationClampedToZeroWhenMeasuredPositive)
{
  SteeringHoldStopLogic logic(defaultParams());
  const auto init = logic.update(triggerOn(), freshInputs(0.0, 10.0, 1.2), kDt);

  ASSERT_TRUE(init.command.has_value());
  EXPECT_DOUBLE_EQ(init.command->acceleration, 0.0);
}

TEST(SteeringHoldStopLogic, InitialAccelerationClampedToTarget)
{
  SteeringHoldStopLogic logic(defaultParams());
  const auto init = logic.update(triggerOn(-3.0, -5.0), freshInputs(0.0, 10.0, -6.0), kDt);

  ASSERT_TRUE(init.command.has_value());
  EXPECT_DOUBLE_EQ(init.command->acceleration, -3.0);
  EXPECT_DOUBLE_EQ(init.command->jerk, 0.0);  // already at target
}

TEST(SteeringHoldStopLogic, LowSpeedStartsAtTargetAccelerationWithoutAccelInput)
{
  SteeringHoldStopLogic logic(defaultParams());
  auto inputs = freshInputs(0.05, 0.5, 0.0);
  inputs.accel_age = std::nullopt;  // acceleration not required at low speed

  const auto init = logic.update(triggerOn(-3.0, -5.0), inputs, kDt);
  EXPECT_EQ(init.state, State::kDecelerating);
  ASSERT_TRUE(init.command.has_value());
  EXPECT_DOUBLE_EQ(init.command->acceleration, -3.0);
  EXPECT_DOUBLE_EQ(init.command->jerk, 0.0);
}

TEST(SteeringHoldStopLogic, TriggerOnWithStaleInputWaitsThenStarts)
{
  SteeringHoldStopLogic logic(defaultParams());

  auto stale = freshInputs();
  stale.odom_age = 1.0;
  const auto waiting = logic.update(triggerOn(), stale, kDt);
  EXPECT_EQ(waiting.state, State::kWaitingInput);
  EXPECT_FALSE(waiting.command.has_value());

  const auto init = logic.update(triggerOn(), freshInputs(0.1, 5.0, -0.2), kDt);
  EXPECT_EQ(init.state, State::kDecelerating);
  ASSERT_TRUE(init.command.has_value());
  EXPECT_DOUBLE_EQ(init.command->velocity, 5.0);
}

TEST(SteeringHoldStopLogic, DecelerationContinuesWithStaleInputs)
{
  SteeringHoldStopLogic logic(defaultParams());
  logic.update(triggerOn(), freshInputs(), kDt);

  MeasuredInputs stale;  // everything missing
  const auto result = logic.update(triggerOn(), stale, kDt);
  EXPECT_EQ(result.state, State::kDecelerating);
  EXPECT_TRUE(result.command.has_value());
}

TEST(SteeringHoldStopLogic, VelocityReachesZeroAndOutputContinues)
{
  SteeringHoldStopLogic logic(defaultParams());
  logic.update(triggerOn(-3.0, -5.0), freshInputs(0.1, 2.0, -0.5), kDt);

  UpdateResult last;
  for (int i = 0; i < 1000; ++i) {
    last = logic.update(triggerOn(-3.0, -5.0), MeasuredInputs{}, kDt);
  }
  ASSERT_TRUE(last.command.has_value());
  EXPECT_DOUBLE_EQ(last.command->velocity, 0.0);
  EXPECT_DOUBLE_EQ(last.command->acceleration, -3.0);
  EXPECT_DOUBLE_EQ(last.command->jerk, 0.0);
  EXPECT_DOUBLE_EQ(last.command->steering_tire_angle, 0.1);
}

TEST(SteeringHoldStopLogic, TriggerOffResetsToMirrorAndLatchesAgainNextTime)
{
  SteeringHoldStopLogic logic(defaultParams());
  logic.update(triggerOn(), freshInputs(0.1, 10.0, -0.5), kDt);

  const auto mirror = logic.update(triggerOff(), freshInputs(0.3, 9.0, -0.2), kDt);
  EXPECT_EQ(mirror.state, State::kMirror);
  ASSERT_TRUE(mirror.command.has_value());
  EXPECT_DOUBLE_EQ(mirror.command->steering_tire_angle, 0.3);

  // A new trigger latches the current measurement, not the previous hold value.
  const auto init = logic.update(triggerOn(), freshInputs(0.3, 9.0, -0.2), kDt);
  ASSERT_TRUE(init.command.has_value());
  EXPECT_DOUBLE_EQ(init.command->steering_tire_angle, 0.3);
  EXPECT_DOUBLE_EQ(init.command->velocity, 9.0);
}

TEST(SteeringHoldStopLogic, RetriggerUpdatesTargetsKeepingHeldSteering)
{
  SteeringHoldStopLogic logic(defaultParams());
  logic.update(triggerOn(-3.0, -5.0), freshInputs(0.1, 10.0, -2.9), kDt);

  // A repeated ON message with different targets: steering stays held, the new
  // target acceleration bounds the ramp.
  const auto step = logic.update(triggerOn(-1.0, -5.0), freshInputs(0.5, 9.0, 0.0), 1.0);
  ASSERT_TRUE(step.command.has_value());
  EXPECT_DOUBLE_EQ(step.command->steering_tire_angle, 0.1);
  EXPECT_DOUBLE_EQ(step.command->acceleration, -1.0);
}

TEST(SteeringHoldStopLogic, DefensiveClampAgainstNonNegativeTargets)
{
  SteeringHoldStopLogic logic(defaultParams());

  // Positive target jerk would never converge: fall back to a stepwise
  // transition to the target acceleration.
  const auto init = logic.update(triggerOn(-3.0, 1.0), freshInputs(0.0, 10.0, -0.5), kDt);
  ASSERT_TRUE(init.command.has_value());
  EXPECT_DOUBLE_EQ(init.command->acceleration, -3.0);

  // Positive target acceleration is clamped so the command never accelerates.
  SteeringHoldStopLogic logic2(defaultParams());
  const auto init2 = logic2.update(triggerOn(2.0, -5.0), freshInputs(0.0, 10.0, 0.5), kDt);
  ASSERT_TRUE(init2.command.has_value());
  EXPECT_LE(init2.command->acceleration, 0.0);
}

}  // namespace autoware::mrm_steering_hold_stop_operator
