// Copyright 2025 TIER IV, Inc.
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

#include "autoware/diffusion_planner/conversion/agent_history_resampler.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::diffusion_planner::test
{

namespace
{
HistoryResamplingParams make_params()
{
  HistoryResamplingParams params;
  params.dt_sub_step_max = 0.11;
  params.yaw_rate_threshold = 0.01;
  params.max_extrapolation_time = 0.5;
  return params;
}
}  // namespace

// Constant-velocity: a straight-moving agent advances speed*dt along its heading, yaw unchanged.
TEST(AgentHistoryResamplerTest, PropagateConstantVelocityStraight)
{
  const auto params = make_params();
  const MotionState start{1.0, 2.0, 0.0};
  const auto out = propagate_motion(start, 3.0, 0.0, 0.1, params);

  EXPECT_NEAR(out.x, 1.0 + 3.0 * 0.1, 1e-9);
  EXPECT_NEAR(out.y, 2.0, 1e-9);
  EXPECT_NEAR(out.yaw, 0.0, 1e-9);
}

// A yaw rate below the threshold is ignored (degenerates to straight constant velocity).
TEST(AgentHistoryResamplerTest, PropagateBelowYawRateThresholdIsStraight)
{
  const auto params = make_params();
  const MotionState start{0.0, 0.0, 0.0};
  const auto out = propagate_motion(start, 2.0, 0.005, 0.1, params);

  EXPECT_NEAR(out.x, 0.2, 1e-9);
  EXPECT_NEAR(out.y, 0.0, 1e-9);
  EXPECT_NEAR(out.yaw, 0.0, 1e-9);
}

// Constant-turn-rate: heading advances by yaw_rate*dt over a single sub-step.
TEST(AgentHistoryResamplerTest, PropagateConstantTurnRateSingleStep)
{
  const auto params = make_params();
  const MotionState start{0.0, 0.0, 0.0};
  const double w = 0.5;
  const double dt = 0.1;  // within dt_sub_step_max -> one sub-step
  const auto out = propagate_motion(start, 1.0, w, dt, params);

  EXPECT_NEAR(out.yaw, w * dt, 1e-9);
  EXPECT_NEAR(out.x, 1.0 * std::cos(0.0) * dt, 1e-9);
  EXPECT_NEAR(out.y, 1.0 * std::sin(0.0) * dt, 1e-9);
}

// Sub-stepping curves the path: a multi-step turn ends with the expected total heading change and
// a laterally displaced position (y > 0 for a positive turn rate).
TEST(AgentHistoryResamplerTest, PropagateConstantTurnRateSubStepped)
{
  const auto params = make_params();
  const MotionState start{0.0, 0.0, 0.0};
  const double w = 1.0;
  const double dt = 0.5;  // ceil(0.5 / 0.11) = 5 sub-steps
  const auto out = propagate_motion(start, 2.0, w, dt, params);

  EXPECT_NEAR(out.yaw, w * dt, 1e-9);
  EXPECT_GT(out.x, 0.0);
  EXPECT_GT(out.y, 0.0);  // curved left
}

// A zero dt is a no-op.
TEST(AgentHistoryResamplerTest, PropagateZeroDtIsNoOp)
{
  const auto params = make_params();
  const MotionState start{5.0, -3.0, 1.2};
  const auto out = propagate_motion(start, 4.0, 0.3, 0.0, params);
  EXPECT_EQ(out.x, start.x);
  EXPECT_EQ(out.y, start.y);
  EXPECT_EQ(out.yaw, start.yaw);
}

// A negative dt integrates the motion backward: a straight agent heading +x sits behind its start.
TEST(AgentHistoryResamplerTest, PropagateNegativeDtGoesBackward)
{
  const auto params = make_params();
  const MotionState start{1.0, 2.0, 0.0};
  const auto out = propagate_motion(start, 3.0, 0.0, -0.2, params);

  EXPECT_NEAR(out.x, 1.0 - 3.0 * 0.2, 1e-9);
  EXPECT_NEAR(out.y, 2.0, 1e-9);
  EXPECT_NEAR(out.yaw, 0.0, 1e-9);
}

// dt is clamped to max_extrapolation_time.
TEST(AgentHistoryResamplerTest, PropagateClampsDt)
{
  const auto params = make_params();  // max_extrapolation_time = 0.5
  const MotionState start{0.0, 0.0, 0.0};
  const auto out = propagate_motion(start, 1.0, 0.0, 2.0, params);

  EXPECT_NEAR(out.x, 0.5, 1e-9);  // 1.0 m/s * 0.5 s clamp
}

// Yaw interpolation takes the shortest arc across the +/-pi discontinuity.
TEST(AgentHistoryResamplerTest, InterpolateYawShortestArcAcrossPi)
{
  const double y = interpolate_yaw(3.0, -3.0, 0.5);
  // 3.0 and -3.0 are 0.283 rad apart the short way; the midpoint sits near +/-pi, not 0.
  EXPECT_NEAR(std::abs(y), M_PI, 1e-6);
}

TEST(AgentHistoryResamplerTest, InterpolateYawMidpoint)
{
  EXPECT_NEAR(interpolate_yaw(0.0, M_PI_2, 0.5), M_PI_4, 1e-9);
  EXPECT_NEAR(interpolate_yaw(0.0, M_PI_2, 0.0), 0.0, 1e-9);
  EXPECT_NEAR(interpolate_yaw(0.0, M_PI_2, 1.0), M_PI_2, 1e-9);
}

}  // namespace autoware::diffusion_planner::test
