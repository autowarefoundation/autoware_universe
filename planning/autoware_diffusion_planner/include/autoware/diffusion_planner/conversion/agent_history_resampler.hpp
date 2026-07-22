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

// cspell:ignore CTRV dedup

#ifndef AUTOWARE__DIFFUSION_PLANNER__CONVERSION__AGENT_HISTORY_RESAMPLER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__CONVERSION__AGENT_HISTORY_RESAMPLER_HPP_

#include "autoware/diffusion_planner/conversion/agent.hpp"

#include <rclcpp/time.hpp>

#include <optional>

namespace autoware::diffusion_planner
{

/**
 * @brief Parameters for re-timing neighbor histories onto the model's assumed grid.
 *
 * The input `neighbor_agents_past` carries no timestamp/dt/mask channel, so the network assumes a
 * uniform 0.1s history grid. Jittered, latent per-UUID histories are resampled onto an
 * odometry-anchored constant grid before being fed in; these parameters govern that resampling.
 */
struct HistoryResamplingParams
{
  // When false, histories are fed as buffered with no grid re-timing, retention, or dedup.
  bool enable{true};

  // Maximum extrapolation horizon [s].
  double max_extrapolation_time{0.5};
};

/**
 * @brief A minimal planar motion state for the CTRV core.
 */
struct MotionState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/**
 * @brief Propagate a planar state by dt with a constant-turn-rate (CTRV) model. A
 * positive dt extrapolates forward and is clamped to max_extrapolation_time; a negative dt
 * extrapolates backward (the caller bounds it to max_extrapolation_time); a zero dt is a no-op.
 * Sub-stepped in either direction with a fixed sub-step to bound forward-Euler error.
 */
MotionState propagate_motion(
  const MotionState & state, double speed, double yaw_rate, double dt,
  const HistoryResamplingParams & params);

/**
 * @brief Interpolate a heading along the shortest arc so it never sweeps the ±pi discontinuity.
 */
double interpolate_yaw(double yaw_a, double yaw_b, double ratio);

/**
 * @brief Re-time one history onto the 0.1s grid anchored at frame_time (oldest first, newest at
 * frame_time): interpolate within the observations, extrapolate (clamped) past the newest one,
 * and freeze slots beyond the backward horizon at the oldest observation. Returns std::nullopt
 * for an empty history.
 */
std::optional<AgentHistory> resample_history(
  const AgentHistory & history, const rclcpp::Time & frame_time,
  const HistoryResamplingParams & params);

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__CONVERSION__AGENT_HISTORY_RESAMPLER_HPP_
