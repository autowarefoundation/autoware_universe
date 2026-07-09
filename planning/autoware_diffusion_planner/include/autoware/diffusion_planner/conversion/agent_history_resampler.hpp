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

  // Maximum CTRV integration sub-step [s]; a larger extrapolation dt is split to bound Euler error.
  double dt_sub_step_max{0.11};

  // Below this |yaw rate| [rad/s] an agent is propagated straight (constant velocity).
  double yaw_rate_threshold{0.01};

  // Maximum forward extrapolation horizon [s] from the newest observation to the frame time.
  double max_extrapolation_time{0.5};

  // A backward-in-time yaw jump larger than this [rad] is treated as a tracker heading flip.
  double flip_yaw_threshold{2.35619449};  // 135 deg

  // Extra grace period [s] beyond the history window before a disappeared agent is pruned.
  double prune_grace{0.5};
};

/**
 * @brief A minimal planar motion state for the CV / CTRV core.
 */
struct MotionState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/**
 * @brief Propagate a planar state forward by dt (CV when |yaw_rate| < yaw_rate_threshold, else
 * CTRV). dt is clamped to max_extrapolation_time and sub-stepped; a non-positive dt is a no-op.
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
 * frame_time): interpolate within the observations, extrapolate live agents past the newest one,
 * freeze disappeared ones. Returns std::nullopt for an empty history.
 */
std::optional<AgentHistory> resample_history(
  const AgentHistory & history, const rclcpp::Time & frame_time,
  const HistoryResamplingParams & params);

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__CONVERSION__AGENT_HISTORY_RESAMPLER_HPP_
