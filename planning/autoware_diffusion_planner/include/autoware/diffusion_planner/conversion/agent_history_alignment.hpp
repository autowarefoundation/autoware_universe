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

#ifndef AUTOWARE__DIFFUSION_PLANNER__CONVERSION__AGENT_HISTORY_ALIGNMENT_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__CONVERSION__AGENT_HISTORY_ALIGNMENT_HPP_

namespace autoware::diffusion_planner
{

/**
 * @brief Parameters controlling neighbor-agent history time-alignment.
 *
 * The diffusion model input `neighbor_agents_past` carries no per-frame timestamp, dt, or mask
 * channel, so the network assumes a uniform 0.1s history grid. Tracked objects arrive latent and
 * jittered, so their per-UUID histories are re-timed onto an odometry-anchored constant grid before
 * being fed to the model. These parameters govern that re-timing.
 */
struct HistoryAlignmentParams
{
  // When false, the legacy behavior is used: histories are fed to the model as buffered, without
  // any grid re-timing, disappeared-agent retention, or duplicate-message dedup.
  bool enable{true};

  // Maximum integration sub-step [s] for the constant-turn-rate motion model. Extrapolation over a
  // dt larger than this is split into equal sub-steps to bound the forward-Euler linearization
  // error (mirrors the tracker's dt_max sub-stepping).
  double dt_sub_step_max{0.11};

  // Below this absolute yaw rate [rad/s] an agent is propagated with a straight constant-velocity
  // model instead of a constant-turn-rate model.
  double yaw_rate_threshold{0.01};

  // Maximum forward extrapolation horizon [s] from the newest observation to the frame time. dt is
  // clamped to this value so a slightly stale "live" agent does not shoot far ahead.
  double max_extrapolation_time{0.5};

  // A backward-in-time yaw jump larger than this [rad] (relative to the newer, more trustworthy
  // neighbor) is treated as a tracker heading flip and corrected by rotating the heading by pi.
  double flip_yaw_threshold{2.35619449};  // 135 deg

  // Extra grace period [s] beyond the history window before a disappeared agent is pruned.
  double prune_grace{0.5};
};

/**
 * @brief A minimal planar motion state used by the constant-velocity / constant-turn-rate core.
 */
struct MotionState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/**
 * @brief Propagate a planar motion state forward by dt using a CV or CTRV model.
 *
 * Mirrors the multi-object tracker's forward-Euler prediction step: straight constant-velocity
 * when |yaw_rate| < yaw_rate_threshold, otherwise constant-turn-rate. A dt larger than
 * dt_sub_step_max is integrated in equal sub-steps to bound the linearization error. dt is clamped
 * to max_extrapolation_time; a non-positive dt is a no-op. The returned yaw is normalized.
 *
 * @param state Initial state (position and heading in the map frame).
 * @param speed Longitudinal speed [m/s] (constant over the horizon).
 * @param yaw_rate Yaw rate [rad/s] (constant over the horizon).
 * @param dt Time to propagate [s].
 * @param params Alignment parameters providing the sub-step, yaw-rate, and horizon limits.
 * @return Propagated state.
 */
MotionState propagate_motion(
  const MotionState & state, double speed, double yaw_rate, double dt,
  const HistoryAlignmentParams & params);

/**
 * @brief Interpolate a heading along the shortest arc.
 *
 * Positions are interpolated linearly elsewhere; headings must be interpolated on the circle so the
 * result never sweeps the long way around the ±pi discontinuity.
 *
 * @param yaw_a Heading at ratio 0 [rad].
 * @param yaw_b Heading at ratio 1 [rad].
 * @param ratio Interpolation ratio in [0, 1].
 * @return Interpolated, normalized heading [rad].
 */
double interpolate_yaw(double yaw_a, double yaw_b, double ratio);

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__CONVERSION__AGENT_HISTORY_ALIGNMENT_HPP_
