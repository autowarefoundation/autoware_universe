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

// cspell:ignore CTRV tmpl

#include "autoware/diffusion_planner/conversion/agent_history_resampler.hpp"

#include "autoware/diffusion_planner/constants.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <rclcpp/duration.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace autoware::diffusion_planner
{

namespace
{

// Fixed CTRV integration sub-step [s]; a larger extrapolation dt is split into steps no longer than
// this to bound forward-Euler error (matches the tracker's dt_max sub-stepping).
constexpr double CTRV_DT_SUB_STEP_MAX_S = 0.11;

// A bracketing observation pair whose headings differ by more than this [rad] (135 deg) is
// treated as a tracker heading flip: the grid slot snaps to the nearer observation.
constexpr double FLIP_YAW_THRESHOLD_RAD = 2.35619449;

// Heading of a pose matrix in its own frame.
double get_yaw(const Eigen::Matrix4d & pose)
{
  return std::atan2(pose(1, 0), pose(0, 0));
}

// Signed longitudinal speed: the twist is body-frame, so the sign of linear.x carries reverse
// motion into the CTRV propagation.
double get_speed(const AgentState & state)
{
  return state.original_info.kinematics.twist_with_covariance.twist.linear.x;
}

double get_yaw_rate(const AgentState & state)
{
  return state.original_info.kinematics.twist_with_covariance.twist.angular.z;
}

// Synthesize a grid state from a template observation, overwriting its planar pose and twist.
AgentState make_grid_state(
  const TrackedObject & tmpl, const double x, const double y, const double yaw,
  const rclcpp::Time & timestamp, const double speed)
{
  TrackedObject object = tmpl;
  auto & pose = object.kinematics.pose_with_covariance.pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  auto & linear = object.kinematics.twist_with_covariance.twist.linear;
  linear.x = speed;
  linear.y = 0.0;
  linear.z = 0.0;
  return AgentState(object, timestamp);
}

}  // namespace

MotionState propagate_motion(
  const MotionState & state, const double speed, const double yaw_rate, double dt,
  const HistoryResamplingParams & params)
{
  if (dt == 0.0) {
    return state;
  }
  if (dt > 0.0) {
    dt = std::min(dt, params.max_extrapolation_time);
  }

  // Sub-step to bound forward-Euler error, matching the tracker's dt_max sub-stepping. A negative
  // dt integrates the same equations backward; the step inherits its sign.
  const int num_steps =
    std::max(1, static_cast<int>(std::ceil(std::abs(dt) / CTRV_DT_SUB_STEP_MAX_S)));
  const double step = dt / static_cast<double>(num_steps);

  MotionState s = state;
  for (int i = 0; i < num_steps; ++i) {
    s.x += speed * std::cos(s.yaw) * step;
    s.y += speed * std::sin(s.yaw) * step;
    s.yaw += yaw_rate * step;
  }
  s.yaw = autoware_utils_math::normalize_radian(s.yaw);
  return s;
}

double interpolate_yaw(const double yaw_a, const double yaw_b, const double ratio)
{
  const double delta = autoware_utils_math::normalize_radian(yaw_b - yaw_a);
  return autoware_utils_math::normalize_radian(yaw_a + ratio * delta);
}

std::optional<AgentHistory> resample_history(
  const AgentHistory & history, const rclcpp::Time & frame_time,
  const HistoryResamplingParams & params)
{
  if (history.empty()) {
    return std::nullopt;
  }

  const auto & raw = history.states();
  const size_t n = raw.size();

  // Per-observation kinematics, snapshotted raw: a tracker heading flip re-expresses the state as
  // (yaw + pi, -speed), which the CTRV propagation maps to identical positions, and flipped
  // headings match the training-data signature. Interpolation across a flipped pair snaps to the
  // nearer observation (see below).
  std::vector<double> obs_x(n);
  std::vector<double> obs_y(n);
  std::vector<double> obs_yaw(n);
  std::vector<double> obs_speed(n);
  std::vector<double> obs_yaw_rate(n);
  std::vector<double> obs_time(n);
  for (size_t i = 0; i < n; ++i) {
    obs_x[i] = raw[i].pose(0, 3);
    obs_y[i] = raw[i].pose(1, 3);
    obs_yaw[i] = get_yaw(raw[i].pose);
    obs_speed[i] = get_speed(raw[i]);
    obs_yaw_rate[i] = get_yaw_rate(raw[i]);
    obs_time[i] = raw[i].timestamp.seconds();
  }

  const double newest_time = obs_time[n - 1];
  const TrackedObject & newest_tmpl = raw[n - 1].original_info;

  constexpr double dt = constants::PREDICTION_TIME_STEP_S;
  const size_t num_timesteps = static_cast<size_t>(INPUT_T_WITH_CURRENT);
  const double ref_sec = frame_time.seconds();

  std::vector<AgentState> grid_states;
  grid_states.reserve(num_timesteps);

  size_t search_start = 0;  // obs times are ascending; carry the bracket search forward
  for (size_t k = 0; k < num_timesteps; ++k) {
    // k = 0 is the oldest slot, k = num_timesteps - 1 is anchored at frame_time.
    const double target_sec = ref_sec - static_cast<double>(num_timesteps - 1 - k) * dt;
    const rclcpp::Time target_time(
      frame_time - rclcpp::Duration::from_seconds(static_cast<double>(num_timesteps - 1 - k) * dt));

    if (target_sec <= obs_time[0]) {
      // Before the oldest observation. Within the backward horizon, extrapolate via the motion
      // model. Beyond the horizon, freeze at the oldest observation, keeping its nonzero speed —
      // the repeat-fill signature the model saw for pre-appearance slots in training.
      const double back_dt = target_sec - obs_time[0];  // <= 0
      if (-back_dt > params.max_extrapolation_time) {
        grid_states.push_back(make_grid_state(
          raw[0].original_info, obs_x[0], obs_y[0], obs_yaw[0], target_time, obs_speed[0]));
      } else {
        const MotionState start{obs_x[0], obs_y[0], obs_yaw[0]};
        const MotionState propagated =
          propagate_motion(start, obs_speed[0], obs_yaw_rate[0], back_dt, params);
        grid_states.push_back(make_grid_state(
          raw[0].original_info, propagated.x, propagated.y, propagated.yaw, target_time,
          obs_speed[0]));
      }
    } else if (target_sec <= newest_time) {
      // Within the observed history: interpolate position linearly, yaw along the shortest arc.
      while (search_start + 1 < n && obs_time[search_start + 1] < target_sec) {
        ++search_start;
      }
      const size_t i0 = search_start;
      const size_t i1 = search_start + 1;
      const double span = obs_time[i1] - obs_time[i0];
      const double ratio = (span > 0.0) ? (target_sec - obs_time[i0]) / span : 0.0;
      const double x = obs_x[i0] + ratio * (obs_x[i1] - obs_x[i0]);
      const double y = obs_y[i0] + ratio * (obs_y[i1] - obs_y[i0]);
      // A near-antipodal pair is a tracker heading flip: snap yaw and speed to the nearer
      // observation; sweeping would fabricate perpendicular headings.
      const double yaw_delta = autoware_utils_math::normalize_radian(obs_yaw[i1] - obs_yaw[i0]);
      double yaw{};
      double speed{};
      if (std::abs(yaw_delta) > FLIP_YAW_THRESHOLD_RAD) {
        const size_t nearer = (ratio < 0.5) ? i0 : i1;
        yaw = obs_yaw[nearer];
        speed = obs_speed[nearer];
      } else {
        yaw = interpolate_yaw(obs_yaw[i0], obs_yaw[i1], ratio);
        speed = obs_speed[i0] + ratio * (obs_speed[i1] - obs_speed[i0]);
      }
      grid_states.push_back(make_grid_state(raw[i1].original_info, x, y, yaw, target_time, speed));
    } else {
      // After the newest observation: extrapolate the leading edge forward via the motion model.
      // Emitted agents are present in the latest message, so the newest observation lags
      // frame_time by at most one tracker cycle and the extrapolation stays clamped.
      const MotionState start{obs_x[n - 1], obs_y[n - 1], obs_yaw[n - 1]};
      const MotionState propagated = propagate_motion(
        start, obs_speed[n - 1], obs_yaw_rate[n - 1], target_sec - newest_time, params);
      grid_states.push_back(make_grid_state(
        newest_tmpl, propagated.x, propagated.y, propagated.yaw, target_time, obs_speed[n - 1]));
    }
  }

  return AgentHistory::from_states(grid_states, num_timesteps);
}

}  // namespace autoware::diffusion_planner
