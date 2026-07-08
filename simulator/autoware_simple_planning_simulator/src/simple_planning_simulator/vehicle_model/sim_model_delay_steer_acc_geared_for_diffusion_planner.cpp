// Copyright 2025 The Autoware Foundation.
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

#include "autoware/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_geared_for_diffusion_planner.hpp"

#include "autoware_vehicle_msgs/msg/gear_command.hpp"

#include <algorithm>
#include <cmath>
#include <deque>

namespace autoware::simulator::simple_planning_simulator
{

SimModelDelaySteerAccGearedForDiffusionPlanner::SimModelDelaySteerAccGearedForDiffusionPlanner(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double acc_delay, double acc_time_constant, double steer_delay,
  double steer_time_constant, double steer_dead_band, double steer_bias,
  double debug_acc_scaling_factor, double debug_steer_scaling_factor, double k_us)
: SimModelInterface(7 /* dim x */, 4 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  acc_delay_(acc_delay),
  acc_time_constant_(std::max(acc_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  steer_dead_band_(steer_dead_band),
  steer_bias_(steer_bias),
  debug_acc_scaling_factor_(std::max(debug_acc_scaling_factor, 0.0)),
  debug_steer_scaling_factor_(std::max(debug_steer_scaling_factor, 0.0)),
  k_us_(k_us)
{
  initializeInputQueue(dt);
  initializeStateQueue(dt);
}

double SimModelDelaySteerAccGearedForDiffusionPlanner::calc_yaw_rate(double vel, double steer) const
{
  const double denom = wheelbase_ + k_us_ * vel * vel;
  return vel * std::tan(steer + steer_bias_) / denom;
}

double SimModelDelaySteerAccGearedForDiffusionPlanner::getX()
{
  return state_(IDX::X);
}

double SimModelDelaySteerAccGearedForDiffusionPlanner::getY()
{
  return state_(IDX::Y);
}

double SimModelDelaySteerAccGearedForDiffusionPlanner::getYaw()
{
  return state_(IDX::YAW);
}

double SimModelDelaySteerAccGearedForDiffusionPlanner::getVx()
{
  return state_(IDX::VX);
}

double SimModelDelaySteerAccGearedForDiffusionPlanner::getVy()
{
  return 0.0;
}

double SimModelDelaySteerAccGearedForDiffusionPlanner::getAx()
{
  return state_(IDX::ACCX);
}

double SimModelDelaySteerAccGearedForDiffusionPlanner::getWz()
{
  return calc_yaw_rate(state_(IDX::VX), state_(IDX::STEER));
}

double SimModelDelaySteerAccGearedForDiffusionPlanner::getSteer()
{
  // Steer bias now enters the yaw equation (calc_yaw_rate, as the viewer's β), not the
  // measured-steer/tracking loop, so that the bias produces a net yaw offset instead of being
  // cancelled by the steer controller. The reported/tracked steer is the actual state value.
  return state_(IDX::STEER);
}

void SimModelDelaySteerAccGearedForDiffusionPlanner::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

  // Command delay queues advance once per update() call (as wo_fall_guard).
  acc_input_queue_.push_back(input_(IDX_U::PEDAL_ACCX_DES));
  delayed_input(IDX_U::PEDAL_ACCX_DES) = acc_input_queue_.front();
  acc_input_queue_.pop_front();
  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();
  delayed_input(IDX_U::GEAR) = input_(IDX_U::GEAR);
  delayed_input(IDX_U::SLOPE_ACCX) = input_(IDX_U::SLOPE_ACCX);

  // full-RHS delay: sample the state feedback for the steer / accel channels at t-d as well.
  // The queues advance identically to the command queues (same size round(delay/dt)); the popped
  // value is frozen for the whole update so the steer / pedal derivatives are constant over
  // dt, matching Python's per-step frozen RHS (e_eff / e_a_eff computed once per step).
  steer_state_queue_.push_back(state_(IDX::STEER));
  delayed_steer_state_ = steer_state_queue_.front();
  steer_state_queue_.pop_front();
  pedal_state_queue_.push_back(state_(IDX::PEDAL_ACCX));
  delayed_pedal_state_ = pedal_state_queue_.front();
  pedal_state_queue_.pop_front();

  const auto prev_state = state_;
  // we cannot use updateRungeKutta() because the differentiability or the continuity
  // condition is not satisfied, but we can use Runge-Kutta method with code reconstruction.
  updateEuler(dt, delayed_input);
  // take velocity limit
  state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));

  // Stop condition: detect zero crossing over the full dt window using the outer prev_state.
  if (
    prev_state(IDX::VX) * state_(IDX::VX) <= 0.0 &&
    -state_(IDX::PEDAL_ACCX) >= std::abs(delayed_input(IDX_U::SLOPE_ACCX))) {
    state_(IDX::VX) = 0.0;
  }

  state_(IDX::ACCX) = (state_(IDX::VX) - prev_state(IDX::VX)) / dt;
}

void SimModelDelaySteerAccGearedForDiffusionPlanner::initializeInputQueue(const double & dt)
{
  size_t acc_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  acc_input_queue_.resize(acc_input_queue_size);
  std::fill(acc_input_queue_.begin(), acc_input_queue_.end(), 0.0);

  size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_input_queue_.resize(steer_input_queue_size);
  std::fill(steer_input_queue_.begin(), steer_input_queue_.end(), 0.0);
}

void SimModelDelaySteerAccGearedForDiffusionPlanner::initializeStateQueue(const double & dt)
{
  // State history buffers mirror the command-queue sizes so the delayed state feedback lags by the
  // same round(delay/dt) samples. Zero-fill (exactly like initializeInputQueue's command-queue
  // fill) so the realtime path — which constructs the model and never calls resetStateQueues() —
  // reads a well-defined at-rest pre-window state for the first round(delay/dt) steps, without
  // relying on the base class having zero-initialised state_.
  const size_t steer_state_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
  steer_state_queue_.assign(steer_state_queue_size, 0.0);

  const size_t acc_state_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
  pedal_state_queue_.assign(acc_state_queue_size, 0.0);
}

void SimModelDelaySteerAccGearedForDiffusionPlanner::resetStateQueues()
{
  // Re-fill the delayed-state history with the current state (constant). Called after the state is
  // seeded (reset / warmup restore) so full-RHS delay reads the steady pre-window state instead of
  // the warm-up transient the substep loop would otherwise have pushed.
  std::fill(steer_state_queue_.begin(), steer_state_queue_.end(), state_(IDX::STEER));
  std::fill(pedal_state_queue_.begin(), pedal_state_queue_.end(), state_(IDX::PEDAL_ACCX));
}

void SimModelDelaySteerAccGearedForDiffusionPlanner::setInputQueues(
  const std::deque<double> & acc_queue, const std::deque<double> & steer_queue)
{
  acc_input_queue_ = acc_queue;
  steer_input_queue_ = steer_queue;
}

int SimModelDelaySteerAccGearedForDiffusionPlanner::getAccQueueSize() const
{
  return static_cast<int>(acc_input_queue_.size());
}

int SimModelDelaySteerAccGearedForDiffusionPlanner::getSteerQueueSize() const
{
  return static_cast<int>(steer_input_queue_.size());
}

Eigen::VectorXd SimModelDelaySteerAccGearedForDiffusionPlanner::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };

  // Current state — used by the un-delayed channels (position, yaw, and the VX gear law, which
  // integrates the current PEDAL_ACCX exactly as Python's dot_v = a uses the current a).
  const double vel = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const double pedal_acc = sat(state(IDX::PEDAL_ACCX), vx_rate_lim_, -vx_rate_lim_);
  const double yaw = state(IDX::YAW);
  const double steer = state(IDX::STEER);
  // Delayed state feedback (frozen per update) — used by the steer / accel channel derivatives so
  // that their whole RHS is evaluated at t-d (full-RHS delay). Reduces to the current state when
  // the corresponding delay is 0.
  const double pedal_acc_delayed = sat(delayed_pedal_state_, vx_rate_lim_, -vx_rate_lim_);
  const double steer_delayed = delayed_steer_state_;
  const double pedal_acc_des =
    sat(input(IDX_U::PEDAL_ACCX_DES), vx_rate_lim_, -vx_rate_lim_) * debug_acc_scaling_factor_;
  const double steer_des =
    sat(input(IDX_U::STEER_DES), steer_lim_, -steer_lim_) * debug_steer_scaling_factor_;

  // Yaw rate (with k_us understeer and steer-bias β); un-delayed (yaw observation delay d_tt is out
  // of scope for this derivative and left for a future improvement).
  const double yaw_rate = calc_yaw_rate(vel, steer);
  // Acceleration target: the actuator (PEDAL_ACCX) tracks the desired accel with a single-tau
  // first-order lag (same form as wo_fall_guard).
  const double pedal_acc_target = pedal_acc_des;
  const double acc_tau = acc_time_constant_;
  // NOTE: `steer_des` is calculated by control from measured values, delayed by the command queue.
  // full-RHS delay: the measured steer that closes the tracking loop is also taken at t-steer_delay
  // (delayed_steer_state_), not the current steer, so the steer channel's whole RHS lags together.
  const double steer_diff = steer_delayed - steer_des;
  const double steer_diff_with_dead_band = std::invoke([&]() {
    if (steer_diff > steer_dead_band_) {
      return steer_diff - steer_dead_band_;
    } else if (steer_diff < -steer_dead_band_) {
      return steer_diff + steer_dead_band_;
    } else {
      return 0.0;
    }
  });
  const double steer_rate =
    sat(-steer_diff_with_dead_band / steer_time_constant_, steer_rate_lim_, -steer_rate_lim_);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);

  d_state(IDX::X) = vel * cos(yaw);
  d_state(IDX::Y) = vel * sin(yaw);
  d_state(IDX::YAW) = yaw_rate;
  d_state(IDX::VX) = [&] {
    if (pedal_acc >= 0.0) {
      using autoware_vehicle_msgs::msg::GearCommand;
      const auto gear = input(IDX_U::GEAR);
      if (gear == GearCommand::NONE || gear == GearCommand::PARK) {
        return 0.0;
      } else if (gear == GearCommand::NEUTRAL) {
        return input(IDX_U::SLOPE_ACCX);
      } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
        return -pedal_acc + input(IDX_U::SLOPE_ACCX);
      } else {
        return pedal_acc + input(IDX_U::SLOPE_ACCX);
      }
    } else {
      if (vel > 0.0) {
        return pedal_acc + input(IDX_U::SLOPE_ACCX);
      } else if (vel < 0.0) {
        return -pedal_acc + input(IDX_U::SLOPE_ACCX);
      } else if (-pedal_acc >= std::abs(input(IDX_U::SLOPE_ACCX))) {
        return 0.0;
      } else {
        return input(IDX_U::SLOPE_ACCX);
      }
    }
  }();
  d_state(IDX::STEER) = steer_rate;
  d_state(IDX::PEDAL_ACCX) = -(pedal_acc_delayed - pedal_acc_target) / acc_tau;

  return d_state;
}

}  // namespace autoware::simulator::simple_planning_simulator
