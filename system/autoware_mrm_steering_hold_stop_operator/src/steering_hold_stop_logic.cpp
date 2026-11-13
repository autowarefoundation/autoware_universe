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

#include <algorithm>
#include <cmath>

namespace autoware::mrm_steering_hold_stop_operator
{

SteeringHoldStopLogic::SteeringHoldStopLogic(const LogicParams & params) : params_(params)
{
}

UpdateResult SteeringHoldStopLogic::update(
  const TriggerState & trigger, const MeasuredInputs & inputs, const double dt)
{
  if (trigger.active) {
    // Once decelerating, keep integrating open-loop regardless of input freshness.
    if (state_ == State::kDecelerating) {
      return step_deceleration(trigger, dt);
    }
    if (are_required_inputs_fresh(inputs)) {
      return initialize_deceleration(trigger, inputs);
    }
    state_ = State::kWaitingInput;
    return UpdateResult{state_, std::nullopt};
  }

  // Trigger OFF: mirror measured state (or wait for fresh inputs).
  if (are_required_inputs_fresh(inputs)) {
    return update_mirror(inputs);
  }
  state_ = State::kWaitingInput;
  return UpdateResult{state_, std::nullopt};
}

bool SteeringHoldStopLogic::is_fresh(const std::optional<double> & age) const
{
  return age.has_value() && *age <= params_.input_freshness_timeout;
}

bool SteeringHoldStopLogic::are_required_inputs_fresh(const MeasuredInputs & inputs) const
{
  if (!is_fresh(inputs.steering_age) || !is_fresh(inputs.odom_age)) {
    return false;
  }
  // Acceleration measurement is unstable at low speed: it is neither used nor
  // required below low_speed_threshold.
  const bool is_low_speed = std::abs(inputs.longitudinal_velocity) < params_.low_speed_threshold;
  if (is_low_speed) {
    return true;
  }
  return is_fresh(inputs.accel_age);
}

UpdateResult SteeringHoldStopLogic::update_mirror(const MeasuredInputs & inputs)
{
  state_ = State::kMirror;

  const bool is_low_speed = std::abs(inputs.longitudinal_velocity) < params_.low_speed_threshold;

  CommandOutput command;
  command.steering_tire_angle = inputs.steering_tire_angle;
  command.velocity = inputs.longitudinal_velocity;
  // Clamp to <= 0 so that a positive acceleration command is never selected by
  // the gate during the small race window between trigger delivery and the
  // gate switching its source to in_lane_stop.
  command.acceleration = is_low_speed ? 0.0 : std::min(inputs.longitudinal_acceleration, 0.0);
  command.jerk = 0.0;
  return UpdateResult{state_, command};
}

UpdateResult SteeringHoldStopLogic::initialize_deceleration(
  const TriggerState & trigger, const MeasuredInputs & inputs)
{
  state_ = State::kDecelerating;

  const double target_acceleration = effective_target_acceleration(trigger);
  const double target_jerk = effective_target_jerk(trigger, target_acceleration);

  theta_hold_ = inputs.steering_tire_angle;
  velocity_ = std::max(inputs.longitudinal_velocity, 0.0);

  const bool is_low_speed = velocity_ < params_.low_speed_threshold;
  if (is_low_speed || target_jerk == 0.0) {
    // Low speed: skip the jerk ramp and start at the target deceleration.
    acceleration_ = target_acceleration;
  } else {
    // Upper clamp 0: release acceleration immediately at MRM start. This also
    // guards against an erroneously positive acceleration measurement.
    acceleration_ = std::clamp(inputs.longitudinal_acceleration, target_acceleration, 0.0);
  }

  CommandOutput command;
  command.steering_tire_angle = theta_hold_;
  command.velocity = velocity_;
  command.acceleration = acceleration_;
  command.jerk = (acceleration_ <= target_acceleration) ? 0.0 : target_jerk;
  return UpdateResult{state_, command};
}

UpdateResult SteeringHoldStopLogic::step_deceleration(const TriggerState & trigger, const double dt)
{
  const double target_acceleration = effective_target_acceleration(trigger);
  const double target_jerk = effective_target_jerk(trigger, target_acceleration);
  const double clamped_dt = std::max(dt, 0.0);

  velocity_ = std::max(velocity_ + acceleration_ * clamped_dt, 0.0);
  if (target_jerk == 0.0) {
    acceleration_ = target_acceleration;
  } else {
    acceleration_ = std::max(acceleration_ + target_jerk * clamped_dt, target_acceleration);
  }

  CommandOutput command;
  command.steering_tire_angle = theta_hold_;
  command.velocity = velocity_;
  command.acceleration = acceleration_;
  command.jerk = (acceleration_ <= target_acceleration) ? 0.0 : target_jerk;
  return UpdateResult{state_, command};
}

double SteeringHoldStopLogic::effective_target_acceleration(const TriggerState & trigger)
{
  // The targets come from the per-profile parameters and must be negative.
  // Clamp defensively so a misconfiguration can never command acceleration.
  return std::min(trigger.target_acceleration, 0.0);
}

double SteeringHoldStopLogic::effective_target_jerk(
  const TriggerState & trigger, const double target_acceleration)
{
  const double jerk = std::min(trigger.target_jerk, 0.0);
  // A zero jerk would never reach the target: degrade to a stepwise transition
  // (handled by the callers). Signal it by returning 0 explicitly.
  return (target_acceleration < 0.0) ? jerk : 0.0;
}

}  // namespace autoware::mrm_steering_hold_stop_operator
