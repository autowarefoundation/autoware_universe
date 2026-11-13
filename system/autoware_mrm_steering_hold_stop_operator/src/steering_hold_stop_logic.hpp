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

#ifndef STEERING_HOLD_STOP_LOGIC_HPP_
#define STEERING_HOLD_STOP_LOGIC_HPP_

#include <optional>

namespace autoware::mrm_steering_hold_stop_operator
{

enum class State {
  kMirror = 0,        // trigger OFF: mirror measured state
  kWaitingInput = 1,  // required inputs stale/missing: no command output
  kDecelerating = 2,  // trigger ON: steering hold + constant jerk deceleration
};

struct LogicParams
{
  double input_freshness_timeout{0.5};  // [s]
  double low_speed_threshold{1.0};      // [m/s]
};

// Measured inputs with their ages. nullopt age = never received.
struct MeasuredInputs
{
  std::optional<double> steering_age;     // [s]
  std::optional<double> odom_age;         // [s]
  std::optional<double> accel_age;        // [s]
  double steering_tire_angle{0.0};        // [rad]
  double longitudinal_velocity{0.0};      // [m/s]
  double longitudinal_acceleration{0.0};  // [m/s^2]
};

struct TriggerState
{
  bool active{false};
  double target_acceleration{0.0};  // [m/s^2] (negative)
  double target_jerk{0.0};          // [m/s^3] (negative)
};

struct CommandOutput
{
  double steering_tire_angle{0.0};  // [rad]
  double velocity{0.0};             // [m/s]
  double acceleration{0.0};         // [m/s^2]
  double jerk{0.0};                 // [m/s^3]
};

struct UpdateResult
{
  State state{State::kMirror};
  // nullopt = do not publish a command this cycle (kWaitingInput)
  std::optional<CommandOutput> command;
};

/// State machine for the steering-hold constant-jerk stop operator.
/// All time handling is injected (ages and dt) so the logic is testable
/// without spinning a node.
class SteeringHoldStopLogic
{
public:
  explicit SteeringHoldStopLogic(const LogicParams & params);

  /// Advance one timer tick.
  /// @param trigger latest trigger state (targets may be updated while active)
  /// @param inputs  measured inputs and their ages at this tick
  /// @param dt      elapsed time since the previous tick [s]
  UpdateResult update(const TriggerState & trigger, const MeasuredInputs & inputs, double dt);

  State state() const { return state_; }
  double held_steering_angle() const { return theta_hold_; }

private:
  bool is_fresh(const std::optional<double> & age) const;
  bool are_required_inputs_fresh(const MeasuredInputs & inputs) const;

  UpdateResult update_mirror(const MeasuredInputs & inputs);
  UpdateResult initialize_deceleration(const TriggerState & trigger, const MeasuredInputs & inputs);
  UpdateResult step_deceleration(const TriggerState & trigger, double dt);

  static double effective_target_acceleration(const TriggerState & trigger);
  static double effective_target_jerk(const TriggerState & trigger, double target_acceleration);

  LogicParams params_;
  State state_{State::kMirror};

  // Deceleration state (valid while state_ == kDecelerating)
  double theta_hold_{0.0};
  double velocity_{0.0};
  double acceleration_{0.0};
};

}  // namespace autoware::mrm_steering_hold_stop_operator

#endif  // STEERING_HOLD_STOP_LOGIC_HPP_
