// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_VALIDATOR__PARAMETERS_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__PARAMETERS_HPP_

#endif  // AUTOWARE__PLANNING_VALIDATOR__PARAMETERS_HPP_

namespace autoware::planning_validator
{
enum class InvalidTrajectoryHandlingType : uint8_t {
  PUBLISH_AS_IT_IS,
  STOP_PUBLISHING,
  USE_PREVIOUS_RESULT,
};

struct ValidityCheck
{
  bool enable = false;
  bool is_critical = false;
  double threshold;
};

struct ValidationParams
{
  ValidityCheck interval;
  ValidityCheck relative_angle;
  ValidityCheck curvature;
  ValidityCheck latency;

  struct SteeringCheck : ValidityCheck
  {
    double rate_th;
  } steering;

  struct AccelerationCheck : ValidityCheck
  {
    double lateral_th;
    double longitudinal_max_th;
    double longitudinal_min_th;
  } acceleration;

  struct DeviationCheck : ValidityCheck
  {
    double velocity_th;
    double distance_th;
    double lon_distance_th;
    double yaw_th;
  } deviation;

  struct TrajectoryShift : ValidityCheck
  {
    double lat_shift_th;
    double forward_shift_th;
    double backward_shift_th;
  } trajectory_shift;

  struct ForwardTrajectoryLength : ValidityCheck
  {
    double acceleration;
    double margin;
  } forward_trajectory_length;
};

struct Params
{
  bool enable_soft_stop_on_prev_traj = true;
  bool publish_diag = true;
  bool display_on_terminal = true;
  double soft_stop_deceleration = -1.0;
  int diag_error_count_threshold = 0;
  ValidationParams validation_params;
  InvalidTrajectoryHandlingType inv_traj_handling_type;
  InvalidTrajectoryHandlingType inv_traj_critical_handling_type;
};
}  // namespace autoware::planning_validator
