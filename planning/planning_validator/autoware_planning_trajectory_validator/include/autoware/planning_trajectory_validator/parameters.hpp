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

#ifndef AUTOWARE__PLANNING_TRAJECTORY_VALIDATOR__PARAMETERS_HPP_
#define AUTOWARE__PLANNING_TRAJECTORY_VALIDATOR__PARAMETERS_HPP_

namespace autoware::planning_validator
{

struct TrajectoryValidityCheck
{
  bool enable = true;
  bool is_critical = false;
  double threshold{};
};

struct TrajectoryValidatorParams
{
  TrajectoryValidityCheck interval;
  TrajectoryValidityCheck relative_angle;
  TrajectoryValidityCheck curvature;
  TrajectoryValidityCheck steering;
  TrajectoryValidityCheck steering_rate;
  TrajectoryValidityCheck lateral_jerk;

  struct AccelerationCheck : TrajectoryValidityCheck
  {
    double lateral_th;
    double longitudinal_max_th;
    double longitudinal_min_th;
  } acceleration{};

  struct DeviationCheck : TrajectoryValidityCheck
  {
    double velocity_th;
    double distance_th;
    double lon_distance_th;
    double yaw_th;
  } deviation{};

  struct TrajectoryShift : TrajectoryValidityCheck
  {
    double lat_shift_th;
    double forward_shift_th;
    double backward_shift_th;
  } trajectory_shift;

  struct ForwardTrajectoryLength : TrajectoryValidityCheck
  {
    double acceleration;
    double margin;
  } forward_trajectory_length{};
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_TRAJECTORY_VALIDATOR__PARAMETERS_HPP_
