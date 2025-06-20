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

#ifndef AUTOWARE__PLANNING_VALIDATOR_TRAJECTORY_CHECKER__PARAMETERS_HPP_
#define AUTOWARE__PLANNING_VALIDATOR_TRAJECTORY_CHECKER__PARAMETERS_HPP_

namespace autoware::planning_validator
{

struct TrajectoryCheck
{
  bool enable = true;
  bool is_critical = false;
  double threshold{};
};

struct TrajectoryCheckerParams
{
  TrajectoryCheck interval;
  TrajectoryCheck relative_angle;
  TrajectoryCheck curvature;
  TrajectoryCheck steering;
  TrajectoryCheck steering_rate;
  TrajectoryCheck lateral_jerk;
  TrajectoryCheck lateral_accel;
  TrajectoryCheck max_lon_accel;
  TrajectoryCheck min_lon_accel;
  TrajectoryCheck velocity_deviation;
  TrajectoryCheck distance_deviation;
  TrajectoryCheck lon_distance_deviation;

  struct YawDeviation : TrajectoryCheck
  {
    double nearest_yaw_trajectory_shift_required_for_checking{};
  } yaw_deviation;

  struct TrajectoryShift : TrajectoryCheck
  {
    double lat_shift_th;
    double forward_shift_th;
    double backward_shift_th;
  } trajectory_shift;

  struct ForwardTrajectoryLength : TrajectoryCheck
  {
    double acceleration;
    double margin;
  } forward_trajectory_length{};
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR_TRAJECTORY_CHECKER__PARAMETERS_HPP_
