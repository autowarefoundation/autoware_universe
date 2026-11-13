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

#ifndef MRM_STOP_VELOCITY_PLANNER_HPP_
#define MRM_STOP_VELOCITY_PLANNER_HPP_

#include "stop_profile.hpp"
#include "type_alias.hpp"

namespace autoware::in_lane_mrm_planner
{

class MrmStopVelocityPlanner
{
public:
  struct DecelLimits
  {
    double jerk{};
    double decel{};
  };

  // Constraints of one deceleration profile (`mrm_velocity.profiles.<name>`).
  struct ProfileLimits
  {
    double target_jerk{};
    double target_deceleration{};
    double max_jerk_relaxation{};
    double max_deceleration_relaxation{};
  };

  explicit MrmStopVelocityPlanner(const Params & params);

  void update_params(const Params & params);

  void apply(
    TrajectoryPoints & points, const Odometry & odom, const AccelWithCovarianceStamped & accel,
    StopProfile profile = StopProfile::MODERATE) const;

  ProfileLimits profile_limits(StopProfile profile) const;

  static std::optional<size_t> find_constraint_stop_index(const TrajectoryPoints & points);

  double required_stop_distance(double v0, double a0, double jerk, double decel) const;

  DecelLimits select_profile_limits(
    const TrajectoryPoints & points, size_t ego_idx, size_t constraint_idx, double v0, double a0,
    StopProfile profile = StopProfile::MODERATE) const;

private:
  using MrmVelocityParams = Params::MrmVelocity;

  bool is_feasible(
    const TrajectoryPoints & points, size_t ego_idx, size_t constraint_idx, double v0, double a0,
    double jerk, double decel) const;

  double effective_initial_accel(double a0) const;

  void densify_near_arc_length(TrajectoryPoints & points, double center_arc_length) const;

  void fill_forward(
    TrajectoryPoints & points, size_t ego_idx, double v0, double a0, double jerk,
    double decel) const;

  static void fill_ego_prefix(TrajectoryPoints & points, size_t ego_idx, double v0, double a0);

  static void fill_zero_velocity_profile(TrajectoryPoints & points, float longitudinal_accel_mps2);

  static void apply_zero_stop_profile(
    TrajectoryPoints & points, const Odometry & odom, float longitudinal_accel_mps2);

  MrmVelocityParams params_;
};

}  // namespace autoware::in_lane_mrm_planner

#endif  // MRM_STOP_VELOCITY_PLANNER_HPP_
