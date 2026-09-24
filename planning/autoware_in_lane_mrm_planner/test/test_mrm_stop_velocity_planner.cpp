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

#include "mrm_stop_velocity_planner.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

namespace autoware::in_lane_mrm_planner
{
namespace
{

Params make_default_params()
{
  Params params;
  params.mrm_velocity.target_deceleration = -3.0;
  params.mrm_velocity.target_jerk = -5.0;
  params.mrm_velocity.max_jerk_relaxation = -20.0;
  params.mrm_velocity.max_deceleration_relaxation = -6.0;
  params.mrm_velocity.step_jerk_relaxation = -5.0;
  params.mrm_velocity.step_deceleration_relaxation = -1.0;
  params.mrm_velocity.decel_resample_range = 2.0;
  params.mrm_velocity.decel_resample_interval = 0.1;
  params.mrm_velocity.brake_delay_time = 0.0;
  return params;
}

Params make_params_with_brake_delay(const double delay_time)
{
  auto params = make_default_params();
  params.mrm_velocity.brake_delay_time = delay_time;
  return params;
}

TrajectoryPoints make_straight_trajectory(
  const size_t num_points, const double spacing, const float velocity)
{
  TrajectoryPoints points;
  points.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = spacing * static_cast<double>(i);
    point.pose.position.y = 0.0;
    point.pose.position.z = 0.0;
    point.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
    point.longitudinal_velocity_mps = velocity;
    point.acceleration_mps2 = 0.0F;
    points.push_back(point);
  }
  return points;
}

TrajectoryPoints make_straight_trajectory_with_constraint_at(
  const size_t num_points, const double spacing, const float velocity, const size_t constraint_idx)
{
  auto points = make_straight_trajectory(num_points, spacing, velocity);
  points.at(constraint_idx).longitudinal_velocity_mps = 0.0F;
  return points;
}

Odometry make_odometry(const double velocity)
{
  Odometry odom;
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
  odom.twist.twist.linear.x = velocity;
  return odom;
}

AccelWithCovarianceStamped make_accel(const double ax)
{
  AccelWithCovarianceStamped accel;
  accel.accel.accel.linear.x = ax;
  return accel;
}

size_t find_first_stopped_index(const TrajectoryPoints & points)
{
  for (size_t i = 0; i < points.size(); ++i) {
    if (points.at(i).longitudinal_velocity_mps <= 0.01F) {
      return i;
    }
  }
  return points.size() - 1;
}

double arc_length_at_index(const TrajectoryPoints & points, const size_t idx)
{
  return autoware::motion_utils::calcSignedArcLength(points, 0, idx);
}

double min_spacing_in_arc_range(
  const TrajectoryPoints & points, const double s_begin, const double s_end)
{
  double min_ds = std::numeric_limits<double>::max();
  for (size_t i = 1; i < points.size(); ++i) {
    const double s0 = arc_length_at_index(points, i - 1);
    const double s1 = arc_length_at_index(points, i);
    if (s1 < s_begin || s0 > s_end) {
      continue;
    }
    min_ds = std::min(min_ds, s1 - s0);
  }
  return min_ds;
}

}  // namespace

TEST(MrmStopVelocityPlannerTest, DensifyDoesNotEmitOverlappingPointsAtTrajectoryEnd)
{
  // Trajectory ends 1.0 m ahead of a still-moving ego: the densify window reaches the trajectory
  // terminal, where the accumulated 0.1 m grid samples collide with the exact terminal arc length
  // in floating point. Regression for the duplicated terminal point that broke MPC spline
  // resampling (2026-06-30 in-lane MRM incident).
  auto points = make_straight_trajectory(11, 0.1, 1.0F);
  const MrmStopVelocityPlanner planner(make_default_params());

  planner.apply(points, make_odometry(0.5), make_accel(-0.5));

  ASSERT_GE(points.size(), 2U);
  const double min_gap = 0.5 * 0.1;  // half of decel_resample_interval
  for (size_t i = 1; i < points.size(); ++i) {
    EXPECT_GE(arc_length_at_index(points, i) - arc_length_at_index(points, i - 1), min_gap * 0.99)
      << "overlapping points at index " << i - 1 << " and " << i;
  }
  // The exact trajectory terminal must survive the spacing filter.
  EXPECT_NEAR(points.back().pose.position.x, 1.0, 1e-6);
}

TEST(MrmStopVelocityPlannerTest, RequiredDistanceFitsWithin30mAtTargetLimits)
{
  const MrmStopVelocityPlanner planner(make_default_params());
  const double required = planner.required_stop_distance(10.0, 0.0, -5.0, -3.0);
  EXPECT_LT(required, 30.0);
  EXPECT_GT(required, 15.0);
}

TEST(MrmStopVelocityPlannerTest, StopsBeforeConstraintWithoutRelaxation)
{
  auto points = make_straight_trajectory_with_constraint_at(51, 1.0, 10.0F, 30);
  const MrmStopVelocityPlanner planner(make_default_params());

  const auto limits = planner.select_profile_limits(points, 0, 30, 10.0, 0.0);
  EXPECT_DOUBLE_EQ(limits.jerk, -5.0);
  EXPECT_DOUBLE_EQ(limits.decel, -3.0);

  planner.apply(points, make_odometry(10.0), make_accel(0.0));

  const auto stopped_idx = find_first_stopped_index(points);
  const double stopped_arc = arc_length_at_index(points, stopped_idx);
  const double required = planner.required_stop_distance(10.0, 0.0, -5.0, -3.0);

  EXPECT_LT(stopped_arc, 30.0);
  EXPECT_NEAR(stopped_arc, required, 2.0);
  EXPECT_GT(stopped_arc, required - 5.0);
}

TEST(MrmStopVelocityPlannerTest, RelaxationTriggeredWhenConstraintIsTooClose)
{
  auto points = make_straight_trajectory_with_constraint_at(25, 1.0, 10.0F, 17);
  const MrmStopVelocityPlanner planner(make_default_params());

  const auto limits = planner.select_profile_limits(points, 0, 17, 10.0, 0.0);
  EXPECT_LT(limits.jerk, -5.0);

  planner.apply(points, make_odometry(10.0), make_accel(0.0));
  EXPECT_LT(arc_length_at_index(points, find_first_stopped_index(points)), 17.0);
}

TEST(MrmStopVelocityPlannerTest, RelaxationReachesMaxDeceleration)
{
  auto points = make_straight_trajectory_with_constraint_at(15, 1.0, 10.0F, 10);
  const MrmStopVelocityPlanner planner(make_default_params());

  const auto limits = planner.select_profile_limits(points, 0, 10, 10.0, 0.0);
  EXPECT_DOUBLE_EQ(limits.jerk, -20.0);
  EXPECT_DOUBLE_EQ(limits.decel, -6.0);

  planner.apply(points, make_odometry(10.0), make_accel(0.0));
  EXPECT_LT(arc_length_at_index(points, find_first_stopped_index(points)), 10.0);
}

TEST(MrmStopVelocityPlannerTest, RelaxationTerminatesWhenStepCannotProgress)
{
  // Misconfigured params: zero relaxation step would never advance jerk/decel toward the
  // max limits, so the relaxation loop must not spin forever. It must fall back to the max
  // relaxation limits and return.
  auto params = make_default_params();
  params.mrm_velocity.step_jerk_relaxation = 0.0;
  params.mrm_velocity.step_deceleration_relaxation = 0.0;

  auto points = make_straight_trajectory_with_constraint_at(15, 1.0, 10.0F, 10);
  const MrmStopVelocityPlanner planner(params);

  const auto limits = planner.select_profile_limits(points, 0, 10, 10.0, 0.0);
  EXPECT_DOUBLE_EQ(limits.jerk, -20.0);
  EXPECT_DOUBLE_EQ(limits.decel, -6.0);
}

TEST(MrmStopVelocityPlannerTest, AcceleratingA0NeedsMoreDistanceThanZero)
{
  const MrmStopVelocityPlanner planner(make_default_params());
  const double d0 = planner.required_stop_distance(10.0, 0.0, -5.0, -3.0);
  const double d_accel = planner.required_stop_distance(10.0, 1.5, -5.0, -3.0);
  EXPECT_GT(d_accel, d0);
}

TEST(MrmStopVelocityPlannerTest, BrakingA0NeedsLessDistanceThanZero)
{
  const MrmStopVelocityPlanner planner(make_default_params());
  const double d0 = planner.required_stop_distance(10.0, 0.0, -5.0, -3.0);
  const double d_brake = planner.required_stop_distance(10.0, -2.0, -5.0, -3.0);
  EXPECT_LT(d_brake, d0);
}

TEST(MrmStopVelocityPlannerTest, ResamplesNearPredictedStopNotConstraintIndex)
{
  auto points = make_straight_trajectory_with_constraint_at(51, 1.0, 10.0F, 30);
  const MrmStopVelocityPlanner planner(make_default_params());
  const double predicted_stop = planner.required_stop_distance(10.0, 0.0, -5.0, -3.0);

  planner.apply(points, make_odometry(10.0), make_accel(0.0));

  const double dense_near_predicted =
    min_spacing_in_arc_range(points, predicted_stop - 2.0, predicted_stop + 2.0);
  const double spacing_near_constraint = min_spacing_in_arc_range(points, 28.0, 32.0);

  EXPECT_LT(dense_near_predicted, 0.15);
  EXPECT_GT(spacing_near_constraint, 0.5);
}

TEST(MrmStopVelocityPlannerTest, FindConstraintStopIndexDetectsFirstZeroVelocity)
{
  auto points = make_straight_trajectory(10, 1.0, 5.0F);
  points.at(3).longitudinal_velocity_mps = 0.0F;

  const auto stop_idx = MrmStopVelocityPlanner::find_constraint_stop_index(points);
  ASSERT_TRUE(stop_idx.has_value());
  EXPECT_EQ(stop_idx.value(), 3U);
}

TEST(MrmStopVelocityPlannerTest, PlansStopAtTrajectoryEndWithoutConstraint)
{
  auto points = make_straight_trajectory(31, 1.0, 8.0F);
  const MrmStopVelocityPlanner planner(make_default_params());

  planner.apply(points, make_odometry(8.0), make_accel(0.0));

  EXPECT_NEAR(points.back().longitudinal_velocity_mps, 0.0, 1e-3);
  bool found_deceleration = false;
  for (const auto & point : points) {
    if (point.longitudinal_velocity_mps < 7.5F) {
      found_deceleration = true;
      break;
    }
  }
  EXPECT_TRUE(found_deceleration);
}

TEST(MrmStopVelocityPlannerTest, FillsZeroVelocityWhenEgoIsStopped)
{
  auto points = make_straight_trajectory(20, 1.0, 10.0F);
  const MrmStopVelocityPlanner planner(make_default_params());

  planner.apply(points, make_odometry(0.0), make_accel(0.5));

  for (const auto & point : points) {
    EXPECT_NEAR(point.longitudinal_velocity_mps, 0.0, 1e-3);
    EXPECT_NEAR(point.acceleration_mps2, 0.5F, 1e-3);
  }
}

TEST(MrmStopVelocityPlannerTest, FillsZeroVelocityWhenStopIsInfeasible)
{
  auto points = make_straight_trajectory_with_constraint_at(15, 1.0, 10.0F, 1);
  const MrmStopVelocityPlanner planner(make_default_params());

  planner.apply(points, make_odometry(10.0), make_accel(0.0));

  for (const auto & point : points) {
    EXPECT_NEAR(point.longitudinal_velocity_mps, 0.0, 1e-3);
    EXPECT_NEAR(point.acceleration_mps2, -6.0F, 1e-3);
  }
}

TEST(MrmStopVelocityPlannerTest, NoSpuriousZeroVelocityIslandAfterDensifyAndApply)
{
  auto points = make_straight_trajectory(40, 0.5, 10.0F);
  Odometry odom = make_odometry(2.0);
  odom.pose.pose.position.x = 2.25;
  const MrmStopVelocityPlanner planner(make_default_params());

  planner.apply(points, odom, make_accel(0.0));

  const size_t ego_idx =
    autoware::motion_utils::findNearestSegmentIndex(points, odom.pose.pose.position);
  ASSERT_GE(points.size(), ego_idx + 2);
  for (size_t i = 0; i <= std::min(ego_idx + 1, points.size() - 1); ++i) {
    EXPECT_NEAR(points.at(i).longitudinal_velocity_mps, 2.0F, 0.05F)
      << "point " << i << " should follow odom/prefix policy, not resample-to-zero";
  }
  EXPECT_GT(find_first_stopped_index(points), ego_idx);
}

TEST(MrmStopVelocityPlannerTest, AlignsPrefixThroughEgoWithOdomVelocity)
{
  // Simulates stale lane-speed at index 0; densify must not inject zeros before ego.
  auto points = make_straight_trajectory(30, 0.5, 9.17F);
  points.back().longitudinal_velocity_mps = 0.0F;

  Odometry odom = make_odometry(2.0);
  odom.pose.pose.position.x = 2.25;
  const MrmStopVelocityPlanner planner(make_default_params());

  planner.apply(points, odom, make_accel(0.0));

  const size_t ego_idx =
    autoware::motion_utils::findNearestSegmentIndex(points, odom.pose.pose.position);
  for (size_t i = 0; i <= ego_idx; ++i) {
    EXPECT_NEAR(points.at(i).longitudinal_velocity_mps, 2.0F, 0.05F)
      << "prefix index " << i << " should match odom velocity";
  }
  EXPECT_GT(points.at(ego_idx + 1).longitudinal_velocity_mps, 0.01F);
  EXPECT_GE(find_first_stopped_index(points), ego_idx + 1);
}

TEST(MrmStopVelocityPlannerTest, HoldsVelocityDuringBrakeDelayAndShiftsStopPoint)
{
  const double v0 = 10.0;
  auto points_no_delay = make_straight_trajectory(300, 0.5, static_cast<float>(v0));
  auto points_with_delay = points_no_delay;

  const MrmStopVelocityPlanner planner_no_delay(make_default_params());
  const MrmStopVelocityPlanner planner_with_delay(make_params_with_brake_delay(0.5));

  planner_no_delay.apply(points_no_delay, make_odometry(v0), make_accel(0.0));
  planner_with_delay.apply(points_with_delay, make_odometry(v0), make_accel(0.0));

  // Velocity is held at v0 while the brake command is in flight (hold length = v0 * 0.5 = 5 m).
  for (size_t i = 0; i < points_with_delay.size(); ++i) {
    if (arc_length_at_index(points_with_delay, i) > 4.0) {
      break;
    }
    EXPECT_NEAR(points_with_delay.at(i).longitudinal_velocity_mps, v0, 0.2);
  }

  const double stop_no_delay =
    arc_length_at_index(points_no_delay, find_first_stopped_index(points_no_delay));
  const double stop_with_delay =
    arc_length_at_index(points_with_delay, find_first_stopped_index(points_with_delay));
  EXPECT_NEAR(stop_with_delay - stop_no_delay, 5.0, 1.0);
}

TEST(MrmStopVelocityPlannerTest, RequiredStopDistanceGrowsByHoldLength)
{
  const MrmStopVelocityPlanner planner_no_delay(make_default_params());
  const MrmStopVelocityPlanner planner_with_delay(make_params_with_brake_delay(0.5));

  const double d0 = planner_no_delay.required_stop_distance(10.0, 0.0, -5.0, -3.0);
  const double d1 = planner_with_delay.required_stop_distance(10.0, 0.0, -5.0, -3.0);
  EXPECT_NEAR(d1 - d0, 5.0, 0.5);
}

TEST(MrmStopVelocityPlannerTest, BrakingA0KeepsDeceleratingDuringDelay)
{
  const double v0 = 10.0;
  const double a0 = -1.0;
  auto points = make_straight_trajectory(300, 0.5, static_cast<float>(v0));
  const MrmStopVelocityPlanner planner(make_params_with_brake_delay(0.5));
  planner.apply(points, make_odometry(v0), make_accel(a0));

  // At s ~= 3 m (t ~= 0.3 s, still inside the hold) the profile follows v0 + a0*t, not flat v0.
  const size_t idx = 6;  // 6 points * 0.5 m spacing
  const double t = arc_length_at_index(points, idx) / v0;
  EXPECT_NEAR(points.at(idx).longitudinal_velocity_mps, v0 + a0 * t, 0.2);
  EXPECT_LT(points.at(idx).longitudinal_velocity_mps, static_cast<float>(v0));
}

TEST(MrmStopVelocityPlannerTest, PositiveA0NeverProducesAcceleratingReference)
{
  const double v0 = 10.0;
  auto points_pos_a0 = make_straight_trajectory(300, 0.5, static_cast<float>(v0));
  auto points_zero_a0 = points_pos_a0;
  const MrmStopVelocityPlanner planner(make_params_with_brake_delay(0.5));

  planner.apply(points_pos_a0, make_odometry(v0), make_accel(1.5));
  planner.apply(points_zero_a0, make_odometry(v0), make_accel(0.0));

  float max_velocity = 0.0F;
  for (const auto & p : points_pos_a0) {
    max_velocity = std::max(max_velocity, p.longitudinal_velocity_mps);
  }
  EXPECT_LE(max_velocity, static_cast<float>(v0) + 0.01F);

  // Positive a0 is clamped to zero during the delay, so the profile matches the a0 = 0 case.
  EXPECT_EQ(find_first_stopped_index(points_pos_a0), find_first_stopped_index(points_zero_a0));
}

TEST(MrmStopVelocityPlannerTest, RelaxationAccountsForBrakeDelayDistance)
{
  const double v0 = 10.0;
  // Constraint at 22 m: feasible at target limits without delay (required ~19.6 m),
  // infeasible with a 0.5 s delay (+5 m hold), so relaxation must kick in.
  const auto points =
    make_straight_trajectory_with_constraint_at(60, 0.5, static_cast<float>(v0), 44);

  const MrmStopVelocityPlanner planner_no_delay(make_default_params());
  const MrmStopVelocityPlanner planner_with_delay(make_params_with_brake_delay(0.5));

  const auto limits_no_delay = planner_no_delay.select_profile_limits(points, 0, 44, v0, 0.0);
  const auto limits_with_delay = planner_with_delay.select_profile_limits(points, 0, 44, v0, 0.0);

  EXPECT_DOUBLE_EQ(limits_no_delay.decel, -3.0);
  EXPECT_LT(limits_with_delay.decel, -3.0);
}

TEST(MrmStopVelocityPlannerTest, StopsGracefullyWhenVelocityReachesZeroDuringHold)
{
  const double v0 = 0.5;
  auto points = make_straight_trajectory(100, 0.5, static_cast<float>(v0));
  const MrmStopVelocityPlanner planner(make_params_with_brake_delay(1.0));
  planner.apply(points, make_odometry(v0), make_accel(-3.0));

  // v0 = 0.5 with a0 = -3 reaches zero in ~0.17 s (~0.04 m), well within the hold.
  const size_t stop_idx = find_first_stopped_index(points);
  EXPECT_LE(arc_length_at_index(points, stop_idx), 1.0);
  for (size_t i = stop_idx; i < points.size(); ++i) {
    EXPECT_FLOAT_EQ(points.at(i).longitudinal_velocity_mps, 0.0F);
  }
}

}  // namespace autoware::in_lane_mrm_planner
