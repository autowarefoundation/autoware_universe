// Copyright 2021 Tier IV, Inc.
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

#include "autoware/interpolation/spherical_linear_interpolation.hpp"
#include "autoware/pid_longitudinal_controller/longitudinal_controller_utils.hpp"
#include "autoware/trajectory/interpolator/linear.hpp"
#include "autoware/trajectory/utils/find_nearest.hpp"
#include "gtest/gtest.h"

#include <tf2/LinearMath/Quaternion.hpp>

#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace longitudinal_utils =
  ::autoware::motion::control::pid_longitudinal_controller::longitudinal_utils;
namespace experimental_trajectory = ::autoware::experimental::trajectory;

using TrajectoryExperimental = longitudinal_utils::TrajectoryExperimental;

namespace
{

autoware_planning_msgs::msg::TrajectoryPoint makeTrajectoryPoint(
  const double x, const double y, const double z, const double velocity = 0.0,
  const double acceleration = 0.0)
{
  autoware_planning_msgs::msg::TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = z;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = velocity;
  point.acceleration_mps2 = acceleration;
  return point;
}

TrajectoryExperimental makeContinuousTrajectory(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points)
{
  const auto trajectory =
    TrajectoryExperimental::Builder{}
      .template set_xy_interpolator<experimental_trajectory::interpolator::Linear>()
      .template set_z_interpolator<experimental_trajectory::interpolator::Linear>()
      .build(points);
  if (!trajectory) {
    throw std::runtime_error("failed to build experimental trajectory for test");
  }
  return trajectory.value();
}

}  // namespace

TEST(TestLongitudinalControllerUtils, calcStopDistance)
{
  using geometry_msgs::msg::Pose;

  Pose current_pose;
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.position.z = 0.0;
  current_pose.orientation.w = 1.0;

  constexpr double max_dist = 10.0;
  constexpr double max_yaw = 0.7;

  const auto zero_vel_trajectory = makeContinuousTrajectory(
    {makeTrajectoryPoint(0.0, 0.0, 0.0, 0.0), makeTrajectoryPoint(1.0, 0.0, 0.0, 0.0)});
  EXPECT_NEAR(
    longitudinal_utils::calcStopDistance(current_pose, zero_vel_trajectory, max_dist, max_yaw), 0.0,
    1e-2);

  const auto non_stopping_trajectory = makeContinuousTrajectory(
    {makeTrajectoryPoint(0.0, 0.0, 0.0, 1.0), makeTrajectoryPoint(1.0, 0.0, 0.0, 1.0),
     makeTrajectoryPoint(2.0, 0.0, 0.0, 1.0)});
  EXPECT_NEAR(
    longitudinal_utils::calcStopDistance(current_pose, non_stopping_trajectory, max_dist, max_yaw),
    2.0, 1e-2);

  current_pose.position.x = 3.0;
  EXPECT_NEAR(
    longitudinal_utils::calcStopDistance(current_pose, non_stopping_trajectory, max_dist, max_yaw),
    -1.0, 1e-2);

  current_pose.position.x = 0.0;

  const auto stopping_trajectory = makeContinuousTrajectory(
    {makeTrajectoryPoint(0.0, 0.0, 0.0, 1.0), makeTrajectoryPoint(1.0, 0.0, 0.0, 1.0),
     makeTrajectoryPoint(2.0, 0.0, 0.0, 1.0), makeTrajectoryPoint(3.0, 0.0, 0.0, 0.0),
     makeTrajectoryPoint(4.0, 0.0, 0.0, 1.0), makeTrajectoryPoint(5.0, 0.0, 0.0, 0.0)});
  EXPECT_NEAR(
    longitudinal_utils::calcStopDistance(current_pose, stopping_trajectory, max_dist, max_yaw), 3.0,
    1e-2);

  // Regression case: a small goal overrun should still report a negative stop distance.
  current_pose.position.x = 6.5;
  EXPECT_LT(
    longitudinal_utils::calcStopDistance(current_pose, stopping_trajectory, max_dist, max_yaw),
    0.0);

  current_pose.position.x = 9.0;
  EXPECT_LT(
    longitudinal_utils::calcStopDistance(current_pose, stopping_trajectory, max_dist, max_yaw),
    0.0);

  current_pose.position.x = 12.0;
  const double far_overrun_stop_dist =
    longitudinal_utils::calcStopDistance(current_pose, stopping_trajectory, max_dist, max_yaw);
  EXPECT_NEAR(far_overrun_stop_dist, -9.0, 1e-2);
}

TEST(TestLongitudinalControllerUtils, calcStopDistanceRejectsMisalignedOrFarOverrun)
{
  using geometry_msgs::msg::Pose;

  Pose current_pose;
  current_pose.position.x = 6.5;
  current_pose.position.y = 0.0;
  current_pose.position.z = 0.0;
  current_pose.orientation.w = 1.0;

  constexpr double max_dist = 3.0;
  constexpr double max_yaw = 0.7;

  const auto stopping_trajectory = makeContinuousTrajectory(
    {makeTrajectoryPoint(0.0, 0.0, 0.0, 1.0), makeTrajectoryPoint(1.0, 0.0, 0.0, 1.0),
     makeTrajectoryPoint(2.0, 0.0, 0.0, 1.0), makeTrajectoryPoint(3.0, 0.0, 0.0, 0.0),
     makeTrajectoryPoint(4.0, 0.0, 0.0, 1.0), makeTrajectoryPoint(5.0, 0.0, 0.0, 0.0)});

  tf2::Quaternion misaligned_quaternion;
  misaligned_quaternion.setRPY(0.0, 0.0, 1.0);
  current_pose.orientation = tf2::toMsg(misaligned_quaternion);
  // Falls back to position-only search (soft-constraints behavior)
  EXPECT_NEAR(
    longitudinal_utils::calcStopDistance(current_pose, stopping_trajectory, max_dist, max_yaw),
    -3.5, 1e-2);

  current_pose.orientation.w = 1.0;
  current_pose.orientation.x = 0.0;
  current_pose.orientation.y = 0.0;
  current_pose.orientation.z = 0.0;
  current_pose.position.y = 3.5;
  // Falls back to position-only search (soft-constraints behavior)
  EXPECT_NEAR(
    longitudinal_utils::calcStopDistance(current_pose, stopping_trajectory, max_dist, max_yaw),
    -3.5, 1e-2);
}

TEST(TestLongitudinalControllerUtils, getPitchByPose)
{
  tf2::Quaternion quaternion_tf;
  quaternion_tf.setRPY(0.0, 0.0, 0.0);
  EXPECT_EQ(longitudinal_utils::getPitchByPose(tf2::toMsg(quaternion_tf)), 0.0);
  quaternion_tf.setRPY(0.0, 1.0, 0.0);
  EXPECT_EQ(longitudinal_utils::getPitchByPose(tf2::toMsg(quaternion_tf)), 1.0);
}

TEST(TestLongitudinalControllerUtils, getPitchByTraj)
{
  const double wheel_base = 0.9;
  const auto trajectory = makeContinuousTrajectory(
    {makeTrajectoryPoint(0.0, 0.0, 0.0), makeTrajectoryPoint(0.6, 0.0, 0.8),
     makeTrajectoryPoint(1.2, 0.0, 0.0), makeTrajectoryPoint(1.8, 0.0, 0.8)});

  EXPECT_NEAR(
    longitudinal_utils::getPitchByTraj(trajectory, 0.0, wheel_base), std::atan2(0.8, 0.6), 1e-6);
  EXPECT_NEAR(
    longitudinal_utils::getPitchByTraj(trajectory, 1.0, wheel_base), std::atan2(-0.8, 0.6), 1e-6);
  EXPECT_NEAR(
    longitudinal_utils::getPitchByTraj(trajectory, 2.0, wheel_base), std::atan2(0.8, 0.6), 1e-6);
  EXPECT_NEAR(
    longitudinal_utils::getPitchByTraj(trajectory, 3.0, wheel_base), std::atan2(0.8, 0.6), 1e-6);
}

TEST(TestLongitudinalControllerUtils, calcPoseAfterTimeDelay)
{
  using geometry_msgs::msg::Pose;
  const double abs_err = 1e-7;
  Pose current_pose;
  current_pose.position.x = 0.0;
  current_pose.position.y = 0.0;
  current_pose.position.z = 0.0;
  tf2::Quaternion quaternion_tf;
  quaternion_tf.setRPY(0.0, 0.0, 0.0);
  current_pose.orientation = tf2::toMsg(quaternion_tf);

  double delay_time = 0.0;
  double current_vel = 0.0;
  double current_acc = 0.0;
  Pose delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  delay_time = 1.0;
  current_vel = 0.0;
  current_acc = 0.0;
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  delay_time = 0.0;
  current_vel = 1.0;
  current_acc = 0.0;
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  delay_time = 1.0;
  current_vel = 1.0;
  current_acc = 0.0;
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x + current_vel * delay_time, abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  delay_time = 1.0;
  current_vel = 1.0;
  current_acc = 1.0;
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(
    delayed_pose.position.x,
    current_pose.position.x + current_vel * delay_time +
      0.5 * current_acc * delay_time * delay_time,
    abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  quaternion_tf.setRPY(0.0, 0.0, M_PI);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(
    delayed_pose.position.x,
    current_pose.position.x - current_vel * delay_time -
      0.5 * current_acc * delay_time * delay_time,
    abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  quaternion_tf.setRPY(0.0, 0.0, M_PI_2);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(
    delayed_pose.position.y,
    current_pose.position.y + current_vel * delay_time +
      0.5 * current_acc * delay_time * delay_time,
    abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  quaternion_tf.setRPY(0.0, 0.0, -M_PI_2);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(delayed_pose.position.x, current_pose.position.x, abs_err);
  EXPECT_NEAR(
    delayed_pose.position.y,
    current_pose.position.y - current_vel * delay_time -
      0.5 * current_acc * delay_time * delay_time,
    abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  quaternion_tf.setRPY(0.0, M_PI_4, 0.0);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(
    delayed_pose.position.x,
    current_pose.position.x + current_vel * delay_time +
      0.5 * current_acc * delay_time * delay_time,
    abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);

  quaternion_tf.setRPY(M_PI_2, 0.0, 0.0);
  current_pose.orientation = tf2::toMsg(quaternion_tf);
  delayed_pose =
    longitudinal_utils::calcPoseAfterTimeDelay(current_pose, delay_time, current_vel, current_acc);
  EXPECT_NEAR(
    delayed_pose.position.x,
    current_pose.position.x + current_vel * delay_time +
      0.5 * current_acc * delay_time * delay_time,
    abs_err);
  EXPECT_NEAR(delayed_pose.position.y, current_pose.position.y, abs_err);
  EXPECT_NEAR(delayed_pose.position.z, current_pose.position.z, abs_err);
}

TEST(TestLongitudinalControllerUtils, lerpOrientation)
{
  geometry_msgs::msg::Quaternion result;
  tf2::Quaternion o_from;
  tf2::Quaternion o_to;
  tf2::Quaternion o_result;
  double roll;
  double pitch;
  double yaw;
  double ratio;

  o_from.setRPY(0.0, 0.0, 0.0);
  o_to.setRPY(M_PI_4, M_PI_4, M_PI_4);

  ratio = 0.0;
  result = autoware::interpolation::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, 0.0);
  EXPECT_DOUBLE_EQ(pitch, 0.0);
  EXPECT_DOUBLE_EQ(yaw, 0.0);

  ratio = 1.0;
  result = autoware::interpolation::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, M_PI_4);
  EXPECT_DOUBLE_EQ(pitch, M_PI_4);
  EXPECT_DOUBLE_EQ(yaw, M_PI_4);

  ratio = 0.5;
  o_to.setRPY(M_PI_4, 0.0, 0.0);
  result = autoware::interpolation::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, M_PI_4 / 2);
  EXPECT_DOUBLE_EQ(pitch, 0.0);
  EXPECT_DOUBLE_EQ(yaw, 0.0);

  o_to.setRPY(0.0, M_PI_4, 0.0);
  result = autoware::interpolation::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, 0.0);
  EXPECT_DOUBLE_EQ(pitch, M_PI_4 / 2);
  EXPECT_DOUBLE_EQ(yaw, 0.0);

  o_to.setRPY(0.0, 0.0, M_PI_4);
  result = autoware::interpolation::lerpOrientation(tf2::toMsg(o_from), tf2::toMsg(o_to), ratio);
  tf2::convert(result, o_result);
  tf2::Matrix3x3(o_result).getRPY(roll, pitch, yaw);
  EXPECT_DOUBLE_EQ(roll, 0.0);
  EXPECT_DOUBLE_EQ(pitch, 0.0);
  EXPECT_DOUBLE_EQ(yaw, M_PI_4 / 2);
}

TEST(TestLongitudinalControllerUtils, applyDiffLimitFilter)
{
  double dt = 1.0;
  double max_val = 0.0;
  double min_val = 0.0;
  double prev_val = 0.0;

  double input_val = 10.0;
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), 0.0);

  max_val = 1.0;
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), 1.0);

  input_val = -10.0;
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), 0.0);

  min_val = -1.0;
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), -1.0);

  dt = 5.0;
  input_val = 10.0;
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), 5.0);
  input_val = -10.0;
  EXPECT_DOUBLE_EQ(
    longitudinal_utils::applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val), -5.0);

  dt = 1.0;
  input_val = 100.0;
  for (double prev = 0.0; prev < 100.0; ++prev) {
    const double new_val =
      longitudinal_utils::applyDiffLimitFilter(input_val, prev, dt, max_val, min_val);
    EXPECT_DOUBLE_EQ(new_val, prev + max_val);
    prev = new_val;
  }
}
