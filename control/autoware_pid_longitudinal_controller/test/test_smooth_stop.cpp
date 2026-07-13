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

#include "autoware/pid_longitudinal_controller/smooth_stop.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include <chrono>

TEST(TestSmoothStop, calculate_stopping_acceleration)
{
  using ::autoware::motion::control::pid_longitudinal_controller::DebugValues;
  using ::autoware::motion::control::pid_longitudinal_controller::SmoothStop;
  using rclcpp::Time;

  const double max_strong_acc = -0.5;
  const double min_strong_acc = -1.0;
  const double weak_acc = -0.3;
  const double weak_stop_acc = -0.8;
  const double strong_stop_acc = -3.4;
  const double max_fast_vel = 0.5;
  const double min_running_vel = 0.01;
  const double min_running_acc = 0.01;
  const double weak_stop_time = 0.8;
  const double weak_stop_dist = -0.3;
  const double strong_stop_dist = -0.5;

  const double delay_time = 0.17;

  SmoothStop ss{SmoothStop::Params{
    max_strong_acc, min_strong_acc, weak_acc, weak_stop_acc, strong_stop_acc, max_fast_vel,
    min_running_vel, min_running_acc, weak_stop_time, weak_stop_dist, strong_stop_dist}};
  DebugValues debug_values;

  double vel_in_target;
  double stop_dist;
  double current_vel;
  double current_acc = 0.0;
  Time now(0, 0, RCL_ROS_TIME);
  // calculate() reads the current velocity, acceleration and time from the latest recorded
  // motion sample, so record one before every call below. A zero time window keeps only that
  // latest sample, which is enough to exercise every branch under test.
  const double vel_hist_time_window = 0.0;
  double accel;

  // strong stop when the stop distance is below the threshold
  vel_in_target = 5.0;
  stop_dist = strong_stop_dist - 0.1;
  current_vel = 2.0;
  ss.init(vel_in_target, stop_dist, now);
  ss.recordMotion(now, current_vel, current_acc, vel_hist_time_window);
  accel = ss.calculate(stop_dist, delay_time, debug_values);
  EXPECT_EQ(accel, strong_stop_acc);
  EXPECT_EQ(debug_values.getValue(DebugValues::TYPE::SMOOTH_STOP_MODE), 3);

  // weak stop when the stop distance is below the threshold (but not bellow the strong_stop_dist)
  stop_dist = weak_stop_dist - 0.1;
  current_vel = 2.0;
  ss.init(vel_in_target, stop_dist, now);
  ss.recordMotion(now, current_vel, current_acc, vel_hist_time_window);
  accel = ss.calculate(stop_dist, delay_time, debug_values);
  EXPECT_EQ(accel, weak_stop_acc);
  EXPECT_EQ(debug_values.getValue(DebugValues::TYPE::SMOOTH_STOP_MODE), 2);

  // if not running, weak accel for 0.5 seconds after the previous init or previous weak_acc
  stop_dist = 0.0;
  current_vel = 0.0;
  ss.recordMotion(now, current_vel, current_acc, vel_hist_time_window);
  accel = ss.calculate(stop_dist, delay_time, debug_values);
  EXPECT_EQ(accel, weak_acc);
  EXPECT_EQ(debug_values.getValue(DebugValues::TYPE::SMOOTH_STOP_MODE), 1);
  now = now + std::chrono::milliseconds(250);
  ss.recordMotion(now, current_vel, current_acc, vel_hist_time_window);
  accel = ss.calculate(stop_dist, delay_time, debug_values);
  EXPECT_EQ(accel, weak_acc);
  EXPECT_EQ(debug_values.getValue(DebugValues::TYPE::SMOOTH_STOP_MODE), 1);
  now = now + std::chrono::milliseconds(500);
  ss.recordMotion(now, current_vel, current_acc, vel_hist_time_window);
  accel = ss.calculate(stop_dist, delay_time, debug_values);
  EXPECT_NE(accel, weak_acc);
  EXPECT_NE(debug_values.getValue(DebugValues::TYPE::SMOOTH_STOP_MODE), 1);

  // strong stop when the car is not running (and is at least 0.5seconds after initialization)
  accel = ss.calculate(stop_dist, delay_time, debug_values);
  EXPECT_EQ(accel, strong_stop_acc);
  EXPECT_EQ(debug_values.getValue(DebugValues::TYPE::SMOOTH_STOP_MODE), 3);

  // accel between min/max_strong_acc when the car is running:
  // not predicted to exceed the stop line and is predicted to stop after weak_stop_time + delay
  stop_dist = 1.0;
  current_vel = 1.0;
  vel_in_target = 1.0;
  ss.init(vel_in_target, stop_dist, now);
  ss.recordMotion(now, current_vel, current_acc, vel_hist_time_window);
  accel = ss.calculate(stop_dist, delay_time, debug_values);
  EXPECT_EQ(accel, max_strong_acc);
  EXPECT_EQ(debug_values.getValue(DebugValues::TYPE::SMOOTH_STOP_MODE), 0);

  vel_in_target = std::sqrt(2.0);
  ss.init(vel_in_target, stop_dist, now);
  ss.recordMotion(now, current_vel, current_acc, vel_hist_time_window);
  accel = ss.calculate(stop_dist, delay_time, debug_values);
  EXPECT_EQ(accel, min_strong_acc);
  EXPECT_EQ(debug_values.getValue(DebugValues::TYPE::SMOOTH_STOP_MODE), 0);

  for (double vel_in_target = 1.1; vel_in_target < std::sqrt(2.0); vel_in_target += 0.1) {
    ss.init(vel_in_target, stop_dist, now);
    ss.recordMotion(now, current_vel, current_acc, vel_hist_time_window);
    accel = ss.calculate(stop_dist, delay_time, debug_values);
    EXPECT_GT(accel, min_strong_acc);
    EXPECT_LT(accel, max_strong_acc);
  }
}
