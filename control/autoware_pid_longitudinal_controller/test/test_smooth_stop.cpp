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
#include "rclcpp/time.hpp"

#include <chrono>
#include <cmath>

namespace
{
using ::autoware::motion::control::pid_longitudinal_controller::SmoothStop;
using rclcpp::Time;

constexpr double max_strong_acc = -0.5;
constexpr double min_strong_acc = -1.0;
constexpr double weak_acc = -0.3;
constexpr double weak_stop_acc = -0.8;
constexpr double strong_stop_acc = -3.4;
constexpr double min_fast_vel = 0.5;
constexpr double min_running_vel = 0.01;
constexpr double min_running_acc = 0.01;
constexpr double weak_stop_time = 0.8;
constexpr double weak_stop_dist = -0.3;
constexpr double strong_stop_dist = -0.5;
constexpr double delay_time = 0.17;

SmoothStop::Params makeDefaultParams()
{
  return SmoothStop::Params{max_strong_acc,  min_strong_acc, weak_acc,        weak_stop_acc,
                            strong_stop_acc, min_fast_vel,   min_running_vel, min_running_acc,
                            weak_stop_time,  weak_stop_dist, strong_stop_dist};
}

// calculate() reads the current velocity, acceleration and time from the latest sample
// recorded via recordMotion(), so every test below must call it at least once beforehand.
// A zero time window keeps only that latest sample.
constexpr double vel_hist_time_window = 0.0;
}  // namespace

TEST(SmoothStopTest, ReturnsStrongStopAccelerationWhenStopDistanceBelowStrongStopThreshold)
{
  // Arrange
  SmoothStop smooth_stop{makeDefaultParams()};
  const Time now(0, 0, RCL_ROS_TIME);
  const double stop_dist = strong_stop_dist - 0.1;
  smooth_stop.init(/*pred_vel_in_target=*/5.0, stop_dist, now);
  smooth_stop.recordMotion(now, /*vel=*/2.0, /*acc=*/0.0, vel_hist_time_window);

  // Act
  const SmoothStop::Result result = smooth_stop.calculate(stop_dist, delay_time);

  // Assert
  EXPECT_DOUBLE_EQ(result.acc, strong_stop_acc);
  EXPECT_EQ(result.mode, SmoothStop::Mode::STRONG_STOP);
}

TEST(SmoothStopTest, ReturnsWeakStopAccelerationWhenStopDistanceBelowWeakStopThreshold)
{
  // Arrange
  SmoothStop smooth_stop{makeDefaultParams()};
  const Time now(0, 0, RCL_ROS_TIME);
  const double stop_dist = weak_stop_dist - 0.1;  // still above strong_stop_dist
  smooth_stop.init(/*pred_vel_in_target=*/5.0, stop_dist, now);
  smooth_stop.recordMotion(now, /*vel=*/2.0, /*acc=*/0.0, vel_hist_time_window);

  // Act
  const SmoothStop::Result result = smooth_stop.calculate(stop_dist, delay_time);

  // Assert
  EXPECT_DOUBLE_EQ(result.acc, weak_stop_acc);
  EXPECT_EQ(result.mode, SmoothStop::Mode::WEAK_STOP);
}

TEST(SmoothStopTest, ReturnsWeakAccelerationWhenStoppedRightAfterInit)
{
  // Arrange
  SmoothStop smooth_stop{makeDefaultParams()};
  const Time now(0, 0, RCL_ROS_TIME);
  const double stop_dist = 0.0;
  smooth_stop.init(/*pred_vel_in_target=*/5.0, stop_dist, now);
  smooth_stop.recordMotion(now, /*vel=*/0.0, /*acc=*/0.0, vel_hist_time_window);

  // Act
  const SmoothStop::Result result = smooth_stop.calculate(stop_dist, delay_time);

  // Assert
  EXPECT_DOUBLE_EQ(result.acc, weak_acc);
  EXPECT_EQ(result.mode, SmoothStop::Mode::WEAK);
}

TEST(SmoothStopTest, ReturnsWeakAccelerationWhileStoppedWithinHalfSecondOfInit)
{
  // Arrange
  SmoothStop smooth_stop{makeDefaultParams()};
  const Time init_time(0, 0, RCL_ROS_TIME);
  const double stop_dist = 0.0;
  smooth_stop.init(/*pred_vel_in_target=*/5.0, stop_dist, init_time);
  const Time now = init_time + std::chrono::milliseconds(250);
  smooth_stop.recordMotion(now, /*vel=*/0.0, /*acc=*/0.0, vel_hist_time_window);

  // Act
  const SmoothStop::Result result = smooth_stop.calculate(stop_dist, delay_time);

  // Assert
  EXPECT_DOUBLE_EQ(result.acc, weak_acc);
  EXPECT_EQ(result.mode, SmoothStop::Mode::WEAK);
}

TEST(SmoothStopTest, ReturnsStrongStopAccelerationWhenStoppedForMoreThanHalfSecondSinceInit)
{
  // Arrange
  SmoothStop smooth_stop{makeDefaultParams()};
  const Time init_time(0, 0, RCL_ROS_TIME);
  const double stop_dist = 0.0;
  smooth_stop.init(/*pred_vel_in_target=*/5.0, stop_dist, init_time);
  const Time now = init_time + std::chrono::milliseconds(750);  // exceeds the 0.5s weak window
  smooth_stop.recordMotion(now, /*vel=*/0.0, /*acc=*/0.0, vel_hist_time_window);

  // Act
  const SmoothStop::Result result = smooth_stop.calculate(stop_dist, delay_time);

  // Assert
  EXPECT_DOUBLE_EQ(result.acc, strong_stop_acc);
  EXPECT_EQ(result.mode, SmoothStop::Mode::STRONG_STOP);
}

TEST(SmoothStopTest, ReturnsMaxStrongAccelerationWhenPredictedStopMatchesUpperLimit)
{
  // Arrange: pred_vel_in_target^2 / (2 * stop_dist) == |max_strong_acc|
  SmoothStop smooth_stop{makeDefaultParams()};
  const Time now(0, 0, RCL_ROS_TIME);
  const double stop_dist = 1.0;
  const double vel_in_target = 1.0;
  smooth_stop.init(vel_in_target, stop_dist, now);
  smooth_stop.recordMotion(now, /*vel=*/1.0, /*acc=*/0.0, vel_hist_time_window);

  // Act
  const SmoothStop::Result result = smooth_stop.calculate(stop_dist, delay_time);

  // Assert
  EXPECT_DOUBLE_EQ(result.acc, max_strong_acc);
  EXPECT_EQ(result.mode, SmoothStop::Mode::STRONG);
}

TEST(SmoothStopTest, ReturnsMinStrongAccelerationWhenPredictedStopExceedsLowerLimit)
{
  // Arrange: pred_vel_in_target^2 / (2 * stop_dist) == |min_strong_acc|
  SmoothStop smooth_stop{makeDefaultParams()};
  const Time now(0, 0, RCL_ROS_TIME);
  const double stop_dist = 1.0;
  const double vel_in_target = std::sqrt(2.0);
  smooth_stop.init(vel_in_target, stop_dist, now);
  smooth_stop.recordMotion(now, /*vel=*/1.0, /*acc=*/0.0, vel_hist_time_window);

  // Act
  const SmoothStop::Result result = smooth_stop.calculate(stop_dist, delay_time);

  // Assert
  EXPECT_DOUBLE_EQ(result.acc, min_strong_acc);
  EXPECT_EQ(result.mode, SmoothStop::Mode::STRONG);
}

TEST(
  SmoothStopTest, ReturnsStrongAccelerationInterpolatedBetweenLimitsForIntermediateTargetVelocity)
{
  // Arrange: pick a target velocity strictly between the two limit cases above, so the
  // resulting acceleration should fall strictly between min_strong_acc and max_strong_acc
  // without being clamped to either.
  SmoothStop smooth_stop{makeDefaultParams()};
  const Time now(0, 0, RCL_ROS_TIME);
  const double stop_dist = 1.0;
  const double vel_in_target = 1.2;
  smooth_stop.init(vel_in_target, stop_dist, now);
  smooth_stop.recordMotion(now, /*vel=*/1.0, /*acc=*/0.0, vel_hist_time_window);

  // Act
  const SmoothStop::Result result = smooth_stop.calculate(stop_dist, delay_time);

  // Assert
  EXPECT_GT(result.acc, min_strong_acc);
  EXPECT_LT(result.acc, max_strong_acc);
  EXPECT_EQ(result.mode, SmoothStop::Mode::STRONG);
}

TEST(SmoothStopTest, ReturnsStrongAccelerationWhenRunningAndPredictedTimeToStopExceedsWeakStopTime)
{
  // Arrange: two motion samples 1.0s apart, decelerating from 2.0 to 1.0 m/s, fit a line
  // predicting the car reaches 0 m/s in 1.0s, which exceeds weak_stop_time + delay_time
  // (0.8 + 0.17 = 0.97s).
  SmoothStop smooth_stop{makeDefaultParams()};
  const Time init_time(0, 0, RCL_ROS_TIME);
  const double stop_dist = 1.0;
  const double vel_in_target =
    1.0;  // matches the max_strong_acc case, m_strong_acc == max_strong_acc
  smooth_stop.init(vel_in_target, stop_dist, init_time);
  const double time_window = 1.1;  // wide enough to keep both samples 1.0s apart
  smooth_stop.recordMotion(init_time, /*vel=*/2.0, /*acc=*/0.0, time_window);
  const Time now = init_time + std::chrono::seconds(1);
  smooth_stop.recordMotion(now, /*vel=*/1.0, /*acc=*/0.0, time_window);

  // Act
  const SmoothStop::Result result = smooth_stop.calculate(stop_dist, delay_time);

  // Assert
  EXPECT_DOUBLE_EQ(result.acc, max_strong_acc);
  EXPECT_EQ(result.mode, SmoothStop::Mode::STRONG);
}

TEST(SmoothStopTest, ReturnsWeakAccelerationWhenRunningAndPredictedTimeToStopWithinWeakStopTime)
{
  // Arrange: two motion samples 0.5s apart, decelerating from 2.0 to 1.0 m/s, fit a line
  // predicting the car reaches 0 m/s in 0.5s, which is within weak_stop_time + delay_time
  // (0.8 + 0.17 = 0.97s), so braking can ease off even though the car is still running.
  SmoothStop smooth_stop{makeDefaultParams()};
  const Time init_time(0, 0, RCL_ROS_TIME);
  const double stop_dist = 1.0;
  smooth_stop.init(/*pred_vel_in_target=*/1.0, stop_dist, init_time);
  const double time_window = 0.6;  // wide enough to keep both samples 0.5s apart
  smooth_stop.recordMotion(init_time, /*vel=*/2.0, /*acc=*/0.0, time_window);
  const Time now = init_time + std::chrono::milliseconds(500);
  smooth_stop.recordMotion(now, /*vel=*/1.0, /*acc=*/0.0, time_window);

  // Act
  const SmoothStop::Result result = smooth_stop.calculate(stop_dist, delay_time);

  // Assert
  EXPECT_DOUBLE_EQ(result.acc, weak_acc);
  EXPECT_EQ(result.mode, SmoothStop::Mode::WEAK);
}
