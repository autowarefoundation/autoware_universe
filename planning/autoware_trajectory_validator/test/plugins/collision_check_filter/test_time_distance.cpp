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

#include "../../../src/filters/safety/collision_check_filter/trajectory_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::trajectory::time_distance
{
namespace
{
constexpr double kDefaultTimeResolution = GlobalParams{}.time_resolution;
}  // namespace

class TimeDistanceTest : public ::testing::Test
{
protected:
  geometry_msgs::msg::Twist create_twist(double vx, double vy)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = vx;
    twist.linear.y = vy;
    return twist;
  }
};

TEST_F(TimeDistanceTest, ZeroInitialVelocity)
{
  auto twist = create_twist(0.0, 0.0);
  auto [times, distances] =
    compute_motion_profile_1d(twist, 1.0, 1.0, 0.0, 5.0, kDefaultTimeResolution);

  ASSERT_EQ(times.size(), 1u);
  ASSERT_EQ(distances.size(), 1u);
  EXPECT_DOUBLE_EQ(times[0], 0.0);
  EXPECT_DOUBLE_EQ(distances[0], 0.0);
}

TEST_F(TimeDistanceTest, ConstantVelocity)
{
  auto twist = create_twist(10.0, 0.0);  // 合成初速: 10.0
  double lag = 1.0;
  double accel = 0.0;
  double end_time = 3.05;

  auto [times, distances] =
    compute_motion_profile_1d(twist, lag, accel, 0.0, end_time, kDefaultTimeResolution);

  EXPECT_EQ(times.front(), 0.0);
  EXPECT_NEAR(times.back(), end_time, 1e-6);
  for (size_t i = 1; i < times.size() - 1; ++i) {
    double t = times[i];
    EXPECT_NEAR(t, i * kDefaultTimeResolution, 1e-6);
    EXPECT_NEAR(distances[i], 10.0 * t, 1e-6);
  }
}

TEST_F(TimeDistanceTest, Acceleration)
{
  auto twist = create_twist(3.0, 4.0);
  double lag = 1.0;
  double accel = 2.0;
  double end_time = 2.05;

  auto [times, distances] =
    compute_motion_profile_1d(twist, lag, accel, 0.0, end_time, kDefaultTimeResolution);

  ASSERT_FALSE(times.empty());

  for (size_t i = 0; i < times.size(); ++i) {
    double t = times[i];
    if (t < lag) {
      EXPECT_NEAR(distances[i], 5.0 * t, 1e-6);
    } else {
      double time_after_lag = t - lag;
      double expected_d =
        (5.0 * lag) + (5.0 * time_after_lag) + (0.5 * accel * time_after_lag * time_after_lag);
      EXPECT_NEAR(distances[i], expected_d, 1e-6);
    }
  }
}

TEST_F(TimeDistanceTest, DecelerationAndStop)
{
  auto twist = create_twist(10.0, 0.0);
  double lag = 1.0;
  double accel = -5.0;
  double end_time = 5.0;

  auto [times, distances] =
    compute_motion_profile_1d(twist, lag, accel, 0.0, end_time, kDefaultTimeResolution);

  double expected_stop_time = 3.0;

  double lag_distance = 10.0 * 1.0;
  double time_to_stop = 10.0 / 5.0;
  double expected_stop_distance =
    lag_distance + (10.0 * time_to_stop) + (0.5 * -5.0 * time_to_stop * time_to_stop);

  auto is_stop_time_in_list = std::find_if(times.begin(), times.end(), [&](double t) {
                                return std::abs(t - expected_stop_time) < 1e-6;
                              }) != times.end();
  EXPECT_TRUE(is_stop_time_in_list);
  EXPECT_NEAR(distances.back(), expected_stop_distance, 1e-6);
}

TEST_F(TimeDistanceTest, LagLongerThanMaxTime)
{
  auto twist = create_twist(5.0, 0.0);
  double lag = 5.0;
  double accel = -10.0;
  double end_time = 2.05;

  auto [times, distances] =
    compute_motion_profile_1d(twist, lag, accel, 0.0, end_time, kDefaultTimeResolution);

  for (size_t i = 0; i < times.size(); ++i) {
    double t = times[i];
    EXPECT_NEAR(distances[i], 5.0 * t, 1e-6);
  }
}

TEST_F(TimeDistanceTest, SamplesStayWithinRangeAndMonotonicWhenStopTimeExceedsEndTime)
{
  auto twist = create_twist(10.0, 0.0);
  const double lag = 1.0;
  const double accel = -10.0 / 1.97;
  const double end_time = 2.95;
  const double expected_stop_time = 2.97;

  auto [times, distances] =
    compute_motion_profile_1d(twist, lag, accel, 0.0, end_time, kDefaultTimeResolution);

  ASSERT_FALSE(times.empty());
  ASSERT_EQ(times.size(), distances.size());
  EXPECT_NEAR(times.front(), 0.0, 1e-6);
  EXPECT_NEAR(times.back(), end_time, 1e-6);

  for (size_t i = 0; i < times.size(); ++i) {
    EXPECT_GE(times[i], -1e-6);
    EXPECT_LE(times[i], end_time + 1e-6);
    if (i > 0) {
      EXPECT_GT(times[i], times[i - 1]);
    }
  }

  const auto stop_time_it = std::find_if(
    times.begin(), times.end(), [&](double t) { return std::abs(t - expected_stop_time) < 1e-6; });
  EXPECT_EQ(stop_time_it, times.end());
}

}  // namespace autoware::trajectory_validator::plugin::safety::trajectory::time_distance
