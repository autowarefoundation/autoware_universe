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

#include "autoware/pointcloud_preprocessor/utility/azimuth.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

using autoware::pointcloud_preprocessor::utils::azimuth_distance;
using autoware::pointcloud_preprocessor::utils::azimuth_step;
using autoware::pointcloud_preprocessor::utils::AzimuthOrderCounter;

namespace
{
constexpr float deg_to_rad = static_cast<float>(M_PI / 180.0);

std::vector<float> make_ring(float start_deg, float step_deg, int num_points)
{
  std::vector<float> azimuths;
  for (int i = 0; i < num_points; ++i) {
    float deg = std::fmod(start_deg + step_deg * static_cast<float>(i), 360.0F);
    if (deg < 0.0F) deg += 360.0F;
    azimuths.push_back(deg * deg_to_rad);
  }
  return azimuths;
}

std::vector<float> make_shuffled_ring(std::mt19937::result_type seed)
{
  auto azimuths = make_ring(0.0F, 0.2F, 1800);
  std::mt19937 rng(seed);
  std::shuffle(azimuths.begin(), azimuths.end(), rng);
  return azimuths;
}

AzimuthOrderCounter count(const std::vector<float> & azimuths)
{
  AzimuthOrderCounter counter;
  for (size_t i = 0; i + 1 < azimuths.size(); ++i) {
    counter.add(azimuth_step(azimuths[i], azimuths[i + 1]));
  }
  return counter;
}

AzimuthOrderCounter count_sampled(const std::vector<float> & azimuths)
{
  AzimuthOrderCounter counter;
  for (size_t i = 0; i + 1 < azimuths.size(); ++i) {
    counter.add_sampled(i, azimuths[i], azimuths[i + 1]);
  }
  return counter;
}
}  // namespace

TEST(AzimuthStepTest, SignFollowsDirectionAndWrapsAround)
{
  // Arrange
  const float step = 0.2F * deg_to_rad;

  // Act
  const float increasing = azimuth_step(10.0F * deg_to_rad, 10.2F * deg_to_rad);
  const float decreasing = azimuth_step(10.2F * deg_to_rad, 10.0F * deg_to_rad);
  const float increasing_across_zero = azimuth_step(359.9F * deg_to_rad, 0.1F * deg_to_rad);
  const float decreasing_across_zero = azimuth_step(0.1F * deg_to_rad, 359.9F * deg_to_rad);
  const float increasing_across_pi = azimuth_step(179.9F * deg_to_rad, -179.9F * deg_to_rad);
  const float no_step = azimuth_step(1.0F, 1.0F);

  // Assert
  EXPECT_NEAR(increasing, step, 1e-6F);
  EXPECT_NEAR(decreasing, -step, 1e-6F);
  EXPECT_NEAR(increasing_across_zero, step, 1e-5F);  // in [0, 2pi)
  EXPECT_NEAR(decreasing_across_zero, -step, 1e-5F);
  EXPECT_NEAR(increasing_across_pi, step, 1e-5F);  // in [-pi, pi)
  EXPECT_FLOAT_EQ(no_step, 0.0F);
}

TEST(AzimuthStepTest, AbsoluteValueMatchesAzimuthDistance)
{
  // Arrange
  // Both azimuths in [0, 2pi), or both in [-pi, pi), as documented
  const std::vector<std::vector<float>> ranges = {
    {0.0F, 0.5F, 3.0F, 3.3F, 6.2F}, {-3.1F, -1.0F, 0.0F, 0.5F, 3.1F}};

  for (const auto & values : ranges) {
    for (const float from : values) {
      for (const float to : values) {
        // Act
        const float step = azimuth_step(from, to);
        const float distance = azimuth_distance(from, to);

        // Assert
        EXPECT_NEAR(std::fabs(step), distance, 1e-5F) << "from=" << from << " to=" << to;
      }
    }
  }
}

TEST(AzimuthOrderCounterTest, IncreasingAndDecreasingRingsAreOrdered)
{
  // Arrange
  // Increasing (nebula) and decreasing (clockwise atan2), each across the wrap-around
  const auto increasing = make_ring(300.0F, 0.2F, 1800);
  const auto decreasing = make_ring(60.0F, -0.2F, 1800);

  // Act
  const auto increasing_counter = count(increasing);
  const auto decreasing_counter = count(decreasing);

  // Assert
  EXPECT_TRUE(increasing_counter.is_ordered());
  EXPECT_TRUE(decreasing_counter.is_ordered());
}

TEST(AzimuthOrderCounterTest, ShuffledRingIsNotOrdered)
{
  // Arrange
  const auto azimuths = make_shuffled_ring(42);

  // Act
  const auto counter = count(azimuths);

  // Assert
  EXPECT_FALSE(counter.is_ordered());
}

TEST(AzimuthOrderCounterTest, FewBackwardStepsAreTolerated)
{
  // Arrange
  // 1 backward step every 50 steps (2 %) is below the 5 % limit
  auto azimuths = make_ring(0.0F, 0.2F, 1000);
  for (size_t i = 25; i + 1 < azimuths.size(); i += 50) {
    std::swap(azimuths[i], azimuths[i + 1]);
  }

  // Act
  const auto counter = count(azimuths);

  // Assert
  EXPECT_GT(counter.backward(), 0U);
  EXPECT_TRUE(counter.is_ordered());
}

TEST(AzimuthOrderCounterTest, ZeroStepsAreNotCounted)
{
  // Arrange
  // Two returns per azimuth, as from a dual-return sensor
  std::vector<float> azimuths;
  for (const float azimuth : make_ring(0.0F, 0.2F, 100)) {
    azimuths.push_back(azimuth);
    azimuths.push_back(azimuth);
  }

  // Act
  const auto counter = count(azimuths);

  // Assert
  EXPECT_EQ(counter.backward(), 0U);
  EXPECT_EQ(counter.forward(), 99U);
  EXPECT_TRUE(counter.is_ordered());
}

TEST(AzimuthOrderCounterTest, TooFewStepsAreNotJudged)
{
  // Arrange
  // Half of the steps go backward in both counters, but only the second one reaches min_steps.
  AzimuthOrderCounter eight_steps;
  AzimuthOrderCounter ten_steps;
  for (int i = 0; i < 4; ++i) {
    eight_steps.add(0.1F);
    eight_steps.add(-0.1F);
  }
  for (int i = 0; i < 5; ++i) {
    ten_steps.add(0.1F);
    ten_steps.add(-0.1F);
  }

  // Act
  const bool eight_steps_ordered = eight_steps.is_ordered();
  const bool ten_steps_ordered = ten_steps.is_ordered();

  // Assert
  EXPECT_TRUE(eight_steps_ordered);
  EXPECT_FALSE(ten_steps_ordered);
}

TEST(AzimuthOrderCounterTest, SamplingEvery8thStepStillDetectsShuffledRing)
{
  // Arrange
  // The ring outlier filter feeds the counter with every 8th step only.
  const auto shuffled = make_shuffled_ring(7);
  const auto ordered = make_ring(120.0F, -0.2F, 1800);

  // Act
  const auto shuffled_counter = count_sampled(shuffled);
  const auto ordered_counter = count_sampled(ordered);

  // Assert
  // 1799 steps, one counted out of 8
  EXPECT_EQ(ordered_counter.forward() + ordered_counter.backward(), 225U);
  EXPECT_FALSE(shuffled_counter.is_ordered());
  EXPECT_TRUE(ordered_counter.is_ordered());
}
