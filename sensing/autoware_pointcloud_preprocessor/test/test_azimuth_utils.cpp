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

AzimuthOrderCounter count(const std::vector<float> & azimuths)
{
  AzimuthOrderCounter counter;
  for (size_t i = 0; i + 1 < azimuths.size(); ++i) {
    counter.add(azimuth_step(azimuths[i], azimuths[i + 1]));
  }
  return counter;
}
}  // namespace

TEST(AzimuthStepTest, SignFollowsDirectionAndWrapsAround)
{
  EXPECT_NEAR(azimuth_step(10.0F * deg_to_rad, 10.2F * deg_to_rad), 0.2F * deg_to_rad, 1e-6F);
  EXPECT_NEAR(azimuth_step(10.2F * deg_to_rad, 10.0F * deg_to_rad), -0.2F * deg_to_rad, 1e-6F);
  // Across 0 in [0, 2pi)
  EXPECT_NEAR(azimuth_step(359.9F * deg_to_rad, 0.1F * deg_to_rad), 0.2F * deg_to_rad, 1e-5F);
  EXPECT_NEAR(azimuth_step(0.1F * deg_to_rad, 359.9F * deg_to_rad), -0.2F * deg_to_rad, 1e-5F);
  // Across pi in [-pi, pi)
  EXPECT_NEAR(azimuth_step(179.9F * deg_to_rad, -179.9F * deg_to_rad), 0.2F * deg_to_rad, 1e-5F);
  EXPECT_FLOAT_EQ(azimuth_step(1.0F, 1.0F), 0.0F);
}

TEST(AzimuthStepTest, AbsoluteValueMatchesAzimuthDistance)
{
  // Both azimuths in [0, 2pi), or both in [-pi, pi), as documented
  const std::vector<std::vector<float>> ranges = {
    {0.0F, 0.5F, 3.0F, 3.3F, 6.2F}, {-3.1F, -1.0F, 0.0F, 0.5F, 3.1F}};
  for (const auto & values : ranges) {
    for (const float from : values) {
      for (const float to : values) {
        EXPECT_NEAR(std::fabs(azimuth_step(from, to)), azimuth_distance(from, to), 1e-5F)
          << "from=" << from << " to=" << to;
      }
    }
  }
}

TEST(AzimuthOrderCounterTest, IncreasingAndDecreasingRingsAreOrdered)
{
  // Increasing (nebula) and decreasing (clockwise atan2), each across the wrap-around
  EXPECT_TRUE(count(make_ring(300.0F, 0.2F, 1800)).is_ordered());
  EXPECT_TRUE(count(make_ring(60.0F, -0.2F, 1800)).is_ordered());
}

TEST(AzimuthOrderCounterTest, ShuffledRingIsNotOrdered)
{
  auto azimuths = make_ring(0.0F, 0.2F, 1800);
  std::mt19937 rng(42);
  std::shuffle(azimuths.begin(), azimuths.end(), rng);
  EXPECT_FALSE(count(azimuths).is_ordered());
}

TEST(AzimuthOrderCounterTest, FewBackwardStepsAreTolerated)
{
  // 1 backward step every 50 steps (2 %) is below the default 5 %
  auto azimuths = make_ring(0.0F, 0.2F, 1000);
  for (size_t i = 25; i + 1 < azimuths.size(); i += 50) {
    std::swap(azimuths[i], azimuths[i + 1]);
  }
  const auto counter = count(azimuths);
  EXPECT_GT(counter.backward(), 0U);
  EXPECT_TRUE(counter.is_ordered());
}

TEST(AzimuthOrderCounterTest, ZeroStepsAreNotCounted)
{
  // Two returns per azimuth, as from a dual-return sensor
  std::vector<float> azimuths;
  for (const float azimuth : make_ring(0.0F, 0.2F, 100)) {
    azimuths.push_back(azimuth);
    azimuths.push_back(azimuth);
  }
  const auto counter = count(azimuths);
  EXPECT_EQ(counter.backward(), 0U);
  EXPECT_EQ(counter.forward(), 99U);
  EXPECT_TRUE(counter.is_ordered());
}

TEST(AzimuthOrderCounterTest, TooFewStepsAreNotJudged)
{
  AzimuthOrderCounter counter;
  for (int i = 0; i < 4; ++i) {
    counter.add(0.1F);
    counter.add(-0.1F);
  }
  EXPECT_TRUE(counter.is_ordered());  // 8 steps < min_steps
  counter.add(0.1F);
  counter.add(-0.1F);
  EXPECT_FALSE(counter.is_ordered());  // 10 steps, half backward
  counter.reset();
  EXPECT_EQ(counter.forward() + counter.backward(), 0U);
}

TEST(AzimuthOrderCounterTest, SamplingEvery8thStepStillDetectsShuffledRing)
{
  // The ring outlier filter feeds the counter with every 8th step only.
  auto shuffled = make_ring(0.0F, 0.2F, 1800);
  std::mt19937 rng(7);
  std::shuffle(shuffled.begin(), shuffled.end(), rng);
  const auto ordered = make_ring(120.0F, -0.2F, 1800);
  AzimuthOrderCounter shuffled_counter;
  AzimuthOrderCounter ordered_counter;
  for (size_t i = 0; i + 1 < shuffled.size(); ++i) {
    shuffled_counter.add_sampled(i, shuffled[i], shuffled[i + 1]);
    ordered_counter.add_sampled(i, ordered[i], ordered[i + 1]);
  }
  // 1799 steps, one counted out of 8
  EXPECT_EQ(ordered_counter.forward() + ordered_counter.backward(), 225U);
  EXPECT_FALSE(shuffled_counter.is_ordered());
  EXPECT_TRUE(ordered_counter.is_ordered());
}
