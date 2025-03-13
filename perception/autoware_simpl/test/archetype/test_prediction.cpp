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

#include "autoware/simpl/archetype/prediction.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace autoware::simpl
{
namespace
{
archetype::PredictedState construct_state()
{
  return {1.0, 1.0, 1.0, 1.0};
}

archetype::Prediction construct_prediction()
{
  constexpr size_t num_mode = 2;
  constexpr size_t num_future = 3;
  const std::vector<double> scores{
    0.2,  // mode1
    0.8   // mode2
  };
  const std::vector<double> trajectories{
    // mode1
    1.0, 1.0, 1.0, 1.0,  // t=1
    2.0, 2.0, 2.0, 2.0,  // t=2
    3.0, 3.0, 3.0, 3.0,  // t=3
    // mode2
    4.0, 4.0, 4.0, 4.0,  // t=1
    5.0, 5.0, 5.0, 5.0,  // t=2
    6.0, 6.0, 6.0, 6.0,  // t=3
  };

  return {num_mode, num_future, scores, trajectories};
}
}  // namespace

TEST(TestPrediction, testPredictedState)
{
  const auto state = construct_state();

  // check elements
  EXPECT_DOUBLE_EQ(state.x, 1.0);
  EXPECT_DOUBLE_EQ(state.y, 1.0);
  EXPECT_DOUBLE_EQ(state.vx, 1.0);
  EXPECT_DOUBLE_EQ(state.vy, 1.0);

  // check member functions
  EXPECT_EQ(state.num_attribute(), 4);
}

TEST(TestPrediction, testPrediction)
{
  const auto prediction = construct_prediction();

  // check elements
  EXPECT_EQ(prediction.num_mode, 2);
  EXPECT_EQ(prediction.num_future, 3);

  for (size_t m = 0; m < prediction.num_mode; ++m) {
    const auto & waypoints = prediction.at(m);
    for (size_t t = 0; t < prediction.num_future; ++t) {
      const auto & score = waypoints.first;
      if (m == 0) {
        EXPECT_DOUBLE_EQ(score, 0.2);
      } else {
        EXPECT_DOUBLE_EQ(score, 0.8);
      }

      const auto & state = waypoints.second.at(t);
      const double value = m * prediction.num_future + t + 1.0;
      EXPECT_DOUBLE_EQ(state.x, value);
      EXPECT_DOUBLE_EQ(state.y, value);
      EXPECT_DOUBLE_EQ(state.vx, value);
      EXPECT_DOUBLE_EQ(state.vy, value);
    }
  }
}
}  // namespace autoware::simpl
