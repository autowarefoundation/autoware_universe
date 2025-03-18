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

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/datatype.hpp"
#include "autoware/simpl/processing/postprocessor.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace autoware::simpl
{
TEST(TestPostProcessor, testPostProcessor)
{
  const std::string agent_id{"agent1"};
  constexpr size_t num_mode = 2;
  constexpr size_t num_future = 3;
  const std::vector<float> scores{
    0.2,  // mode1
    0.8   // mode2
  };
  const std::vector<float> trajectories{
    // mode1
    1.0, 1.0, 1.0, 1.0,  // t=1
    2.0, 2.0, 2.0, 2.0,  // t=2
    3.0, 3.0, 3.0, 3.0,  // t=3
    // mode2
    4.0, 4.0, 4.0, 4.0,  // t=1
    5.0, 5.0, 5.0, 5.0,  // t=2
    6.0, 6.0, 6.0, 6.0,  // t=3
  };

  std::vector<std::pair<std::string, archetype::AgentState>> current_states{
    {agent_id, {1.0, 1.0, 1.0, 0.5 * M_PI, 1.0, 1.0, archetype::AgentLabel::VEHICLE, true}}};

  const processing::PostProcessor processor(num_mode, num_future);
  const auto results = processor.process(scores, trajectories, current_states);

  EXPECT_EQ(results.size(), 1);

  const auto & prediction = results.front();
  // check elements
  EXPECT_EQ(prediction.agent_id, agent_id);
  EXPECT_EQ(prediction.num_mode, num_mode);
  EXPECT_EQ(prediction.num_future, num_future);
  constexpr double abs_error = 1e-6;
  for (size_t m = 0; m < prediction.num_mode; ++m) {
    const auto & waypoints = prediction.at(m);
    for (size_t t = 0; t < prediction.num_future; ++t) {
      const auto & score = waypoints.first;
      if (m == 0) {
        EXPECT_NEAR(score, 0.2, abs_error);
      } else {
        EXPECT_NEAR(score, 0.8, abs_error);
      }

      const auto & state = waypoints.second.at(t);
      const double value = m * prediction.num_future + t + 1.0;
      EXPECT_NEAR(state.x, 1.0 - value, abs_error);
      EXPECT_NEAR(state.y, value + 1.0, abs_error);
      EXPECT_NEAR(state.vx, -value, abs_error);
      EXPECT_NEAR(state.vy, value, abs_error);
    }
  }
}
}  // namespace autoware::simpl
