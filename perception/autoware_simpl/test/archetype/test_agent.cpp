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

#include <gtest/gtest.h>
#include <math.h>

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace autoware::simpl
{
namespace
{
archetype::AgentState construct_state()
{
  return {1.0, 1.0, 1.0, 0.5 * M_PI, 1.0, 1.0, archetype::AgentLabel::VEHICLE, true};
}

archetype::AgentHistory construct_history(const std::string & agent_id)
{
  archetype::AgentHistory history(agent_id, 10, 1.0, construct_state());
  for (double time = 2.0; time <= 5.0; ++time) {
    if (std::fmod(time, 2) == 0) {
      history.update(time, construct_state());
    } else {
      history.update();  // update with empty state
    }
  }
  return history;
}
}  // namespace

TEST(TestAgent, testToLabelIds)
{
  std::vector<std::string> label_names{
    "VEHICLE", "PEDESTRIAN", "MOTORCYCLIST", "CYCLIST", "LARGE_VEHICLE"};

  const auto result = archetype::to_label_ids(label_names);
  ASSERT_EQ(result.size(), 5UL);
  EXPECT_EQ(result[0], static_cast<size_t>(archetype::AgentLabel::VEHICLE));
  EXPECT_EQ(result[1], static_cast<size_t>(archetype::AgentLabel::PEDESTRIAN));
  EXPECT_EQ(result[2], static_cast<size_t>(archetype::AgentLabel::MOTORCYCLIST));
  EXPECT_EQ(result[3], static_cast<size_t>(archetype::AgentLabel::CYCLIST));
  EXPECT_EQ(result[4], static_cast<size_t>(archetype::AgentLabel::LARGE_VEHICLE));
}

TEST(TestAgent, testAgentState)
{
  const auto state = construct_state();

  // check elements
  EXPECT_DOUBLE_EQ(state.x, 1.0);
  EXPECT_DOUBLE_EQ(state.y, 1.0);
  EXPECT_DOUBLE_EQ(state.z, 1.0);
  EXPECT_DOUBLE_EQ(state.yaw, 0.5 * M_PI);
  EXPECT_DOUBLE_EQ(state.vx, 1.0);
  EXPECT_DOUBLE_EQ(state.vy, 1.0);
  EXPECT_EQ(state.label, archetype::AgentLabel::VEHICLE);
  EXPECT_TRUE(state.is_valid);

  // check member functions
  EXPECT_EQ(state.num_attribute(), 8);
}

TEST(TestAgent, testAgentHistory)
{
  auto history = construct_history("agent1");

  // the latest valid time is 4.0
  EXPECT_TRUE(history.is_ancient(6.0, 2.0));

  // the latest state is invalid
  EXPECT_FALSE(history.is_current_valid());

  // the history size is 10
  EXPECT_EQ(history.size(), 10);
}
}  // namespace autoware::simpl
