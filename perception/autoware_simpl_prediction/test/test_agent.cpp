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

#include "autoware/simpl_prediction/archetype/agent.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

namespace autoware::simpl_prediction::test
{
namespace
{
bool is_close(double a, double b, double tol = 1e-6)
{
  return std::abs(a - b) < tol;
}
}  // namespace

using autoware::simpl_prediction::archetype::AgentHistory;
using autoware::simpl_prediction::archetype::AgentLabel;
using autoware::simpl_prediction::archetype::AgentState;

TEST(TestAgentState, TransformToAnotherFrame)
{
  AgentState original(10.0, 0.0, 0.0, M_PI / 2, 1.0, 0.0, true);
  AgentState frame(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);

  AgentState transformed = original.transform(frame);

  EXPECT_TRUE(is_close(transformed.x, 10.0));
  EXPECT_TRUE(is_close(transformed.y, 0.0));
  EXPECT_TRUE(is_close(transformed.yaw, M_PI / 2));
  EXPECT_TRUE(is_close(transformed.vx, 1.0));
  EXPECT_TRUE(is_close(transformed.vy, 0.0));
}

TEST(TestAgentState, TransformToRotatedFrame)
{
  AgentState original{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, true};
  AgentState frame{0.0, 0.0, 0.0, M_PI / 2, 0.0, 0.0, true};

  AgentState transformed = original.transform(frame);

  EXPECT_TRUE(is_close(transformed.x, 0.0));
  EXPECT_TRUE(is_close(transformed.y, -1.0));
  EXPECT_TRUE(is_close(transformed.vx, 0.0));
  EXPECT_TRUE(is_close(transformed.vy, -1.0));
  EXPECT_TRUE(is_close(transformed.yaw, -M_PI / 2));
}

TEST(TestAgentHistory, DistanceFromReference)
{
  AgentState ref{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true};
  AgentState state{3.0, 4.0, 0.0, 0.0, 0.0, 0.0, true};
  AgentHistory history("agent0", AgentLabel::UNKNOWN, 1, state);

  EXPECT_DOUBLE_EQ(history.distance_from(ref), 5.0);
}

TEST(TestAgentHistory, TransformToCurrentFrame)
{
  AgentHistory history("agentX", AgentLabel::UNKNOWN, 3);

  // 3 time steps: t0 = (2, 0), t1 = (1, 0), t2 = (0, 0) ← current
  history.update(AgentState{2.0, 0.0, 0.0, 0.0, 0.0, 0.0, true});
  history.update(AgentState{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, true});
  history.update(AgentState{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true});

  AgentHistory transformed = history.transform_to_current();

  ASSERT_EQ(transformed.size(), 3u);
  EXPECT_TRUE(is_close(transformed.at(0).x, 2.0));
  EXPECT_TRUE(is_close(transformed.at(1).x, 1.0));
  EXPECT_TRUE(is_close(transformed.at(2).x, 0.0));
}

TEST(TestTrimNeighbors, ReturnsNearestAgents)
{
  AgentState reference(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);

  std::vector<AgentHistory> histories;
  for (int i = 0; i < 5; ++i) {
    AgentState state(static_cast<double>(i), 0.0, 0.0, 0.0, 0.0, 0.0, true);
    histories.emplace_back("agent" + std::to_string(i), AgentLabel::UNKNOWN, 1, state);
  }

  const auto label_ids = archetype::to_label_ids({"VEHICLE", "UNKNOWN"});

  auto result = trim_neighbors(histories, label_ids, reference, 3);

  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result[0].agent_id, "agent0");
  EXPECT_EQ(result[1].agent_id, "agent1");
  EXPECT_EQ(result[2].agent_id, "agent2");
}

TEST(TestTrimNeighbors, FilterNonTargetAgents)
{
  AgentState reference(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);

  std::vector<AgentHistory> histories;
  for (int i = 0; i < 5; ++i) {
    AgentState state(static_cast<double>(i), 0.0, 0.0, 0.0, 0.0, 0.0, true);
    histories.emplace_back("agent" + std::to_string(i), AgentLabel::UNKNOWN, 1, state);
  }

  const auto label_ids = archetype::to_label_ids({"VEHICLE"});

  auto result = trim_neighbors(histories, label_ids, reference, 3);

  EXPECT_TRUE(result.empty());
}

TEST(TestTrimNeighbors, HandlesTopKLargerThanInput)
{
  AgentState reference(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);

  std::vector<AgentHistory> histories;
  for (int i = 0; i < 2; ++i) {
    AgentState state(static_cast<double>(i), 0.0, 0.0, 0.0, 0.0, 0.0, true);
    histories.emplace_back("agent" + std::to_string(i), AgentLabel::UNKNOWN, 1, state);
  }

  const auto label_ids = archetype::to_label_ids({"VEHICLE", "UNKNOWN"});

  auto result = trim_neighbors(histories, label_ids, reference, 10);

  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].agent_id, "agent0");
  EXPECT_EQ(result[1].agent_id, "agent1");
}

TEST(TestTrimNeighbors, ReturnEmptyWhenNoInput)
{
  AgentState reference(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true);

  std::vector<AgentHistory> histories;

  const auto label_ids = archetype::to_label_ids({"VEHICLE", "UNKNOWN"});

  auto result = trim_neighbors(histories, label_ids, reference, 3);

  EXPECT_TRUE(result.empty());
}
}  // namespace autoware::simpl_prediction::test
