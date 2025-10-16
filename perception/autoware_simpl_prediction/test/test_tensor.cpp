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

#include "autoware/simpl_prediction/archetype/tensor.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace autoware::simpl_prediction::test
{
using autoware::simpl_prediction::archetype::AgentTensor;
using autoware::simpl_prediction::archetype::MapTensor;
using autoware::simpl_prediction::archetype::SimplException;

TEST(TestAgentTensor, ConstructValid)
{
  size_t N = 2, T = 3, Da = 4;
  std::vector<float> tensor(N * T * Da, 1.0f);

  AgentTensor agent_tensor(tensor, N, T, Da);

  EXPECT_EQ(agent_tensor.size(), N * T * Da);
  EXPECT_EQ(agent_tensor.num_agent, N);
  EXPECT_EQ(agent_tensor.num_past, T);
  EXPECT_EQ(agent_tensor.num_attribute, Da);
  EXPECT_EQ(agent_tensor.data()[0], 1.0f);
}

TEST(TestAgentTensor, ConstructInvalidThrows)
{
  size_t N = 2, T = 3, Da = 4;
  std::vector<float> tensor(N * T * Da - 1, 1.0f);  // wrong size

  EXPECT_THROW(
    {
      try {
        AgentTensor agent_tensor(tensor, N, T, Da);
      } catch (const SimplException & e) {
        EXPECT_STREQ(e.what(), "[InvalidValue]: Invalid size of agent tensor: 23 != 24");
        throw;
      }
    },
    SimplException);
}

TEST(TestMapTensor, ConstructValid)
{
  size_t K = 2, P = 3, Dm = 2;
  std::vector<float> tensor(K * P * Dm, 0.5f);

  MapTensor map_tensor(tensor, K, P, Dm);

  EXPECT_EQ(map_tensor.size(), K * P * Dm);
  EXPECT_EQ(map_tensor.num_polyline, K);
  EXPECT_EQ(map_tensor.num_point, P);
  EXPECT_EQ(map_tensor.num_attribute, Dm);
  EXPECT_EQ(map_tensor.data()[0], 0.5f);
}

TEST(TestMapTensor, ConstructInvalidThrows)
{
  size_t K = 2, P = 3, Dm = 2;
  std::vector<float> tensor(K * P * Dm - 1, 0.5f);

  EXPECT_THROW(
    {
      try {
        MapTensor map_tensor(tensor, K, P, Dm);
      } catch (const SimplException & e) {
        EXPECT_STREQ(e.what(), "[InvalidValue]: Invalid size of map tensor: 11 != 12");
        throw;
      }
    },
    SimplException);
}
}  // namespace autoware::simpl_prediction::test
