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

#include "autoware/simpl/processing/preprocessor.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::simpl::test
{
namespace
{
bool is_close(double a, double b, double tol = 1e-6)
{
  return std::abs(a - b) < tol;
}
}  // namespace

using autoware::simpl::archetype::AgentHistory;
using autoware::simpl::archetype::AgentLabel;
using autoware::simpl::archetype::AgentState;
using autoware::simpl::archetype::MapLabel;
using autoware::simpl::archetype::Polyline;
using autoware::simpl::archetype::SimplException;
using autoware::simpl::processing::AbstractMetadata;
using autoware::simpl::processing::NodePoints;
using autoware::simpl::processing::PreProcessor;

// Ensure AbstractMetadata throws if vector sizes do not match
TEST(TestAbstractMetadata, MismatchedSizeThrows)
{
  NodePoints centers = {{0.0, 0.0}, {1.0, 1.0}};
  NodePoints vectors = {{0.0, 1.0}};  // Deliberately mismatched

  EXPECT_THROW({ AbstractMetadata m(centers, vectors); }, SimplException);
}

// Minimal input test for PreProcessor::process
TEST(TestPreProcessor, ProcessMinimalInput)
{
  // Construct PreProcessor with minimal configuration
  PreProcessor processor({static_cast<size_t>(AgentLabel::VEHICLE)}, 1, 1, 1, 2, 100.0, 10.0);

  // Create a single-agent history
  AgentState ego{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, AgentLabel::UNKNOWN, true};
  AgentHistory history("A", 1);
  history.update(AgentState{1.0, 1.0, 0.0, 0.0, 0.5, 0.5, AgentLabel::VEHICLE, true});
  std::vector<AgentHistory> histories = {history};

  // Create a polyline with 2 points
  std::vector<Polyline> polylines = {
    Polyline({{2.0, 0.0, 0.0, MapLabel::ROADWAY}, {3.0, 0.0, 0.0, MapLabel::ROADWAY}})};

  // Run preprocessing
  auto [agent_meta, map_meta, rpe] = processor.process(histories, polylines, ego);

  // Validate AgentMetadata
  EXPECT_EQ(agent_meta.size(), 1u);
  EXPECT_EQ(agent_meta.agent_ids.size(), 1u);
  EXPECT_EQ(agent_meta.tensor.num_agent, 1u);
  EXPECT_EQ(agent_meta.tensor.num_past, 1u);
  EXPECT_TRUE(is_close(agent_meta.tensor.data()[0], 0.0f));  // dx at t=0
  EXPECT_TRUE(is_close(agent_meta.centers[0].norm, std::hypot(1.0, 1.0)));

  // Validate MapMetadata
  EXPECT_EQ(map_meta.size(), 1u);
  EXPECT_EQ(map_meta.tensor.num_polyline, 1u);
  EXPECT_EQ(map_meta.tensor.num_point, 2u);
  EXPECT_FALSE(std::isnan(map_meta.tensor.data()[0]));

  // Validate RPE tensor
  size_t N = agent_meta.size(), K = map_meta.size();
  EXPECT_EQ(rpe.size(), (N + K) * (N + K) * 5);

  // Self-distance should be 0
  EXPECT_TRUE(is_close(rpe[4], 0.0));

  // Distance to another node should be positive
  size_t idx = (0 * (N + K) + 1) * 5 + 4;
  EXPECT_GT(rpe[idx], 0.0f);
}
}  // namespace autoware::simpl::test
