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
#include "autoware/simpl_prediction/processing/preprocessor.hpp"

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
using autoware::simpl_prediction::archetype::MapLabel;
using autoware::simpl_prediction::archetype::Polyline;
using autoware::simpl_prediction::archetype::SimplException;
using autoware::simpl_prediction::processing::AbstractMetadata;
using autoware::simpl_prediction::processing::NodePoints;
using autoware::simpl_prediction::processing::PreProcessor;

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
  AgentState ego{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, true};
  AgentHistory history("A", AgentLabel::VEHICLE, 1);
  history.update(AgentState{1.0, 1.0, 0.0, 0.0, 0.5, 0.5, true});
  std::vector<AgentHistory> histories = {history};

  // Create a polyline with 2 points
  std::vector<Polyline> polylines = {
    Polyline(0, {{2.0, 0.0, 0.0, MapLabel::ROADWAY}, {3.0, 0.0, 0.0, MapLabel::ROADWAY}})};

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

class PreProcessorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    setup_agent();
    setup_map();
  }

  static constexpr size_t num_agent = 1;
  static constexpr size_t num_past = 3;
  static constexpr size_t num_polyline = 1;
  static constexpr size_t num_point = 10;
  static constexpr double polyline_range_distance = 100.0;
  static constexpr double polyline_break_distance = 1.0;

  std::vector<size_t> label_ids;
  std::vector<AgentHistory> histories;
  std::vector<Polyline> polylines;
  AgentState current_ego;

private:
  void setup_agent()
  {
    label_ids = archetype::to_label_ids({"VEHICLE", "PEDESTRIAN", "CYCLIST"});
    std::vector<std::pair<std::string, archetype::AgentLabel>> agent_id_and_label{
      {"a", archetype::AgentLabel::VEHICLE},
      {"b", archetype::AgentLabel::PEDESTRIAN},
    };

    for (const auto & [agent_id, label] : agent_id_and_label) {
      archetype::AgentHistory history(agent_id, label, num_past);
      for (size_t t = 0; t < num_past; ++t) {
        history.update(archetype::AgentState(1, 1, 1, 1, 1, 1, true));
      }
      histories.emplace_back(history);
    }

    current_ego = archetype::AgentState(1, 1, 1, 1, 1, 1, true);
  }

  void setup_map()
  {
    // lane
    archetype::Polyline lane{
      0,
      {
        {1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY},
        {1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY},
        {1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY},
      }};
    polylines.emplace_back(lane);

    // left boundary

    archetype::Polyline left_boundary{
      1,
      {
        {1.0, 1.0, 1.0, archetype::MapLabel::SOLID},
        {1.0, 1.0, 1.0, archetype::MapLabel::SOLID},
      }};
    polylines.emplace_back(left_boundary);

    // right boundary
    archetype::Polyline right_boundary{2, {{1.0, 1.0, 1.0, archetype::MapLabel::DASHED}}};
    polylines.emplace_back(right_boundary);

    // crosswalk
    archetype::Polyline crosswalk{
      3,
      {
        {1.0, 1.0, 1.0, archetype::MapLabel::CROSSWALK},
        {1.0, 1.0, 1.0, archetype::MapLabel::CROSSWALK},
        {1.0, 1.0, 1.0, archetype::MapLabel::CROSSWALK},
        {1.0, 1.0, 1.0, archetype::MapLabel::CROSSWALK},
        {1.0, 1.0, 1.0, archetype::MapLabel::CROSSWALK},
      }};
    polylines.emplace_back(crosswalk);
  }
};

TEST_F(PreProcessorTest, PreProcess)
{
  PreProcessor preprocessor(
    label_ids, num_agent, num_past, num_polyline, num_point, polyline_range_distance,
    polyline_break_distance);

  const auto [agent_metadata, map_metadata, rpe_tensor] =
    preprocessor.process(histories, polylines, current_ego);

  // clang-format off
  std::vector<float> expected_agent = {
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    1.0f, 1.0f, 1.0f,
    0.0f, 0.0f, 0.0f,
    1.3817732f, 1.3817732f, 1.3817732f,
    -0.30116862f, -0.30116862f, -0.30116862f,
    1.0f, 1.0f, 1.0f,
    0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
    1.0f, 1.0f, 1.0f,
  };

  std::vector<float> expected_map = {
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f
  };

  std::vector<float> expected_rpe = {
    1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f
  };
  // clang-format on

  ASSERT_EQ(agent_metadata.tensor.size(), expected_agent.size());
  ASSERT_EQ(map_metadata.tensor.size(), expected_map.size());
  ASSERT_EQ(rpe_tensor.size(), expected_rpe.size());

  for (size_t i = 0; i < agent_metadata.tensor.size(); ++i) {
    auto answer = *(agent_metadata.tensor.data() + i);
    auto & expect = expected_agent[i];
    EXPECT_NEAR(answer, expect, 1e-5);
  }

  for (size_t i = 0; i < map_metadata.tensor.size(); ++i) {
    auto answer = *(map_metadata.tensor.data() + i);
    auto & expect = expected_map[i];
    EXPECT_NEAR(answer, expect, 1e-5);
  }

  for (size_t i = 0; i < rpe_tensor.size(); ++i) {
    auto & answer = rpe_tensor[i];
    auto & expect = expected_rpe[i];
    EXPECT_NEAR(answer, expect, 1e-5);
  }
}
}  // namespace autoware::simpl_prediction::test
