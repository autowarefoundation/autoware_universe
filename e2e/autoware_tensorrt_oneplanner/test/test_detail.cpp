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

#include "autoware/tensorrt_oneplanner/detail.hpp"
#include "autoware/tensorrt_oneplanner/dimensions.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace autoware::tensorrt_oneplanner
{
namespace
{
constexpr std::size_t kAgentStride = static_cast<std::size_t>(diffusion_planner::OUTPUT_T) *
                                     static_cast<std::size_t>(diffusion_planner::POSE_DIM);
}  // namespace

TEST(OnePlannerDetail, NumElementsSkipsBatchDimension)
{
  // The leading (batch) dimension is intentionally excluded.
  EXPECT_EQ(detail::num_elements(std::array<int64_t, 3>{2, 3, 4}), 12u);
  EXPECT_EQ(detail::num_elements(std::array<int64_t, 4>{7, 1, 1, 1}), 1u);
  EXPECT_EQ(detail::num_elements(std::array<int64_t, 2>{5, 9}), 9u);
  // Matches the per-sample element count of the ego prediction tensor.
  EXPECT_EQ(detail::num_elements(EGO_PREDICTION_SHAPE), kAgentStride);
  // Degenerate shapes are well-defined (no dereference past the end).
  EXPECT_EQ(detail::num_elements(std::array<int64_t, 1>{4}), 1u);
  EXPECT_EQ(detail::num_elements(std::vector<int64_t>{}), 1u);
}

TEST(OnePlannerDetail, ExpandEgoPredictionLayout)
{
  std::vector<float> ego(kAgentStride);
  for (std::size_t i = 0; i < ego.size(); ++i) {
    ego[i] = static_cast<float>(i + 1);  // non-zero, distinct values
  }

  const std::vector<float> expanded = detail::expand_ego_prediction(ego);

  // Ego occupies the first agent block; all remaining agents are zero-filled.
  ASSERT_EQ(
    expanded.size(), static_cast<std::size_t>(diffusion_planner::MAX_NUM_AGENTS) * kAgentStride);
  for (std::size_t i = 0; i < kAgentStride; ++i) {
    EXPECT_FLOAT_EQ(expanded[i], ego[i]);
  }
  for (std::size_t i = kAgentStride; i < expanded.size(); ++i) {
    EXPECT_FLOAT_EQ(expanded[i], 0.0f);
  }
}

TEST(OnePlannerDetail, ExpandEgoPredictionRejectsWrongSize)
{
  EXPECT_THROW(
    detail::expand_ego_prediction(std::vector<float>(kAgentStride - 1)), std::runtime_error);
  EXPECT_THROW(
    detail::expand_ego_prediction(std::vector<float>(kAgentStride + 1)), std::runtime_error);
  EXPECT_THROW(detail::expand_ego_prediction(std::vector<float>{}), std::runtime_error);
}

TEST(OnePlannerDetail, EgoNormalizerConstantsAreStable)
{
  // Guard against accidental drift from the StateNormalizer baked into the exported graph.
  EXPECT_FLOAT_EQ(detail::kEgoPositionXMean, 10.0f);
  EXPECT_FLOAT_EQ(detail::kEgoPositionStd, 20.0f);
}

}  // namespace autoware::tensorrt_oneplanner
