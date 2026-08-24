// Copyright 2026 Xinchen Lin
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

#include "autoware/trajectory_ranker/camp_ranker.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

namespace autoware::trajectory_ranker
{
namespace
{

CampFixedWeightModel deployment_model()
{
  return load_camp_fixed_weight_model(CAMP_TEST_MODEL_PATH);
}

CampStatusPattern all_observed()
{
  CampStatusPattern status{};
  status.fill(CampAtomStatus::Observed);
  return status;
}

}  // namespace

TEST(CampRanker, LoadsDeploymentModel)
{
  const auto model = deployment_model();
  EXPECT_EQ(model.candidate_pool_k, 8U);
  EXPECT_EQ(model.atom_names.size(), kCampAtomCount);
  EXPECT_EQ(model.patterns.size(), 24U);
}

TEST(CampRanker, AppliesFrozenScalesAndWeights)
{
  const auto model = deployment_model();
  const auto status = all_observed();
  const auto pattern = std::find_if(
    model.patterns.begin(), model.patterns.end(),
    [&status](const CampPatternWeights & candidate) { return candidate.status == status; });
  ASSERT_NE(pattern, model.patterns.end());

  std::vector<CampAtomVector> candidates(model.candidate_pool_k, CampAtomVector{});
  candidates.at(1).at(0) = model.scales.at(0);
  candidates.at(2).at(13) = 2.0 * model.scales.at(13);

  const auto result = rank_camp_candidates(model, status, candidates);
  EXPECT_EQ(result.selected_index, 0U);
  EXPECT_NEAR(result.costs.at(1), pattern->weights.at(0), 1.0e-12);
  EXPECT_NEAR(result.costs.at(2), 2.0 * pattern->weights.at(13), 1.0e-12);
}

TEST(CampRanker, MatchesTheReferenceImplementation)
{
  const auto model = deployment_model();
  std::vector<CampAtomVector> candidates(model.candidate_pool_k, CampAtomVector{});
  for (std::size_t k = 0; k < candidates.size(); ++k) {
    for (std::size_t atom = 0; atom < kCampAtomCount; ++atom) {
      const auto multiplier = static_cast<double>(((k + 1) * (atom + 2)) % 17) / 4.0;
      candidates.at(k).at(atom) = model.scales.at(atom) * multiplier;
    }
  }

  const auto result = rank_camp_candidates(model, all_observed(), candidates);
  const std::vector<double> expected_costs{
    2.115275390002698, 2.1576671584107743, 1.9804126215115798, 2.023544030207182,
    1.971631120089219, 2.018646435540969, 2.0821084140558774, 1.8918500852111888};

  ASSERT_EQ(result.costs.size(), expected_costs.size());
  for (std::size_t i = 0; i < expected_costs.size(); ++i) {
    EXPECT_NEAR(result.costs.at(i), expected_costs.at(i), 1.0e-12);
  }
  EXPECT_EQ(result.selected_index, 7U);
}

TEST(CampRanker, IgnoresNonObservedAtomValues)
{
  const auto model = deployment_model();
  const auto status = model.patterns.front().status;
  std::vector<CampAtomVector> candidates(model.candidate_pool_k, CampAtomVector{});

  for (auto & candidate : candidates) {
    for (std::size_t i = 0; i < kCampAtomCount; ++i) {
      if (status.at(i) != CampAtomStatus::Observed) {
        candidate.at(i) = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }

  EXPECT_NO_THROW(rank_camp_candidates(model, status, candidates));
}

TEST(CampRanker, RejectsNonFiniteObservedAtoms)
{
  const auto model = deployment_model();
  const auto status = all_observed();
  std::vector<CampAtomVector> candidates(model.candidate_pool_k, CampAtomVector{});
  candidates.front().front() = std::numeric_limits<double>::quiet_NaN();

  EXPECT_THROW(rank_camp_candidates(model, status, candidates), std::invalid_argument);
}

TEST(CampRanker, KeepsTheFirstCandidateForAnExactTie)
{
  const auto model = deployment_model();
  const auto result = rank_camp_candidates(
    model, all_observed(),
    std::vector<CampAtomVector>(model.candidate_pool_k, CampAtomVector{}));

  EXPECT_EQ(result.selected_index, 0U);
}

TEST(CampRanker, RejectsTheWrongCandidatePoolSize)
{
  const auto model = deployment_model();
  EXPECT_THROW(
    rank_camp_candidates(model, all_observed(), std::vector<CampAtomVector>(7)),
    std::invalid_argument);
}

}  // namespace autoware::trajectory_ranker
