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

#include "autoware/diffusion_planner/camp_atom_materializer.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace autoware::diffusion_planner
{
namespace
{

constexpr std::int64_t kCandidateCount = 8;
constexpr std::int64_t kAgentCount = MAX_NUM_AGENTS;
constexpr std::size_t kPoseDimensions = static_cast<std::size_t>(POSE_DIM);
constexpr std::size_t kPointsPerLaneSegment = static_cast<std::size_t>(POINTS_PER_SEGMENT);
constexpr std::size_t kLanePointDimensions = static_cast<std::size_t>(SEGMENT_POINT_DIM);
constexpr std::size_t kLaneSegmentCount = static_cast<std::size_t>(NUM_SEGMENTS_IN_LANE);
constexpr std::size_t kRouteSegmentCount = static_cast<std::size_t>(NUM_SEGMENTS_IN_ROUTE);

std::size_t prediction_index(
  const std::size_t candidate, const std::size_t agent, const std::size_t time,
  const std::size_t component)
{
  return (((candidate * static_cast<std::size_t>(kAgentCount) + agent) * kCampHorizonSteps + time) *
          kPoseDimensions) +
         component;
}

std::size_t lane_index(
  const std::size_t segment, const std::size_t point, const std::size_t feature)
{
  return (segment * kPointsPerLaneSegment + point) * kLanePointDimensions + feature;
}

void populate_straight_lane(std::vector<float> & tensor)
{
  for (std::size_t point = 0; point < kPointsPerLaneSegment; ++point) {
    const float x = -20.0F + 6.0F * static_cast<float>(point);
    tensor.at(lane_index(0, point, X)) = x;
    tensor.at(lane_index(0, point, dX)) = 6.0F;
    tensor.at(lane_index(0, point, LB_Y)) = 5.0F;
    tensor.at(lane_index(0, point, RB_Y)) = -5.0F;
    tensor.at(lane_index(0, point, TRAFFIC_LIGHT_NO_TRAFFIC_LIGHT)) = 1.0F;
  }
}

CampAtomMaterializationInput make_straight_input()
{
  CampAtomMaterializationInput input;
  input.batch_size = kCandidateCount;
  input.agent_count = kAgentCount;
  input.ego_wheelbase_m = 2.7;
  input.ego_length_m = 4.8;
  input.ego_width_m = 2.0;
  input.denormalized_predictions.assign(
    static_cast<std::size_t>(kCandidateCount * kAgentCount) * kCampHorizonSteps * kPoseDimensions,
    0.0F);

  for (std::size_t candidate = 0; candidate < static_cast<std::size_t>(kCandidateCount);
       ++candidate) {
    const float displacement = 0.125F * static_cast<float>(candidate + 1);
    for (std::size_t time = 0; time < kCampHorizonSteps; ++time) {
      input.denormalized_predictions.at(prediction_index(candidate, 0, time, 0)) =
        displacement * static_cast<float>(time + 1);
      input.denormalized_predictions.at(prediction_index(candidate, 0, time, 2)) = 1.0F;
    }
  }

  input.tensor_context.lanes.assign(
    kLaneSegmentCount * kPointsPerLaneSegment * kLanePointDimensions, 0.0F);
  input.tensor_context.route_lanes.assign(
    kRouteSegmentCount * kPointsPerLaneSegment * kLanePointDimensions, 0.0F);
  input.tensor_context.route_speed_limits.assign(kRouteSegmentCount, 0.0F);
  populate_straight_lane(input.tensor_context.lanes);
  populate_straight_lane(input.tensor_context.route_lanes);
  input.tensor_context.route_speed_limits.at(0) = 20.0F;
  return input;
}

TEST(CampAtomMaterializer, StraightPoolMatchesFixedWeightRankingContract)
{
  const std::array<double, 3> transition_scales{
    15.340749389584412, 0.5042537448405768, 4.582727731088896};
  const auto materialized = materialize_camp_atoms(make_straight_input(), transition_scales);

  ASSERT_EQ(materialized.raw_atoms.size(), 8U);
  EXPECT_EQ(materialized.status.at(3), trajectory_ranker::CampAtomStatus::Observed);
  EXPECT_EQ(materialized.status.at(4), trajectory_ranker::CampAtomStatus::Observed);
  EXPECT_EQ(materialized.status.at(6), trajectory_ranker::CampAtomStatus::NotApplicable);
  EXPECT_EQ(materialized.status.at(7), trajectory_ranker::CampAtomStatus::NotApplicable);
  EXPECT_EQ(materialized.status.at(15), trajectory_ranker::CampAtomStatus::NotApplicable);
  for (const auto & atoms : materialized.raw_atoms) {
    EXPECT_TRUE(std::isnan(atoms.at(6)));
    EXPECT_TRUE(std::isnan(atoms.at(7)));
    EXPECT_TRUE(std::isnan(atoms.at(15)));
  }

  for (const auto & atoms : materialized.raw_atoms) {
    EXPECT_DOUBLE_EQ(atoms.at(0), 0.0);
    EXPECT_DOUBLE_EQ(atoms.at(1), 0.0);
    EXPECT_DOUBLE_EQ(atoms.at(2), 0.0);
    EXPECT_DOUBLE_EQ(atoms.at(3), 0.0);
    EXPECT_NEAR(atoms.at(4), 0.0, 1.0e-9);
    EXPECT_DOUBLE_EQ(atoms.at(5), 0.0);
    for (std::size_t atom = 9; atom <= 14; ++atom) {
      EXPECT_NEAR(atoms.at(atom), 0.0, 1.0e-9);
    }
  }
  for (std::size_t candidate = 0; candidate < materialized.raw_atoms.size(); ++candidate) {
    EXPECT_NEAR(
      materialized.raw_atoms.at(candidate).at(8),
      10.0 * static_cast<double>(materialized.raw_atoms.size() - candidate - 1), 1.0e-9);
  }

  const auto model = trajectory_ranker::load_camp_fixed_weight_model(CAMP_TEST_MODEL_PATH);
  const auto ranking =
    trajectory_ranker::rank_camp_candidates(model, materialized.status, materialized.raw_atoms);
  EXPECT_EQ(ranking.selected_index, 7U);
}

TEST(CampAtomMaterializer, PreviousPlanActivatesContinuityAtom)
{
  const std::array<double, 3> transition_scales{
    15.340749389584412, 0.5042537448405768, 4.582727731088896};
  const auto first = materialize_camp_atoms(make_straight_input(), transition_scales);
  auto second_input = make_straight_input();
  second_input.previous_plan = CampPreviousPlan{0.0, first.candidate_world_plans.front()};
  const auto second = materialize_camp_atoms(second_input, transition_scales);

  EXPECT_EQ(second.status.at(15), trajectory_ranker::CampAtomStatus::Observed);
  EXPECT_NEAR(second.raw_atoms.front().at(15), 0.0, 1.0e-12);
  EXPECT_NEAR(second.raw_atoms.at(1).at(15), 0.27000403286881997, 1.0e-9);
}

TEST(CampAtomMaterializer, PreservesTypedMissingEndpointSemantics)
{
  const std::array<double, 3> transition_scales{
    15.340749389584412, 0.5042537448405768, 4.582727731088896};
  auto input = make_straight_input();
  input.tensor_context.route_speed_limits.at(0) = 0.0F;
  input.tensor_context.route_has_traffic_light = true;
  const auto materialized = materialize_camp_atoms(input, transition_scales);

  EXPECT_EQ(materialized.status.at(3), trajectory_ranker::CampAtomStatus::TypedMissing);
  EXPECT_EQ(materialized.status.at(6), trajectory_ranker::CampAtomStatus::TypedMissing);
  EXPECT_EQ(materialized.status.at(7), trajectory_ranker::CampAtomStatus::TypedMissing);
  for (const auto & atoms : materialized.raw_atoms) {
    EXPECT_TRUE(std::isnan(atoms.at(3)));
    EXPECT_TRUE(std::isnan(atoms.at(6)));
    EXPECT_TRUE(std::isnan(atoms.at(7)));
  }
  const auto model = trajectory_ranker::load_camp_fixed_weight_model(CAMP_TEST_MODEL_PATH);
  EXPECT_NO_THROW(
    trajectory_ranker::rank_camp_candidates(model, materialized.status, materialized.raw_atoms));
}

TEST(CampAtomMaterializer, NonzeroAtomsMatchTheFrozenReferenceFormulas)
{
  const std::array<double, 3> transition_scales{
    15.340749389584412, 0.5042537448405768, 4.582727731088896};
  auto input = make_straight_input();
  input.tensor_context.route_speed_limits.at(0) = 5.0F;
  input.actor_shapes.at(0) = {true, 4.0, 2.0};
  for (std::size_t candidate = 0; candidate < static_cast<std::size_t>(kCandidateCount);
       ++candidate) {
    for (std::size_t time = 0; time < kCampHorizonSteps; ++time) {
      const std::size_t ego_x = prediction_index(candidate, 0, time, 0);
      input.denormalized_predictions.at(prediction_index(candidate, 1, time, 0)) =
        input.denormalized_predictions.at(ego_x) + 4.0F;
      input.denormalized_predictions.at(prediction_index(candidate, 1, time, 2)) = 1.0F;
    }
  }
  const auto materialized = materialize_camp_atoms(input, transition_scales);

  const double collision_ttc = 0.1 * 80.0 * 0.95 * 0.95;
  for (const auto & atoms : materialized.raw_atoms) {
    EXPECT_DOUBLE_EQ(atoms.at(0), 1.0);
    EXPECT_NEAR(atoms.at(1), collision_ttc, 1.0e-12);
    EXPECT_DOUBLE_EQ(atoms.at(2), 0.0);
  }
  EXPECT_DOUBLE_EQ(materialized.raw_atoms.at(3).at(3), 0.0);
  EXPECT_NEAR(materialized.raw_atoms.back().at(3), 197.5, 1.0e-9);

  auto off_road = make_straight_input();
  for (std::size_t candidate = 0; candidate < static_cast<std::size_t>(kCandidateCount);
       ++candidate) {
    for (std::size_t time = 0; time < kCampHorizonSteps; ++time) {
      off_road.denormalized_predictions.at(prediction_index(candidate, 0, time, 1)) = 10.0F;
    }
  }
  const auto off_road_atoms = materialize_camp_atoms(off_road, transition_scales);
  for (const auto & atoms : off_road_atoms.raw_atoms) {
    EXPECT_NEAR(atoms.at(4), 8.0, 1.0e-9);
  }
}

}  // namespace
}  // namespace autoware::diffusion_planner
