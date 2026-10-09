// Copyright 2026 Xinchen Lin
// Licensed under the Apache License, Version 2.0.

#include "autoware/camp_selector/camp_atom_materializer.hpp"
#include "autoware/camp_selector/tensor_dimensions.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::camp_selector
{
namespace
{
CampAtomMaterializationInput straight_input(const double y)
{
  CampAtomMaterializationInput input;
  input.batch_size = 1;
  input.agent_count = 33;
  input.ego_wheelbase_m = 2.7;
  input.ego_length_m = 4.8;
  input.ego_width_m = 2.0;
  input.denormalized_predictions.assign(33 * kCampHorizonSteps * POSE_DIM, 0.0F);
  for (std::size_t time = 0; time < kCampHorizonSteps; ++time) {
    input.denormalized_predictions.at(time * POSE_DIM) = static_cast<float>(time + 1);
    input.denormalized_predictions.at(time * POSE_DIM + 1) = static_cast<float>(y);
    input.denormalized_predictions.at(time * POSE_DIM + 2) = 1.0F;
  }
  input.tensor_context.lanes.assign(
    NUM_SEGMENTS_IN_LANE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM, 0.0F);
  input.tensor_context.route_lanes.assign(
    NUM_SEGMENTS_IN_ROUTE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM, 0.0F);
  input.tensor_context.route_speed_limits.assign(NUM_SEGMENTS_IN_ROUTE, 0.0F);
  input.tensor_context.route_speed_limits.at(0) = 20.0F;
  for (auto * tensor : {&input.tensor_context.lanes, &input.tensor_context.route_lanes}) {
    for (std::int64_t point = 0; point < POINTS_PER_SEGMENT; ++point) {
      const auto base = point * SEGMENT_POINT_DIM;
      tensor->at(base + X) = -20.0F + 6.0F * static_cast<float>(point);
      tensor->at(base + dX) = 6.0F;
      tensor->at(base + LB_Y) = 5.0F;
      tensor->at(base + RB_Y) = -5.0F;
      tensor->at(base + TRAFFIC_LIGHT_NO_TRAFFIC_LIGHT) = 1.0F;
    }
  }
  input.ego_to_map << 0.0, -1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 200.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 1.0;
  return input;
}

CampLaneBoundary boundary(const Eigen::Matrix4d & transform, const double start, const double end)
{
  CampLaneBoundary result;
  for (const double x : {start, end}) {
    result.left_boundary.push_back((transform * Eigen::Vector4d(x, 5.0, 0.0, 1.0)).head<3>());
    result.right_boundary.push_back((transform * Eigen::Vector4d(x, -5.0, 0.0, 1.0)).head<3>());
  }
  return result;
}

TEST(CampLaneBoundary, MapTransformAndUnionMatchOriginalTensorGeometry)
{
  for (const double y : {0.0, 10.0}) {
    auto input = straight_input(y);
    const auto fallback = materialize_camp_atoms(input, {1.0, 1.0, 1.0});
    const std::vector<CampLaneBoundary> boundaries{
      boundary(input.ego_to_map, -20.0, 40.0), boundary(input.ego_to_map, 30.0, 94.0)};
    input.lane_boundaries = &boundaries;
    const auto mapped = materialize_camp_atoms(input, {1.0, 1.0, 1.0});
    EXPECT_EQ(mapped.status, fallback.status);
    for (std::size_t atom = 0; atom < kCampAtomCount; ++atom) {
      if (mapped.status.at(atom) == CampAtomStatus::Observed) {
        EXPECT_NEAR(mapped.raw_atoms.front().at(atom), fallback.raw_atoms.front().at(atom), 1.0e-9);
      } else {
        EXPECT_TRUE(std::isnan(mapped.raw_atoms.front().at(atom)));
      }
    }
    EXPECT_NEAR(mapped.raw_atoms.front().at(4), y == 0.0 ? 0.0 : 8.0, 1.0e-9);
    EXPECT_DOUBLE_EQ(mapped.candidate_world_plans.front().front().x_m, 100.0 - y);
    EXPECT_DOUBLE_EQ(mapped.candidate_world_plans.front().front().y_m, 201.0);
  }
}

TEST(CampLaneBoundary, EmptyAuthoritativeMapKeepsOnlyRoadEndpointMissing)
{
  auto input = straight_input(0.0);
  const std::vector<CampLaneBoundary> empty;
  input.lane_boundaries = &empty;
  const auto output = materialize_camp_atoms(input, {1.0, 1.0, 1.0});
  EXPECT_EQ(output.status.at(4), CampAtomStatus::TypedMissing);
  EXPECT_TRUE(std::isnan(output.raw_atoms.front().at(4)));
  EXPECT_EQ(output.status.at(3), CampAtomStatus::Observed);
  EXPECT_EQ(output.status.at(8), CampAtomStatus::Observed);
}
}  // namespace
}  // namespace autoware::camp_selector
