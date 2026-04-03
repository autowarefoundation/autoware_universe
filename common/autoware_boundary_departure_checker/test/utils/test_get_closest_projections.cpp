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

#include "autoware/boundary_departure_checker/data_structs.hpp"
#include "autoware/boundary_departure_checker/utils.hpp"
#include "test_plot_utils.hpp"

#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

namespace autoware::boundary_departure_checker
{

namespace
{
constexpr double th_critical = 0.5;
constexpr double th_near = 2.0;
constexpr double braking_dist_min = 10.0;
constexpr double braking_dist_max = 15.0;
constexpr double cutoff_time_crit = 2.0;
constexpr double cutoff_time_near = 3.0;

ProjectionToBound create_mock_projection(
  std::optional<FootprintType> type, double lat_dist, double lon_dist, double time_from_start,
  size_t idx = 0)
{
  ProjectionToBound p(idx);
  p.footprint_type_opt = type;
  p.lat_dist = lat_dist;
  p.lon_dist_on_pred_traj = lon_dist;
  p.time_from_start = time_from_start;
  // Dummy points just for completeness in plotting
  p.pt_on_ego = {lon_dist, lat_dist};
  p.pt_on_bound = {lon_dist, 0.0};
  return p;
}

Param create_mock_param()
{
  Param p;
  p.footprint_types_to_check = {
    autoware::boundary_departure_checker::FootprintType::NORMAL,
    autoware::boundary_departure_checker::FootprintType::LOCALIZATION,
    autoware::boundary_departure_checker::FootprintType::STEERING_STUCK};
  p.th_trigger.th_dist_to_boundary_m[SideKey::LEFT].min = th_critical;
  p.th_trigger.th_dist_to_boundary_m[SideKey::LEFT].max = th_near;
  p.th_cutoff_time_departure_s = cutoff_time_crit;
  p.th_cutoff_time_near_boundary_s = cutoff_time_near;
  return p;
}
#ifdef EXPORT_TEST_PLOT_FIGURE
void plot_trajectory_evaluation(
  const std::vector<ProjectionToBound> & res_vec, const std::string & title,
  const FootprintMap<Side<ProjectionsToBound>> * all_projections = nullptr,
  const std::optional<double> braking_zone_start = std::nullopt)
{
  auto plt = autoware::pyplot::import();

  // 1. Plot safety thresholds
  plt.axhline(
    Args(th_critical),
    Kwargs("color"_a = "red", "linestyle"_a = "--", "label"_a = "Critical Thresh"));
  plt.axhline(
    Args(th_near), Kwargs("color"_a = "orange", "linestyle"_a = "--", "label"_a = "Near Thresh"));
  plt.axvline(
    Args(braking_dist_max),
    Kwargs("color"_a = "black", "linestyle"_a = ":", "label"_a = "Max Braking"));

  // Optional: Plot the start of the braking zone
  if (braking_zone_start) {
    plt.axvline(
      Args(*braking_zone_start),
      Kwargs("color"_a = "orange", "linestyle"_a = ":", "label"_a = "Braking Zone Start"));
  }

  // 2. Plot ALL initial candidates as gray 'x' marks
  if (all_projections) {
    std::vector<double> cand_x;
    std::vector<double> cand_y;
    for (const auto & [type, side_map] : all_projections->data) {
      // Use .at() or [] safely because we know it's a test environment
      if (side_map.right.empty() && side_map.left.empty()) continue;
      for (const auto & cand : side_map[SideKey::LEFT]) {
        cand_x.push_back(cand.lon_dist_on_pred_traj);
        cand_y.push_back(cand.lat_dist);
      }
    }
    if (!cand_x.empty()) {
      plt.scatter(
        Args(cand_x, cand_y), Kwargs(
                                "color"_a = "gray", "marker"_a = "x", "s"_a = 60,
                                "label"_a = "All Candidates", "alpha"_a = 0.5));
    }
  }

  // 3. Plot the selected winners as solid circles
  std::vector<double> near_x;
  std::vector<double> near_y;
  std::vector<double> app_x;
  std::vector<double> app_y;
  std::vector<double> crit_x;
  std::vector<double> crit_y;
  for (const auto & res : res_vec) {
    if (res.departure_type_opt == DepartureType::NEAR_BOUNDARY) {
      near_x.push_back(res.lon_dist_on_pred_traj);
      near_y.push_back(res.lat_dist);
    } else if (res.departure_type_opt == DepartureType::APPROACHING_DEPARTURE) {
      app_x.push_back(res.lon_dist_on_pred_traj);
      app_y.push_back(res.lat_dist);
    } else if (res.departure_type_opt == DepartureType::CRITICAL_DEPARTURE) {
      crit_x.push_back(res.lon_dist_on_pred_traj);
      crit_y.push_back(res.lat_dist);
    }
  }

  if (!near_x.empty())
    plt.scatter(Args(near_x, near_y), Kwargs("color"_a = "green", "label"_a = "Selected (Near)"));
  if (!app_x.empty())
    plt.scatter(
      Args(app_x, app_y), Kwargs("color"_a = "orange", "label"_a = "Selected (Approaching)"));
  if (!crit_x.empty())
    plt.scatter(Args(crit_x, crit_y), Kwargs("color"_a = "red", "label"_a = "Selected (Critical)"));

  plt.xlabel(Args("Longitudinal Distance [m]"));
  plt.ylabel(Args("Lateral Distance [m]"));
  plt.title(Args(title));
  plt.legend();

  save_figure(plt, "test_get_closest_projections");
}
#endif
}  // namespace

// ==============================================================================
// 1. Tests for get_closest_projection_by_departure_severity (Single Index Logic)
// ==============================================================================

struct ProjectionAtIndexTestParam
{
  std::string description;
  std::vector<ProjectionToBound> candidates;
  std::optional<double> previous_longitudinal_distance;
  std::optional<DepartureType> expected_departure_type;
  std::optional<double> expected_lat_dist;
};

std::ostream & operator<<(std::ostream & os, const ProjectionAtIndexTestParam & p)
{
  return os << p.description;
}

class GetClosestProjectionAtIndexTest : public ::testing::TestWithParam<ProjectionAtIndexTestParam>
{
protected:
  Param param = create_mock_param();
};

TEST_P(GetClosestProjectionAtIndexTest, TestAllBranches)
{
  const auto & p = GetParam();

  const auto result = utils::get_closest_projection_by_departure_severity(
    p.candidates, param, braking_dist_min, braking_dist_max, SideKey::LEFT,
    p.previous_longitudinal_distance);

  if (p.expected_departure_type) {
    ASSERT_TRUE(result.has_value()) << "Expected a projection to be returned, but got nullopt.";
    EXPECT_EQ(result->departure_type_opt.value(), p.expected_departure_type.value());
    EXPECT_DOUBLE_EQ(result->lat_dist, p.expected_lat_dist.value());
  } else {
    EXPECT_FALSE(result.has_value()) << "Expected nullopt, but a projection was returned.";
  }

  // Generate Plot for individual index tests
  BDC_PLOT_RESULT({
    auto plt = autoware::pyplot::import();

    plt.axhline(
      Args(th_critical),
      Kwargs("color"_a = "red", "linestyle"_a = "--", "label"_a = "Critical Thresh"));
    plt.axhline(
      Args(th_near), Kwargs("color"_a = "orange", "linestyle"_a = "--", "label"_a = "Near Thresh"));

    plt.axvline(
      Args(braking_dist_min),
      Kwargs("color"_a = "gray", "linestyle"_a = ":", "label"_a = "Min Braking"));
    plt.axvline(
      Args(braking_dist_max),
      Kwargs("color"_a = "black", "linestyle"_a = ":", "label"_a = "Max Braking"));

    std::vector<double> cx;
    std::vector<double> cy;
    for (const auto & cand : p.candidates) {
      cx.push_back(cand.lon_dist_on_pred_traj);
      cy.push_back(cand.lat_dist);
    }
    if (!cx.empty()) {
      plt.scatter(
        Args(cx, cy), Kwargs("color"_a = "blue", "marker"_a = "x", "label"_a = "Candidates"));
    }

    if (result) {
      plt.scatter(
        Args(
          std::vector<double>{result->lon_dist_on_pred_traj},
          std::vector<double>{result->lat_dist}),
        Kwargs(
          "color"_a = "green", "marker"_a = "o", "s"_a = 100, "facecolors"_a = "none",
          "label"_a = "Selected Result"));
    }

    plt.xlabel(Args("Longitudinal Distance [m]"));
    plt.ylabel(Args("Lateral Distance [m]"));
    plt.title(Args("Single Index: " + p.description));
    plt.legend();

    save_figure(plt, "test_get_closest_projections");
  });
}

INSTANTIATE_TEST_SUITE_P(
  ClosestProjectionTests, GetClosestProjectionAtIndexTest,
  ::testing::Values(
    ProjectionAtIndexTestParam{"EmptyCandidates", {}, std::nullopt, std::nullopt, std::nullopt},
    ProjectionAtIndexTestParam{
      "NoFootprintType",
      {create_mock_projection(std::nullopt, 0.1, 5.0, 1.0)},
      std::nullopt,
      std::nullopt,
      std::nullopt},
    ProjectionAtIndexTestParam{
      "NormalCriticalPriority",
      {create_mock_projection(FootprintType::LOCALIZATION, 0.1, 5.0, 1.0),
       create_mock_projection(FootprintType::NORMAL, 0.4, 5.0, 1.0)},
      std::nullopt,
      DepartureType::CRITICAL_DEPARTURE,
      0.4},
    ProjectionAtIndexTestParam{
      "OutsideNearThreshold",
      {create_mock_projection(FootprintType::NORMAL, 3.0, 5.0, 1.0)},
      std::nullopt,
      DepartureType::NONE,  // CHANGED: Now expects the safe NONE skeleton
      3.0},                 // CHANGED: The distance of the skeleton
    ProjectionAtIndexTestParam{
      "ClosestNearBoundary",
      {create_mock_projection(FootprintType::NORMAL, 1.5, 5.0, 1.0),
       create_mock_projection(FootprintType::LOCALIZATION, 0.8, 5.0, 1.0),
       create_mock_projection(FootprintType::STEERING_STUCK, 1.9, 5.0, 1.0)},
      std::nullopt,
      DepartureType::NEAR_BOUNDARY,
      0.8},
    ProjectionAtIndexTestParam{
      "FilteredByPreviousLongitudinalDistance",
      {create_mock_projection(FootprintType::LOCALIZATION, 0.8, 5.2, 1.0)},
      5.0,
      std::nullopt,  // Remains nullopt (Filtered out by down sample)
      std::nullopt},
    ProjectionAtIndexTestParam{
      "CriticalNotFilteredByPreviousDistance",
      {create_mock_projection(FootprintType::NORMAL, 0.2, 5.2, 1.0)},
      5.0,
      DepartureType::CRITICAL_DEPARTURE,
      0.2},
    ProjectionAtIndexTestParam{
      "NearBoundaryExceedsCutoff",
      {create_mock_projection(
        FootprintType::NORMAL, 1.0, braking_dist_max + 1.0, cutoff_time_near + 0.5)},
      std::nullopt,
      DepartureType::NONE,  // CHANGED: Now expects the safe NONE skeleton
      1.0},                 // CHANGED: The distance of the skeleton
    ProjectionAtIndexTestParam{
      "CriticalDowngradedToApproaching",
      {create_mock_projection(
        FootprintType::NORMAL, 0.2, braking_dist_min + 1.0, cutoff_time_crit + 0.5)},
      std::nullopt,
      DepartureType::APPROACHING_DEPARTURE,
      0.2},
    ProjectionAtIndexTestParam{
      "ValidNearBoundaryWithinCutoffs",
      {create_mock_projection(
        FootprintType::NORMAL, 1.0, braking_dist_max - 1.0, cutoff_time_near + 0.5)},
      std::nullopt,
      DepartureType::NEAR_BOUNDARY,
      1.0}),
  ::testing::PrintToStringParamName());

// ==============================================================================
// 2. Tests for get_closest_projections_for_side (Multi-Index / Full Trajectory)
// ==============================================================================

class GetClosestProjectionsForSideTest : public ::testing::Test
{
protected:
  Param param = create_mock_param();
};

TEST_F(GetClosestProjectionsForSideTest, TestInvalidFootprints)
{
  FootprintMap<Side<ProjectionsToBound>> projections;

  for (const auto type : param.footprint_types_to_check) {
    projections[type][SideKey::LEFT] = {};
    projections[type][SideKey::RIGHT] = {};
  }

  auto result = autoware::boundary_departure_checker::utils::get_closest_projections_for_side(
    projections, param, braking_dist_min, braking_dist_max, SideKey::LEFT);

  EXPECT_FALSE(result.has_value());
}

TEST_F(GetClosestProjectionsForSideTest, TestValidFootprintsWithoutCritical)
{
  FootprintMap<Side<ProjectionsToBound>> projections;

  projections[FootprintType::NORMAL][SideKey::LEFT] = {
    create_mock_projection(std::nullopt, 3.0, 0.0, 1.0, 0),
    create_mock_projection(std::nullopt, 1.5, 10.0, 1.0, 1),
    create_mock_projection(std::nullopt, 1.2, 14.0, 1.0, 2)};

  projections[FootprintType::LOCALIZATION][SideKey::LEFT] = {
    create_mock_projection(std::nullopt, 3.0, 0.0, 1.0, 0),
    create_mock_projection(std::nullopt, 0.8, 10.0, 1.0, 1),
    create_mock_projection(std::nullopt, 1.5, 14.0, 1.0, 2)};

  projections[FootprintType::STEERING_STUCK][SideKey::LEFT] = {
    create_mock_projection(std::nullopt, 3.0, 0.0, 1.0, 0),
    create_mock_projection(std::nullopt, 1.8, 10.0, 1.0, 1),
    create_mock_projection(std::nullopt, 1.9, 14.0, 1.0, 2)};

  auto result = utils::get_closest_projections_for_side(
    projections, param, braking_dist_min, braking_dist_max, SideKey::LEFT);

  ASSERT_TRUE(result.has_value());
  const auto & res_vec = result.value();

  ASSERT_EQ(res_vec.size(), 2);

  EXPECT_EQ(res_vec[0].ego_sides_idx, 1);
  EXPECT_EQ(res_vec[0].footprint_type_opt.value(), FootprintType::LOCALIZATION);
  EXPECT_EQ(res_vec[0].departure_type_opt.value(), DepartureType::NEAR_BOUNDARY);

  EXPECT_EQ(res_vec[1].ego_sides_idx, 2);
  EXPECT_EQ(res_vec[1].footprint_type_opt.value(), FootprintType::NORMAL);
  EXPECT_EQ(res_vec[1].departure_type_opt.value(), DepartureType::NEAR_BOUNDARY);

  BDC_PLOT_RESULT({
    plot_trajectory_evaluation(
      res_vec, "Trajectory Filtering: Valid Footprints w/o Critical", &projections);
  });
}

TEST_F(GetClosestProjectionsForSideTest, TestApproachingDowngradeLoop)
{
  FootprintMap<Side<ProjectionsToBound>> projections;

  // Simulate a trajectory approaching a boundary.
  // CRITICAL threshold = 0.5, braking_dist_max = 15.0
  projections[FootprintType::NORMAL][SideKey::LEFT] = {
    create_mock_projection(std::nullopt, 1.5, 0.0, 1.0, 0),   // Index 0: 30m away from wall
    create_mock_projection(std::nullopt, 1.5, 14.0, 1.0, 1),  // Index 1: 14m away from wall
    create_mock_projection(std::nullopt, 0.1, 28.0, 1.0, 2),  // Index 2: Hits wall (Critical)
    create_mock_projection(std::nullopt, 0.1, 40.0, 1.0, 3)   // Index 3: Past wall
  };

  // Dummy data so map matches sizes
  projections[FootprintType::LOCALIZATION][SideKey::LEFT] =
    projections[FootprintType::NORMAL][SideKey::LEFT];
  projections[FootprintType::STEERING_STUCK][SideKey::LEFT] =
    projections[FootprintType::NORMAL][SideKey::LEFT];

  auto result = utils::get_closest_projections_for_side(
    projections, param, braking_dist_min, braking_dist_max, SideKey::LEFT);

  ASSERT_TRUE(result.has_value());
  const auto & res_vec = result.value();

  ASSERT_EQ(res_vec.size(), 3);  // Breaks after index 2

  // Point 0: Outside max_braking_dist (30.0 - 0.0 > 15.0)
  EXPECT_EQ(res_vec[0].ego_sides_idx, 0);
  EXPECT_EQ(res_vec[0].departure_type_opt.value(), DepartureType::NEAR_BOUNDARY);

  // Point 1: Inside max_braking_dist (30.0 - 16.0 < 15.0) -> Downgraded!
  EXPECT_EQ(res_vec[1].ego_sides_idx, 1);
  EXPECT_EQ(res_vec[1].departure_type_opt.value(), DepartureType::APPROACHING_DEPARTURE);

  // Point 2: Critical Departure
  EXPECT_EQ(res_vec[2].ego_sides_idx, 2);
  EXPECT_EQ(res_vec[2].departure_type_opt.value(), DepartureType::CRITICAL_DEPARTURE);

  BDC_PLOT_RESULT({
    double braking_start = res_vec.back().lon_dist_on_pred_traj - braking_dist_max;
    plot_trajectory_evaluation(res_vec, "Trajectory Downgrade Logic", nullptr, braking_start);
  });
}

TEST_F(GetClosestProjectionsForSideTest, TestBackwardBufferAndCleanup)
{
  FootprintMap<Side<ProjectionsToBound>> projections;

  // We simulate a physical crash at s = 20.0m.
  // min_braking = 10.0, max_braking = 15.0, cutoff = 2.0 (from mock param)
  // Buffer is 1.0m. Therefore:
  // - CRITICAL zone: 19.0m to 20.0m
  // - APPROACHING zone: 4.0m to 19.0m (20.0 - 15.0 - 1.0 = 4.0)
  projections[FootprintType::NORMAL][SideKey::LEFT] = {
    create_mock_projection(
      std::nullopt, 1.5, 0.0, 0.0, 0),  // dist_to_crash = 20.0 (> 16.0) -> NEAR
    create_mock_projection(
      std::nullopt, 1.5, 10.0, 1.0, 1),  // dist_to_crash = 10.0 (<= 16.0) -> APPROACHING
    create_mock_projection(
      std::nullopt, 1.5, 19.0, 1.9,
      2),  // dist_to_crash = 1.0  (<= 1.0) -> CRITICAL (Earliest buffered point!)
    create_mock_projection(
      std::nullopt, 1.5, 19.5, 1.95,
      3),  // dist_to_crash = 0.5  (<= 1.0) -> CRITICAL (Will be erased)
    create_mock_projection(
      std::nullopt, 0.1, 20.0, 2.0,
      4)  // dist_to_crash = 0.0  -> CRITICAL (Original crash, will be erased)
  };

  // Dummy data so map matches sizes
  projections[FootprintType::LOCALIZATION][SideKey::LEFT] =
    projections[FootprintType::NORMAL][SideKey::LEFT];
  projections[FootprintType::STEERING_STUCK][SideKey::LEFT] =
    projections[FootprintType::NORMAL][SideKey::LEFT];

  auto result = utils::get_closest_projections_for_side(
    projections, param, braking_dist_min, braking_dist_max, SideKey::LEFT);

  ASSERT_TRUE(result.has_value());
  const auto & res_vec = result.value();

  // 1. Verify Cleanup: Original size was 5. The last two (indices 3 and 4) should be erased.
  ASSERT_EQ(res_vec.size(), 3);

  // 2. Verify States
  EXPECT_EQ(res_vec[0].ego_sides_idx, 0);
  EXPECT_EQ(res_vec[0].departure_type_opt.value(), DepartureType::NEAR_BOUNDARY);

  EXPECT_EQ(res_vec[1].ego_sides_idx, 1);
  EXPECT_EQ(res_vec[1].departure_type_opt.value(), DepartureType::APPROACHING_DEPARTURE);

  // 3. Verify the Buffered Critical Point
  EXPECT_EQ(res_vec[2].ego_sides_idx, 2);
  EXPECT_EQ(res_vec[2].departure_type_opt.value(), DepartureType::CRITICAL_DEPARTURE);

  // 4. Verify the math: The new CRITICAL point must be exactly at the 1.0m buffer line (s = 19.0)
  EXPECT_DOUBLE_EQ(res_vec.back().lon_dist_on_pred_traj, 19.0);

  // Plot it using our shiny new helper!
  BDC_PLOT_RESULT({
    // Calculate the original braking start line based on the original crash point (20.0)
    double original_crash_s = 20.0;
    double braking_start = original_crash_s - braking_dist_max;
    plot_trajectory_evaluation(
      res_vec, "Backward Buffer and Cleanup Logic", nullptr, braking_start);
  });
}
}  // namespace autoware::boundary_departure_checker
