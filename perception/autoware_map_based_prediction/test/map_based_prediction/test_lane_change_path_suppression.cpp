// Copyright 2021 TIER IV, Inc.
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

#include "autoware/map_based_prediction/predictor_vehicle/lane_change_path_suppression.hpp"
#include "autoware/map_based_prediction/predictor_vehicle/path_processing.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <cmath>
#include <memory>
#include <vector>

namespace autoware::map_based_prediction::detail
{
namespace
{
class LaneChangePathSuppressionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Reuse boundary endpoints so the graph has one actual Successor relation.
    const lanelet::Point3d l0(1, 0.0, 1.0, 0.0);
    const lanelet::Point3d l1(2, 20.0, 1.0, 0.0);
    const lanelet::Point3d l2(3, 30.0, 1.0, 0.0);
    const lanelet::Point3d r0(4, 0.0, -1.0, 0.0);
    const lanelet::Point3d r1(5, 20.0, -1.0, 0.0);
    const lanelet::Point3d r2(6, 30.0, -1.0, 0.0);
    first_ =
      lanelet::Lanelet(11, lanelet::LineString3d(7, {l0, l1}), lanelet::LineString3d(8, {r0, r1}));
    last_ =
      lanelet::Lanelet(12, lanelet::LineString3d(9, {l1, l2}), lanelet::LineString3d(10, {r1, r2}));
    auto map = std::make_shared<lanelet::LaneletMap>();
    for (auto * lanelet : {&first_, &last_}) {
      lanelet->attributes()[lanelet::AttributeName::Subtype] = "road";
      lanelet->attributes()[lanelet::AttributeName::Location] = "urban";
      map->add(*lanelet);
    }
    const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    graph_ = lanelet::routing::RoutingGraph::build(*map, *rules);
    position_.x = 15.0;
    ASSERT_EQ(graph_->following(first_).size(), 1U);
    ASSERT_TRUE(graph_->following(last_).empty());
  }

  lanelet::ConstLanelets buildChain(const size_t depth, const bool cycle)
  {
    auto map = std::make_shared<lanelet::LaneletMap>();
    std::vector<lanelet::Point3d> left, right;
    lanelet::Id id = 1000;
    for (size_t i = 0; i <= depth; ++i) {
      if (cycle && i == depth) {
        left.push_back(left.front());
        right.push_back(right.front());
      } else if (cycle) {
        const double angle = 2.0 * M_PI * i / depth;
        left.emplace_back(id++, 9.0 * std::cos(angle), 9.0 * std::sin(angle), 0.0);
        right.emplace_back(id++, 11.0 * std::cos(angle), 11.0 * std::sin(angle), 0.0);
      } else {
        left.emplace_back(id++, i * 0.1, 1.0, 0.0);
        right.emplace_back(id++, i * 0.1, -1.0, 0.0);
      }
    }
    lanelet::ConstLanelets lanes;
    for (size_t i = 0; i < depth; ++i) {
      const lanelet::LineString3d lb(id++, {left[i], left[i + 1]});
      const lanelet::LineString3d rb(id++, {right[i], right[i + 1]});
      lanelet::Lanelet lane(id++, lb, rb);
      lane.attributes()[lanelet::AttributeName::Subtype] = "road";
      lane.attributes()[lanelet::AttributeName::Location] = "urban";
      map->add(lane);
      lanes.push_back(lane);
    }
    const auto rules = lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle);
    graph_ = lanelet::routing::RoutingGraph::build(*map, *rules);
    return lanes;
  }

  std::vector<LaneletPathWithPathInfo> makePaths() const
  {
    std::vector<LaneletPathWithPathInfo> paths;
    for (const auto maneuver :
         {Maneuver::LANE_FOLLOW, Maneuver::LEFT_LANE_CHANGE, Maneuver::RIGHT_LANE_CHANGE}) {
      PredictedRefPath info{};
      info.maneuver = maneuver;
      info.probability = maneuver == Maneuver::LANE_FOLLOW ? 1.0F : 0.3F;
      paths.emplace_back(lanelet::routing::LaneletPath({first_, last_}), info);
    }
    return paths;
  }

  // Origins of one current lanelet that has a lane-follow candidate.
  std::vector<LaneChangePathSuppressionInfo> makeOrigins(
    const Maneuver detected = Maneuver::LANE_FOLLOW, const bool lane_ends = false) const
  {
    return std::vector<LaneChangePathSuppressionInfo>(3, {detected, lane_ends, true});
  }

  lanelet::Lanelet first_;
  lanelet::Lanelet last_;
  lanelet::routing::RoutingGraphPtr graph_;
  geometry_msgs::msg::Point position_;
};

TEST_F(LaneChangePathSuppressionTest, ThousandLaneletChainHasLinearExpansionCount)
{
  for (const size_t depth : {100U, 1000U}) {
    const auto lanes = buildChain(depth, false);
    geometry_msgs::msg::Point position;
    size_t expanded = 0;
    ASSERT_TRUE(laneEndsWithinHorizon(lanes.front(), position, 150.0, *graph_, &expanded));
    EXPECT_EQ(expanded, depth);
    expanded = 0;
    EXPECT_FALSE(laneEndsWithinHorizon(lanes.front(), position, 0.25, *graph_, &expanded));
    EXPECT_EQ(expanded, 2U);
  }
}

TEST_F(LaneChangePathSuppressionTest, CycleTerminatesWithoutTerminal)
{
  const auto lanes = buildChain(32, true);
  for (const auto & lane : lanes) ASSERT_EQ(graph_->following(lane).size(), 1U);
  geometry_msgs::msg::Point position;
  position.x = 10.0;
  size_t expanded = 0;
  EXPECT_FALSE(laneEndsWithinHorizon(lanes.front(), position, 1000.0, *graph_, &expanded));
  EXPECT_EQ(expanded, lanes.size());
}

TEST_F(LaneChangePathSuppressionTest, RemovesUndetectedLaneChangesWithCenterPath)
{
  const auto result = suppressUndetectedLaneChangePaths(makePaths(), makeOrigins(), true, true);
  ASSERT_EQ(result.size(), 1U);
  EXPECT_EQ(result.front().second.maneuver, Maneuver::LANE_FOLLOW);
  EXPECT_FLOAT_EQ(result.front().second.probability, 1.0F);
}

TEST_F(LaneChangePathSuppressionTest, KeepsDetectedLaneChangesIncludingZeroWeightOpposite)
{
  for (const auto detected : {Maneuver::LEFT_LANE_CHANGE, Maneuver::RIGHT_LANE_CHANGE}) {
    auto paths = makePaths();
    paths[0].second.probability = 0.9F;
    paths[1].second.probability = detected == Maneuver::LEFT_LANE_CHANGE ? 1.0F : 0.0F;
    paths[2].second.probability = detected == Maneuver::RIGHT_LANE_CHANGE ? 1.0F : 0.0F;
    const auto result = suppressUndetectedLaneChangePaths(paths, makeOrigins(detected), true, true);
    ASSERT_EQ(result.size(), paths.size());
    for (size_t i = 0; i < paths.size(); ++i) {
      EXPECT_EQ(result[i].second.maneuver, paths[i].second.maneuver);
      EXPECT_FLOAT_EQ(result[i].second.probability, paths[i].second.probability);
    }
  }
}

TEST_F(LaneChangePathSuppressionTest, FiveSecondRescueDoesNotKeepLaneChangesAt60Meters)
{
  PathProcessor::Params params;
  EXPECT_DOUBLE_EQ(params.lane_change_path_suppression_lane_end_rescue_time_horizon, 5.0);
  // Current lane: 20 m, final lane: 55 m, object arc: 15 m => 60 m remaining.
  last_.setCenterline(
    lanelet::LineString3d(
      20, {lanelet::Point3d(21, 20.0, 0.0, 0.0), lanelet::Point3d(22, 75.0, 0.0, 0.0)}));
  double distance = 0.0;
  const bool lane_ends = laneEndsWithinHorizon(
    first_, position_, 10.0 * params.lane_change_path_suppression_lane_end_rescue_time_horizon,
    *graph_, nullptr, &distance);
  EXPECT_FALSE(lane_ends);
  EXPECT_DOUBLE_EQ(distance, -1.0);
  EXPECT_EQ(
    suppressUndetectedLaneChangePaths(
      makePaths(), makeOrigins(Maneuver::LANE_FOLLOW, lane_ends), true, true)
      .size(),
    1U);
}

TEST_F(LaneChangePathSuppressionTest, EighteenSecondRescueKeepsLaneChangesAt60Meters)
{
  PathProcessor::Params params;
  params.lane_change_path_suppression_lane_end_rescue_time_horizon = 18.0;
  last_.setCenterline(
    lanelet::LineString3d(
      20, {lanelet::Point3d(21, 20.0, 0.0, 0.0), lanelet::Point3d(22, 75.0, 0.0, 0.0)}));
  double distance = -1.0;
  const bool lane_ends = laneEndsWithinHorizon(
    first_, position_, 10.0 * params.lane_change_path_suppression_lane_end_rescue_time_horizon,
    *graph_, nullptr, &distance);
  ASSERT_TRUE(lane_ends);
  EXPECT_DOUBLE_EQ(distance, 60.0);
  EXPECT_EQ(
    suppressUndetectedLaneChangePaths(
      makePaths(), makeOrigins(Maneuver::LANE_FOLLOW, lane_ends), true, true)
      .size(),
    3U);
  // Disabling rescue still suppresses LC regardless of the configured rescue horizon.
  EXPECT_EQ(
    suppressUndetectedLaneChangePaths(
      makePaths(), makeOrigins(Maneuver::LANE_FOLLOW, lane_ends), true, false)
      .size(),
    1U);
}

TEST_F(LaneChangePathSuppressionTest, DisabledRetainsOriginalCandidates)
{
  const auto paths = makePaths();
  const auto result = suppressUndetectedLaneChangePaths(paths, makeOrigins(), false, true);
  ASSERT_EQ(result.size(), paths.size());
  for (size_t i = 0; i < paths.size(); ++i) {
    EXPECT_EQ(result[i].second.maneuver, paths[i].second.maneuver);
    EXPECT_FLOAT_EQ(result[i].second.probability, paths[i].second.probability);
  }
}

TEST_F(LaneChangePathSuppressionTest, DeadEndBeyondHorizonDoesNotKeepLaneChanges)
{
  const bool lane_ends =
    laneEndsWithinHorizon(lanelet::routing::LaneletPath({first_, last_}), position_, 14.9, *graph_);
  EXPECT_FALSE(lane_ends);
  EXPECT_EQ(
    suppressUndetectedLaneChangePaths(
      makePaths(), makeOrigins(Maneuver::LANE_FOLLOW, lane_ends), true, true)
      .size(),
    1U);
}

TEST_F(LaneChangePathSuppressionTest, UsesArcLengthInsteadOfEndpointDistance)
{
  last_.setCenterline(
    lanelet::LineString3d(
      20, {lanelet::Point3d(21, 20.0, 0.0, 0.0), lanelet::Point3d(22, 20.0, 10.0, 0.0),
           lanelet::Point3d(23, 30.0, 10.0, 0.0)}));
  position_.x = 20.0;
  position_.y = 5.0;
  const lanelet::routing::LaneletPath path({last_});
  // Endpoint distance is sqrt(125) < 12, but remaining arc length is 15 m.
  EXPECT_FALSE(laneEndsWithinHorizon(path, position_, 12.0, *graph_));
  EXPECT_TRUE(laneEndsWithinHorizon(path, position_, 15.0, *graph_));
}

TEST_F(LaneChangePathSuppressionTest, SearchCutoffWithFollowingIsNotDeadEnd)
{
  EXPECT_FALSE(
    laneEndsWithinHorizon(lanelet::routing::LaneletPath({first_}), position_, 100.0, *graph_));
}

TEST_F(LaneChangePathSuppressionTest, CanDisableDeadEndException)
{
  EXPECT_EQ(
    suppressUndetectedLaneChangePaths(
      makePaths(), makeOrigins(Maneuver::LANE_FOLLOW, true), true, false)
      .size(),
    1U);
}

TEST_F(LaneChangePathSuppressionTest, OriginWithoutCenterKeepsLaneChanges)
{
  auto paths = makePaths();
  paths.erase(paths.begin());
  const std::vector<LaneChangePathSuppressionInfo> origins(
    2, {Maneuver::LANE_FOLLOW, false, false});
  EXPECT_EQ(suppressUndetectedLaneChangePaths(paths, origins, true, true).size(), 2U);
}

TEST_F(LaneChangePathSuppressionTest, UsesEachCandidatesOriginCenterExistence)
{
  auto paths = makePaths();
  auto origins = makeOrigins();
  // Another current lanelet contributes only an LC candidate: without its own center it is kept
  // even though the first lanelet has one.
  paths.push_back(paths[1]);
  origins.push_back({Maneuver::LANE_FOLLOW, false, false});
  // Lanelets with a center still suppress undetected LC unless detected or terminal.
  paths.push_back(paths[1]);
  origins.push_back({Maneuver::LANE_FOLLOW, false, true});
  paths.push_back(paths[1]);
  origins.push_back({Maneuver::LEFT_LANE_CHANGE, false, true});
  paths.push_back(paths[2]);
  origins.push_back({Maneuver::LANE_FOLLOW, true, true});
  const auto result = suppressUndetectedLaneChangePaths(paths, origins, true, true);
  ASSERT_EQ(result.size(), 4U);
  EXPECT_EQ(result[0].second.maneuver, Maneuver::LANE_FOLLOW);
  EXPECT_EQ(result[1].second.maneuver, Maneuver::LEFT_LANE_CHANGE);
  EXPECT_EQ(result[2].second.maneuver, Maneuver::LEFT_LANE_CHANGE);
  EXPECT_EQ(result[3].second.maneuver, Maneuver::RIGHT_LANE_CHANGE);
}

TEST_F(LaneChangePathSuppressionTest, TerminalOriginWithoutCenterKeepsItsLaneChange)
{
  auto paths = makePaths();
  // Only B contributes a center. A is terminal and contributes only its LC.
  paths.resize(2);
  const bool a_ends = laneEndsWithinHorizon(last_, position_, 15.0, *graph_);
  ASSERT_TRUE(a_ends);
  const std::vector<LaneChangePathSuppressionInfo> origins{
    {Maneuver::LANE_FOLLOW, false, true}, {Maneuver::LANE_FOLLOW, a_ends, false}};
  EXPECT_EQ(suppressUndetectedLaneChangePaths(paths, origins, true, true).size(), 2U);
}

TEST_F(LaneChangePathSuppressionTest, SelectsHeldPathsWhoseOriginCenterDidNotSurvive)
{
  auto held = makePaths();
  held.erase(held.begin());
  held[0].second.origin_lanelet_id = 1;
  held[1].second.origin_lanelet_id = 2;
  const auto restored = selectPathsToRestore(held, {1});
  ASSERT_EQ(restored.size(), 1U);
  EXPECT_EQ(restored.front().second.origin_lanelet_id, 2);
  EXPECT_EQ(restored.front().second.maneuver, Maneuver::RIGHT_LANE_CHANGE);
  EXPECT_TRUE(selectPathsToRestore(held, {1, 2}).empty());
  EXPECT_EQ(selectPathsToRestore(held, {}).size(), 2U);
}

TEST_F(LaneChangePathSuppressionTest, FollowsCurrentLaneletIndependentlyOfCenterCandidates)
{
  EXPECT_TRUE(laneEndsWithinHorizon(first_, position_, 15.0, *graph_));
  EXPECT_FALSE(laneEndsWithinHorizon(first_, position_, 14.9, *graph_));
}

TEST_F(LaneChangePathSuppressionTest, HoldsOnlyUndetectedNonTerminalOriginsForRestoration)
{
  auto paths = makePaths();
  paths.push_back(paths[1]);
  paths.push_back(paths[2]);
  paths.push_back(paths[2]);
  const std::vector<LaneChangePathSuppressionInfo> origins{
    {Maneuver::LANE_FOLLOW, false, true},       {Maneuver::LANE_FOLLOW, false, true},
    {Maneuver::LEFT_LANE_CHANGE, false, true},  {Maneuver::LANE_FOLLOW, true, true},
    {Maneuver::RIGHT_LANE_CHANGE, false, true}, {Maneuver::LANE_FOLLOW, false, false}};
  std::vector<LaneletPathWithPathInfo> held;
  const auto result = suppressUndetectedLaneChangePaths(paths, origins, true, true, &held);
  ASSERT_EQ(held.size(), 1U);
  EXPECT_EQ(held.front().second.maneuver, Maneuver::LEFT_LANE_CHANGE);
  EXPECT_EQ(result.size(), 5U);
}

TEST_F(LaneChangePathSuppressionTest, FallsBackIfAllCandidatesWouldBeRemoved)
{
  auto paths = makePaths();
  paths.erase(paths.begin());
  const std::vector<LaneChangePathSuppressionInfo> origins(2, {Maneuver::LANE_FOLLOW, false, true});
  // Defensive behavior for inconsistent metadata; normal generation always retains the center.
  EXPECT_EQ(suppressUndetectedLaneChangePaths(paths, origins, true, true).size(), 2U);
}
}  // namespace
}  // namespace autoware::map_based_prediction::detail
