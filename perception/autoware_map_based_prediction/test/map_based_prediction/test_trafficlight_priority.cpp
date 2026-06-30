// Copyright 2026 TIER IV, inc.
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

#include "autoware/map_based_prediction/priority_predictor/signal_stop_hysteresis.hpp"
#include "autoware/map_based_prediction/priority_predictor/traffic_signal_stop_predictor.hpp"

#include <autoware/lanelet2_utils/nn_search.hpp>
#include <rclcpp/time.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/LaneletPath.h>

#include <optional>
#include <string>
#include <vector>

namespace autoware::map_based_prediction::priority_predictor
{
namespace
{
using autoware::experimental::lanelet2_utils::LaneletRTree;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;

lanelet::Point3d makePoint(const lanelet::Id id, const double x, const double y)
{
  return lanelet::Point3d(id, x, y, 0.0);
}

lanelet::Lanelet makeLanelet(lanelet::Id & next_id, const double x_offset)
{
  const lanelet::LineString3d left(
    next_id++, {makePoint(next_id++, x_offset, 1.0), makePoint(next_id++, x_offset + 1.0, 1.0)});
  const lanelet::LineString3d right(
    next_id++, {makePoint(next_id++, x_offset, 0.0), makePoint(next_id++, x_offset + 1.0, 0.0)});
  return lanelet::Lanelet(next_id++, left, right);
}

TrafficLightElement makeElement(const uint8_t color, const uint8_t shape)
{
  TrafficLightElement element;
  element.color = color;
  element.shape = shape;
  return element;
}

lanelet::LineString3d makeStopLine(lanelet::Id & next_id, const double x_pos)
{
  return lanelet::LineString3d(
    next_id++, {makePoint(next_id++, x_pos, -1.0), makePoint(next_id++, x_pos, 1.0)});
}

void attachTrafficLight(
  lanelet::Lanelet & lanelet, lanelet::Id & next_id,
  const std::optional<lanelet::LineString3d> & stop_line)
{
  const lanelet::LineString3d bulb(
    next_id++, {makePoint(next_id++, 0.0, 2.0), makePoint(next_id++, 0.5, 2.0)});
  const auto traffic_light = stop_line
                               ? lanelet::TrafficLight::make(next_id++, {}, {bulb}, *stop_line)
                               : lanelet::TrafficLight::make(next_id++, {}, {bulb});
  lanelet.addRegulatoryElement(traffic_light);
}

PosePath makeRefPath(const double length)
{
  PosePath ref_path;
  for (double x = 0.0; x <= length + 1e-6; x += 1.0) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = 0.0;
    ref_path.push_back(pose);
  }
  return ref_path;
}

PredictedPath makePredictedPathAlongX(const double length)
{
  PredictedPath predicted_path;
  for (double x = 0.0; x <= length + 1e-6; x += 0.5) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = 0.5;
    predicted_path.path.push_back(pose);
  }
  return predicted_path;
}

LaneletRTree makeRoadLaneletRTree(const lanelet::ConstLanelets & lanelets)
{
  return LaneletRTree(lanelets);
}

TEST(PriorityUtils, FindsTrafficLightLaneletOnPath)
{
  lanelet::Id id = 4000;
  const auto approach = makeLanelet(id, 0.0);
  auto junction = makeLanelet(id, 5.0);
  const auto stop_line = makeStopLine(id, 6.0);
  attachTrafficLight(junction, id, stop_line);
  const lanelet::routing::LaneletPath path(lanelet::ConstLanelets{approach, junction});
  lanelet::ConstLanelet signal_lanelet;
  ASSERT_TRUE(findTrafficLightLaneletOnPath(path, signal_lanelet));
  EXPECT_EQ(signal_lanelet.id(), junction.id());
  const auto got = getStopLineOrEntryEdge(signal_lanelet);
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->id(), stop_line.id());
}

TEST(PriorityUtils, FindsTrafficLightLaneletOnPredictedPath)
{
  lanelet::Id id = 4050;
  const auto approach = makeLanelet(id, 0.0);
  auto junction = makeLanelet(id, 5.0);
  const auto stop_line = makeStopLine(id, 6.0);
  attachTrafficLight(junction, id, stop_line);
  const LaneletRTree rtree = makeRoadLaneletRTree({approach, junction});
  const PredictedPath predicted_path = makePredictedPathAlongX(10.0);
  lanelet::ConstLanelet signal_lanelet;
  ASSERT_TRUE(findTrafficLightLaneletOnPredictedPath(predicted_path, rtree, signal_lanelet));
  EXPECT_EQ(signal_lanelet.id(), junction.id());
}

TEST(PriorityUtils, NoTrafficLightLaneletOnPath)
{
  lanelet::Id id = 4100;
  const auto a = makeLanelet(id, 0.0);
  const auto b = makeLanelet(id, 5.0);
  const lanelet::routing::LaneletPath path(lanelet::ConstLanelets{a, b});
  lanelet::ConstLanelet signal_lanelet;
  EXPECT_FALSE(findTrafficLightLaneletOnPath(path, signal_lanelet));
}

TEST(PriorityUtils, FindsFirstTrafficLightLaneletOnPath)
{
  lanelet::Id id = 4200;
  const auto approach = makeLanelet(id, 0.0);
  auto first = makeLanelet(id, 5.0);
  auto second = makeLanelet(id, 10.0);
  attachTrafficLight(first, id, makeStopLine(id, 6.0));
  attachTrafficLight(second, id, makeStopLine(id, 11.0));
  const lanelet::routing::LaneletPath path(lanelet::ConstLanelets{approach, first, second});
  lanelet::ConstLanelet signal_lanelet;
  ASSERT_TRUE(findTrafficLightLaneletOnPath(path, signal_lanelet));
  EXPECT_EQ(signal_lanelet.id(), first.id());
}

TEST(PriorityUtils, GetStopLineOrEntryEdgeFallsBackToEntryEdge)
{
  lanelet::Id id = 4300;
  auto junction = makeLanelet(id, 0.0);
  attachTrafficLight(junction, id, std::nullopt);
  EXPECT_TRUE(getStopLineOrEntryEdge(junction).has_value());
}

TEST(PriorityUtils, ArcLengthToStopLineUsesIntersectionNotCentroid)
{
  lanelet::Id id = 5000;
  const PosePath ref_path = makeRefPath(50.0);
  const lanelet::LineString3d stop_line(
    id++, {makePoint(id++, 10.0, 0.0), makePoint(id++, 12.0, 4.0)});
  const auto arc = arcLengthToStopLine(ref_path, stop_line);
  ASSERT_TRUE(arc.has_value());
  EXPECT_NEAR(*arc, 10.0, 0.2);
}

TEST(PriorityUtils, ArcLengthToStopLineFallsBackWhenNoCrossing)
{
  lanelet::Id id = 5100;
  const PosePath ref_path = makeRefPath(50.0);
  const lanelet::LineString3d stop_line(
    id++, {makePoint(id++, 60.0, -1.0), makePoint(id++, 60.0, 1.0)});
  const auto arc = arcLengthToStopLine(ref_path, stop_line);
  ASSERT_TRUE(arc.has_value());
  EXPECT_NEAR(*arc, 50.0, 0.6);
}

TEST(PriorityUtils, ArcLengthToStopLineStraightPath)
{
  lanelet::Id id = 5200;
  const auto ref_path = makeRefPath(20.0);
  const auto stop_line = makeStopLine(id, 10.0);
  const auto distance = arcLengthToStopLine(ref_path, stop_line);
  ASSERT_TRUE(distance.has_value());
  EXPECT_NEAR(*distance, 10.0, 1e-6);
}

TEST(PriorityUtils, ArcLengthToStopLineNoneForShortPath)
{
  lanelet::Id id = 5300;
  const PosePath ref_path;
  const auto stop_line = makeStopLine(id, 10.0);
  EXPECT_FALSE(arcLengthToStopLine(ref_path, stop_line).has_value());
}

TEST(PriorityUtils, ArcLengthToStopLineNoneWhenBehindPathStart)
{
  lanelet::Id id = 5400;
  const PosePath ref_path = makeRefPath(50.0);
  const auto stop_line = makeStopLine(id, -10.0);
  EXPECT_FALSE(arcLengthToStopLine(ref_path, stop_line).has_value());
}

TEST(PriorityUtils, HasStopLineAheadByObjectPosition)
{
  lanelet::Id id = 5500;
  const PosePath ref_path = makeRefPath(50.0);
  const auto stop_line = makeStopLine(id, 10.0);

  geometry_msgs::msg::Point position;
  position.y = 0.0;

  position.x = 5.0;
  EXPECT_TRUE(hasStopLineAhead(position, ref_path, stop_line));

  position.x = -5.0;
  EXPECT_TRUE(hasStopLineAhead(position, ref_path, stop_line));

  position.x = 15.0;
  EXPECT_FALSE(hasStopLineAhead(position, ref_path, stop_line));

  const auto behind_line = makeStopLine(id, -10.0);
  position.x = 5.0;
  EXPECT_FALSE(hasStopLineAhead(position, ref_path, behind_line));
}

TEST(PriorityUtils, SignalStopNotRequiredWithoutObservation)
{
  lanelet::Id id = 5000;
  const auto lane = makeLanelet(id, 0.0);
  EXPECT_FALSE(evaluateSignalStopRequirement(lane, std::nullopt));
}

TEST(PriorityUtils, SignalStopNotRequiredOnGreenCircle)
{
  lanelet::Id id = 5100;
  const auto lane = makeLanelet(id, 0.0);
  TrafficLightGroup signal;
  signal.elements.push_back(makeElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE));
  EXPECT_FALSE(evaluateSignalStopRequirement(lane, signal));
}

TEST(PriorityUtils, SignalStopRequiredOnAmber)
{
  lanelet::Id id = 5200;
  const auto lane = makeLanelet(id, 0.0);
  TrafficLightGroup signal;
  signal.elements.push_back(makeElement(TrafficLightElement::AMBER, TrafficLightElement::CIRCLE));
  EXPECT_TRUE(evaluateSignalStopRequirement(lane, signal));
}

TEST(PriorityUtils, SignalStopRequiredOnRed)
{
  lanelet::Id id = 5300;
  const auto lane = makeLanelet(id, 0.0);
  TrafficLightGroup signal;
  signal.elements.push_back(makeElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE));
  EXPECT_TRUE(evaluateSignalStopRequirement(lane, signal));
}

TEST(PriorityUtils, SignalStopNotRequiredForMatchingArrow)
{
  TrafficLightGroup signal;
  signal.elements.push_back(makeElement(TrafficLightElement::GREEN, TrafficLightElement::UP_ARROW));

  lanelet::Id id = 5500;
  auto straight_lane = makeLanelet(id, 0.0);
  straight_lane.setAttribute("turn_direction", "straight");
  EXPECT_FALSE(evaluateSignalStopRequirement(straight_lane, signal));

  auto right_lane = makeLanelet(id, 0.0);
  right_lane.setAttribute("turn_direction", "right");
  EXPECT_TRUE(evaluateSignalStopRequirement(right_lane, signal));
}

TEST(PriorityUtils, GetStopLineFromTrafficLight)
{
  lanelet::Id id = 6100;
  auto junction = makeLanelet(id, 0.0);
  const auto stop_line = makeStopLine(id, 10.0);
  attachTrafficLight(junction, id, stop_line);
  const auto got = getStopLine(junction);
  ASSERT_TRUE(got.has_value());
  EXPECT_EQ(got->id(), stop_line.id());
}

TEST(PriorityUtils, GetStopLineNoneWhenUntagged)
{
  lanelet::Id id = 6200;
  const auto plain_lanelet = makeLanelet(id, 0.0);
  EXPECT_FALSE(getStopLine(plain_lanelet).has_value());
}

TEST(PriorityUtils, ShouldAddStopHypothesisOnlyOnRedWithStopLineAhead)
{
  EXPECT_TRUE(shouldAddStopHypothesis(true, true));
  EXPECT_FALSE(shouldAddStopHypothesis(false, true));
  EXPECT_FALSE(shouldAddStopHypothesis(true, false));
}

TEST(PriorityUtils, StopHypothesisConfidenceCenterIsStrongest)
{
  const double weight = 0.35;
  const double center = weakenConfidenceInLaneChange(Maneuver::LANE_FOLLOW, weight);
  EXPECT_DOUBLE_EQ(center, weight);
  EXPECT_LT(weakenConfidenceInLaneChange(Maneuver::LEFT_LANE_CHANGE, weight), center);
  EXPECT_LT(weakenConfidenceInLaneChange(Maneuver::RIGHT_LANE_CHANGE, weight), center);
}

// ---- signal stop hysteresis -------------------------------------------------------

rclcpp::Time t(const double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
}

TrafficLightGroup makeGroup(const uint8_t color, const uint8_t shape = TrafficLightElement::CIRCLE)
{
  TrafficLightGroup group;
  group.elements.push_back(makeElement(color, shape));
  return group;
}

uint8_t firstColor(const TrafficLightGroup & group)
{
  return group.elements.empty() ? TrafficLightElement::UNKNOWN : group.elements.front().color;
}

TEST(PriorityUtils, StopHysteresisDelaysEnteringStop)
{
  SignalStabilizeState state;
  EXPECT_FALSE(debounceStopDecision(state, true, t(0.0), 0.2, 0.1));
  EXPECT_FALSE(debounceStopDecision(state, true, t(0.15), 0.2, 0.1));
  EXPECT_TRUE(debounceStopDecision(state, true, t(0.2), 0.2, 0.1));
}

TEST(PriorityUtils, StopHysteresisRevertsToGoSoonerThanStop)
{
  SignalStabilizeState state;
  ASSERT_FALSE(debounceStopDecision(state, true, t(0.0), 0.2, 0.1));
  ASSERT_TRUE(debounceStopDecision(state, true, t(0.2), 0.2, 0.1));
  EXPECT_TRUE(debounceStopDecision(state, false, t(0.25), 0.2, 0.1));
  EXPECT_TRUE(debounceStopDecision(state, false, t(0.3), 0.2, 0.1));
  EXPECT_FALSE(debounceStopDecision(state, false, t(0.35), 0.2, 0.1));
}

TEST(PriorityUtils, StopHysteresisAbsorbsSingleFrameFlicker)
{
  SignalStabilizeState state;
  EXPECT_FALSE(debounceStopDecision(state, true, t(0.0), 0.2, 0.1));
  EXPECT_FALSE(debounceStopDecision(state, true, t(0.1), 0.2, 0.1));
  EXPECT_FALSE(debounceStopDecision(state, false, t(0.12), 0.2, 0.1));
  EXPECT_FALSE(debounceStopDecision(state, true, t(0.2), 0.2, 0.1));
}

TEST(PriorityUtils, StopHysteresisIsIdempotentWithinAFrame)
{
  SignalStabilizeState state;
  EXPECT_FALSE(debounceStopDecision(state, true, t(0.0), 0.2, 0.1));
  EXPECT_FALSE(debounceStopDecision(state, true, t(0.0), 0.2, 0.1));
  EXPECT_FALSE(debounceStopDecision(state, true, t(0.0), 0.2, 0.1));
  EXPECT_TRUE(debounceStopDecision(state, true, t(0.2), 0.2, 0.1));
}

TEST(PriorityUtils, ShowsRedOrAmberCircleClassifiesCircleColors)
{
  EXPECT_TRUE(showsRedOrAmberCircle(makeGroup(TrafficLightElement::RED)));
  EXPECT_TRUE(showsRedOrAmberCircle(makeGroup(TrafficLightElement::AMBER)));
  EXPECT_FALSE(showsRedOrAmberCircle(makeGroup(TrafficLightElement::GREEN)));
  EXPECT_FALSE(showsRedOrAmberCircle(makeGroup(TrafficLightElement::UNKNOWN)));
  TrafficLightGroup mixed = makeGroup(TrafficLightElement::RED);
  mixed.elements.push_back(
    makeElement(TrafficLightElement::GREEN, TrafficLightElement::LEFT_ARROW));
  EXPECT_TRUE(showsRedOrAmberCircle(mixed));
}

TEST(PriorityUtils, StabilizeHoldsLastStableUntilStopCommits)
{
  SignalStabilizeState state;
  EXPECT_EQ(
    firstColor(debounceSignalGroup(state, makeGroup(TrafficLightElement::GREEN), t(0.0), 0.2, 0.1)),
    TrafficLightElement::GREEN);
  EXPECT_EQ(
    firstColor(debounceSignalGroup(state, makeGroup(TrafficLightElement::RED), t(0.1), 0.2, 0.1)),
    TrafficLightElement::GREEN);
  EXPECT_EQ(
    firstColor(debounceSignalGroup(state, makeGroup(TrafficLightElement::RED), t(0.2), 0.2, 0.1)),
    TrafficLightElement::GREEN);
  EXPECT_EQ(
    firstColor(debounceSignalGroup(state, makeGroup(TrafficLightElement::RED), t(0.3), 0.2, 0.1)),
    TrafficLightElement::RED);
}

TEST(PriorityUtils, StabilizeRestoresGoQuickerThanStop)
{
  SignalStabilizeState state;
  ASSERT_EQ(
    firstColor(debounceSignalGroup(state, makeGroup(TrafficLightElement::RED), t(0.0), 0.2, 0.1)),
    TrafficLightElement::RED);
  EXPECT_EQ(
    firstColor(
      debounceSignalGroup(state, makeGroup(TrafficLightElement::GREEN), t(0.05), 0.2, 0.1)),
    TrafficLightElement::RED);
  EXPECT_EQ(
    firstColor(
      debounceSignalGroup(state, makeGroup(TrafficLightElement::GREEN), t(0.15), 0.2, 0.1)),
    TrafficLightElement::GREEN);
}

}  // namespace
}  // namespace autoware::map_based_prediction::priority_predictor
