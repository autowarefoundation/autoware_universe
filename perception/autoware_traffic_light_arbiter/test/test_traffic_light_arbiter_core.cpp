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

#include "autoware/traffic_light_arbiter/traffic_light_arbiter_core.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/LineStringOrPolygon.h>
#include <lanelet2_core/primitives/Point.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{

using autoware::traffic_light::SourcePriority;
using autoware::traffic_light::TrafficLightArbiterCore;
using PredictedTrafficLightState = autoware_perception_msgs::msg::PredictedTrafficLightState;
using TrafficLightElement = autoware_perception_msgs::msg::TrafficLightElement;
using TrafficLightGroup = autoware_perception_msgs::msg::TrafficLightGroup;
using TrafficLightGroupArray = autoware_perception_msgs::msg::TrafficLightGroupArray;

// --- Short aliases for enum values used throughout the tests ----------
//
// Anonymous-namespace scoped so they don't leak; keeps each test's reading
// flow on color/shape/priority rather than on the enum-qualifying prefix.

constexpr auto CONFIDENCE = SourcePriority::CONFIDENCE;
constexpr auto EXTERNAL = SourcePriority::EXTERNAL;
constexpr auto PERCEPTION = SourcePriority::PERCEPTION;

constexpr uint8_t RED = TrafficLightElement::RED;
constexpr uint8_t GREEN = TrafficLightElement::GREEN;
constexpr uint8_t UNKNOWN = TrafficLightElement::UNKNOWN;
constexpr uint8_t CIRCLE = TrafficLightElement::CIRCLE;
constexpr uint8_t RIGHT_ARROW = TrafficLightElement::RIGHT_ARROW;

const std::string & INTERNAL_ESTIMATION =
  PredictedTrafficLightState::INFORMATION_SOURCE_INTERNAL_ESTIMATION;
const std::string & V2I = PredictedTrafficLightState::INFORMATION_SOURCE_V2I;

// --- Map construction --------------------------------------------------

lanelet::Point3d make_point(lanelet::Id id, double x, double y, double z = 0.0)
{
  return lanelet::Point3d(id, x, y, z);
}

lanelet::Lanelet make_lanelet(double y_left, double y_right, const std::string & subtype)
{
  using namespace lanelet;  // NOLINT(build/namespaces)
  LineString3d left(
    utils::getId(),
    {make_point(utils::getId(), 0.0, y_left), make_point(utils::getId(), 10.0, y_left)});
  LineString3d right(
    utils::getId(),
    {make_point(utils::getId(), 0.0, y_right), make_point(utils::getId(), 10.0, y_right)});
  Lanelet ll(utils::getId(), left, right);
  ll.attributes()[AttributeName::Subtype] = subtype;
  ll.attributes()[AttributeName::Type] = AttributeValueString::Lanelet;
  return ll;
}

lanelet::LineString3d make_traffic_light_bulb()
{
  // Geometry does not affect arbiter logic; pick lanelet midpoint at z=5.
  constexpr double bulb_x = 5.0;
  constexpr double bulb_z = 5.0;
  using namespace lanelet;  // NOLINT(build/namespaces)
  LineString3d bulb(
    utils::getId(), {make_point(utils::getId(), bulb_x, 0.0, bulb_z),
                     make_point(utils::getId(), bulb_x, 1.0, bulb_z)});
  bulb.attributes()[AttributeName::Type] = AttributeValueString::TrafficLight;
  return bulb;
}

namespace map_ids
{
constexpr lanelet::Id vehicle_a = 1001;
constexpr lanelet::Id vehicle_b = 1002;
constexpr lanelet::Id pedestrian = 2001;
constexpr lanelet::Id off_map_probe = 9999;
}  // namespace map_ids

// Two roads + one crosswalk, each carrying a Traffic Light regulatory
// element. The crosswalk row is what makes set_map() expose a pedestrian
// id to the arbiter; without it the pedestrian-specific reconciliation
// rule is unreachable.
//
//   Lanelet      Traffic Light id
//   ----------   ----------------------
//   crosswalk    2001 (pedestrian)
//   road2        1002 (vehicle_b)
//   road1        1001 (vehicle_a)
//
// Id 9999 (off_map_probe) is intentionally absent from the map so the
// arbiter drops it as off-map.
lanelet::LaneletMapConstPtr build_minimal_map()
{
  using namespace lanelet;  // NOLINT(build/namespaces)
  Lanelet road1 = make_lanelet(0.0, -3.0, AttributeValueString::Road);
  Lanelet road2 = make_lanelet(4.0, 1.0, AttributeValueString::Road);
  Lanelet crosswalk = make_lanelet(8.0, 5.0, AttributeValueString::Crosswalk);
  road1.addRegulatoryElement(
    TrafficLight::make(map_ids::vehicle_a, {}, LineStringsOrPolygons3d{make_traffic_light_bulb()}));
  road2.addRegulatoryElement(
    TrafficLight::make(map_ids::vehicle_b, {}, LineStringsOrPolygons3d{make_traffic_light_bulb()}));
  crosswalk.addRegulatoryElement(
    TrafficLight::make(
      map_ids::pedestrian, {}, LineStringsOrPolygons3d{make_traffic_light_bulb()}));
  return LaneletMapConstPtr{utils::createMap(Lanelets{road1, road2, crosswalk}).release()};
}

// Empty LaneletMap (no lanelets, no regulatory elements). Exercises the
// "map present but contains no signals" case (engaged but empty output).
lanelet::LaneletMapConstPtr build_empty_map()
{
  using namespace lanelet;  // NOLINT(build/namespaces)
  return LaneletMapConstPtr{utils::createMap(Lanelets{}).release()};
}

// --- Input builders ----------------------------------------------------

TrafficLightElement make_element(uint8_t color, uint8_t shape, float confidence = 1.0f)
{
  TrafficLightElement element;
  element.color = color;
  element.shape = shape;
  element.status = TrafficLightElement::SOLID_ON;
  element.confidence = confidence;
  return element;
}

// Predictions ride unmodified through arbitrate(), so the body fields
// chosen here only need to be syntactically valid.
PredictedTrafficLightState make_prediction(
  const rclcpp::Time & stamp, const std::string & source = INTERNAL_ESTIMATION)
{
  constexpr int32_t prediction_offset_sec = 10;
  constexpr float full_certainty = 1.0f;
  PredictedTrafficLightState prediction;
  prediction.predicted_stamp = stamp;
  prediction.predicted_stamp.sec += prediction_offset_sec;
  prediction.simultaneous_elements.push_back(make_element(UNKNOWN, CIRCLE, full_certainty));
  prediction.reliability = full_certainty;
  prediction.information_source = source;
  return prediction;
}

// Builds a one-group TrafficLightGroupArray. Each test exercises one id per
// ingest call, so collapsing the previous make_signals + make_group pair
// into a single flat helper removes a verbose layer of nested braces at
// every call site.
TrafficLightGroupArray make_signal(
  const rclcpp::Time & stamp, lanelet::Id id, std::vector<TrafficLightElement> elements,
  std::vector<PredictedTrafficLightState> predictions = {})
{
  TrafficLightGroup group;
  group.traffic_light_group_id = id;
  group.elements = std::move(elements);
  group.predictions = std::move(predictions);

  TrafficLightGroupArray signals;
  signals.stamp = stamp;
  signals.traffic_light_groups = {std::move(group)};
  return signals;
}

// --- Assert helpers ----------------------------------------------------
//
// The Core's arbitrate() returns an ArbitrationResult whose `output` may be
// std::nullopt (e.g. when no map was supplied). Helpers take that optional
// directly so callers don't need a separate ASSERT_TRUE guard before each
// observation — the helpers fold the nullopt case into their return.

const TrafficLightGroup * find_group(
  const std::optional<TrafficLightGroupArray> & output, lanelet::Id id)
{
  if (!output) {
    return nullptr;
  }
  for (const auto & group : output->traffic_light_groups) {
    if (group.traffic_light_group_id == id) {
      return &group;
    }
  }
  return nullptr;
}

const TrafficLightElement * find_element(const TrafficLightGroup & group, uint8_t shape)
{
  for (const auto & element : group.elements) {
    if (element.shape == shape) {
      return &element;
    }
  }
  return nullptr;
}

std::optional<uint8_t> observed_color(
  const std::optional<TrafficLightGroupArray> & output, lanelet::Id id)
{
  const auto * group = find_group(output, id);
  if (!group || group->elements.empty()) {
    return std::nullopt;
  }
  return group->elements[0].color;
}

std::optional<float> observed_confidence(
  const std::optional<TrafficLightGroupArray> & output, lanelet::Id id)
{
  const auto * group = find_group(output, id);
  if (!group || group->elements.empty()) {
    return std::nullopt;
  }
  return group->elements[0].confidence;
}

std::optional<std::size_t> observed_group_count(
  const std::optional<TrafficLightGroupArray> & output)
{
  if (!output) {
    return std::nullopt;
  }
  return output->traffic_light_groups.size();
}

std::optional<uint8_t> observed_color_of_shape(
  const std::optional<TrafficLightGroupArray> & output, lanelet::Id id, uint8_t shape)
{
  const auto * group = find_group(output, id);
  if (!group) {
    return std::nullopt;
  }
  const auto * element = find_element(*group, shape);
  if (!element) {
    return std::nullopt;
  }
  return element->color;
}

std::optional<std::string> observed_prediction_source(
  const std::optional<TrafficLightGroupArray> & output, lanelet::Id id, std::size_t index)
{
  const auto * group = find_group(output, id);
  if (!group || index >= group->predictions.size()) {
    return std::nullopt;
  }
  return group->predictions[index].information_source;
}

// --- Suite-wide state and factory -------------------------------------

// Mirror config/traffic_light_arbiter.param.yaml defaults so the numeric
// envelope these tests assume matches the production parameter set.
constexpr double default_external_delay_tolerance = 5.0;
constexpr double default_external_time_tolerance = 5.0;
constexpr double default_perception_time_tolerance = 1.0;

// Arbitrary positive seconds used as the suite-wide base time. Far enough
// from zero that any base_time - duration derived in a test stays positive
// (largest backward offset is default_external_delay_tolerance + a few seconds).
constexpr int32_t base_time_seconds = 1'000'000'000;
const rclcpp::Time base_time{base_time_seconds, 0, RCL_ROS_TIME};

// Static initializer reserves off_map_probe so utils::getId() (used by map
// builders) never hands out an id that collides with it, then builds the
// suite-wide map once. Process-global state; one registration is enough.
const lanelet::LaneletMapConstPtr shared_map = []() {
  lanelet::utils::registerId(map_ids::off_map_probe);
  return build_minimal_map();
}();

// Builds an arbiter with the suite-wide map already loaded. priority and
// enable_signal_matching select the reconciliation mode under test.
TrafficLightArbiterCore make_arbiter(SourcePriority priority, bool enable_signal_matching)
{
  TrafficLightArbiterCore arbiter(
    priority, enable_signal_matching, default_external_delay_tolerance,
    default_external_time_tolerance, default_perception_time_tolerance);
  arbiter.set_map(shared_map);
  return arbiter;
}

// ---------------------------------------------------------------------------
// Mode A: Signal Matching (enable_signal_matching=true)
//
// The arbiter reconciles perception and external signals: agreement passes
// through, disagreement falls back to UNKNOWN. These tests pin the non-
// pedestrian behaviour; pedestrian ids follow a different rule (see the
// pedestrian section below).
// ---------------------------------------------------------------------------

// Matched: when perception and external agree on color and shape, the
// output carries the same element verbatim.
TEST(TrafficLightArbiterCoreSignalMatching, matchedSignalsPassThrough)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/true);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)}), base_time);
  arbiter.ingest_perception(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), RED);
}

// Agreement is decided by color/shape only — confidence does not factor
// in. On a match the perception side flows through, so its confidence
// (0.90) rather than external's (0.10) reaches the output. Seeing 0.90
// (and not 0.10) on a same-color/shape pair is what proves the differing
// confidences did not break the match.
TEST(TrafficLightArbiterCoreSignalMatching, differingConfidencesStillMatch)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/true);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE, 0.10f)}), base_time);
  arbiter.ingest_perception(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE, 0.90f)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), RED);
  EXPECT_NEAR(observed_confidence(result.output, map_ids::vehicle_a).value_or(-1.0f), 0.90f, 1e-5f);
}

// Color mismatch: when both sides share the shape but disagree on color,
// the output falls back to UNKNOWN over that shape.
TEST(TrafficLightArbiterCoreSignalMatching, colorMismatchProducesUnknown)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/true);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_b, {make_element(GREEN, CIRCLE)}), base_time);
  arbiter.ingest_perception(
    make_signal(base_time, map_ids::vehicle_b, {make_element(RED, CIRCLE)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_b), UNKNOWN);
}

// Shape-set mismatch: perception has CIRCLE + RIGHT_ARROW, external has
// only CIRCLE → the output is UNKNOWN over the shape union.
TEST(TrafficLightArbiterCoreSignalMatching, shapeSetMismatchProducesUnknown)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/true);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_b, {make_element(RED, CIRCLE)}), base_time);
  arbiter.ingest_perception(make_signal(
    base_time, map_ids::vehicle_b, {make_element(RED, CIRCLE), make_element(GREEN, RIGHT_ARROW)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color_of_shape(result.output, map_ids::vehicle_b, CIRCLE), UNKNOWN);
  EXPECT_EQ(observed_color_of_shape(result.output, map_ids::vehicle_b, RIGHT_ARROW), UNKNOWN);
}

// Perception-only single source: with no external to agree with, the
// output for that id becomes UNKNOWN. Pedestrian ids would pass through
// unchanged instead — see singleSourcePedestrianPassesThrough.
TEST(TrafficLightArbiterCoreSignalMatching, perceptionOnlySingleSourceYieldsUnknown)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/true);

  arbiter.ingest_perception(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), UNKNOWN);
}

// External-only single source: symmetric counterpart — reconciliation is
// direction-symmetric, so the output for that id is also UNKNOWN when
// perception is the missing side.
TEST(TrafficLightArbiterCoreSignalMatching, externalOnlySingleSourceYieldsUnknown)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/true);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)}), base_time);

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), UNKNOWN);
}

// Off-map id is dropped from the output and recorded in off_map_signal_ids.
TEST(TrafficLightArbiterCoreSignalMatching, offMapIdIsDroppedAndReported)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/true);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::off_map_probe, {make_element(RED, CIRCLE)}), base_time);

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(find_group(result.output, map_ids::off_map_probe), nullptr);
  EXPECT_EQ(result.off_map_signal_ids, std::vector<lanelet::Id>{map_ids::off_map_probe});
}

// ---------------------------------------------------------------------------
// Signal Matching — Pedestrian ids
//
// Pedestrian ids reconcile differently from vehicle ids: instead of
// requiring color/shape agreement between sources, the arbiter picks a
// winner per the source-priority setting.
// ---------------------------------------------------------------------------

// EXTERNAL priority: for pedestrian ids the external side wins even when
// its confidence is lower than perception's.
TEST(TrafficLightArbiterCorePedestrian, externalPriorityWinsForPedestrian)
{
  auto arbiter = make_arbiter(EXTERNAL, /*enable_signal_matching=*/true);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::pedestrian, {make_element(RED, CIRCLE, 0.1f)}), base_time);
  arbiter.ingest_perception(
    make_signal(base_time, map_ids::pedestrian, {make_element(GREEN, CIRCLE, 0.9f)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::pedestrian), RED);
}

// PERCEPTION priority: symmetric counterpart — for pedestrian ids the
// perception side wins even when its confidence is lower than external's.
TEST(TrafficLightArbiterCorePedestrian, perceptionPriorityWinsForPedestrian)
{
  auto arbiter = make_arbiter(PERCEPTION, /*enable_signal_matching=*/true);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::pedestrian, {make_element(RED, CIRCLE, 0.99f)}), base_time);
  arbiter.ingest_perception(
    make_signal(base_time, map_ids::pedestrian, {make_element(GREEN, CIRCLE, 0.10f)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::pedestrian), GREEN);
}

// CONFIDENCE mode: for pedestrian ids the arbiter walks each shape and
// picks the side with the higher confidence.
TEST(TrafficLightArbiterCorePedestrian, confidenceModePicksHigherConfidenceForPedestrian)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/true);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::pedestrian, {make_element(GREEN, CIRCLE, 0.6f)}), base_time);
  arbiter.ingest_perception(
    make_signal(base_time, map_ids::pedestrian, {make_element(RED, CIRCLE, 0.9f)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::pedestrian), RED);
}

// Single-source pedestrian: contrast with the non-pedestrian behaviour
// (perceptionOnlySingleSourceYieldsUnknown / externalOnlySingleSourceYieldsUnknown).
// For pedestrian ids the present side flows through unchanged with no
// UNKNOWN translation, even when source_priority would otherwise prefer
// the absent side.
TEST(TrafficLightArbiterCorePedestrian, singleSourcePedestrianPassesThrough)
{
  auto arbiter = make_arbiter(EXTERNAL, /*enable_signal_matching=*/true);

  arbiter.ingest_perception(
    make_signal(base_time, map_ids::pedestrian, {make_element(GREEN, CIRCLE)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::pedestrian), GREEN);
}

// ---------------------------------------------------------------------------
// Mode B: Priority-based — CONFIDENCE (enable_signal_matching=false)
// ---------------------------------------------------------------------------

// CONFIDENCE: same shape from both sources → the higher-confidence element
// wins (here, perception 0.9 over external 0.7).
TEST(TrafficLightArbiterCoreConfidencePriority, picksHigherConfidenceElement)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(GREEN, CIRCLE, 0.7f)}), base_time);
  arbiter.ingest_perception(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE, 0.9f)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), RED);
}

// Per-shape union: when each side contributes a different shape under
// the same id, the output carries both shapes verbatim. Reconciliation
// runs independently per shape, so the absence of CIRCLE on the external
// side does not block external's RIGHT_ARROW and vice versa.
TEST(TrafficLightArbiterCoreConfidencePriority, perShapeUnionFromBothSides)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  arbiter.ingest_perception(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)}));
  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(GREEN, RIGHT_ARROW)}), base_time);

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color_of_shape(result.output, map_ids::vehicle_a, CIRCLE), RED);
  EXPECT_EQ(observed_color_of_shape(result.output, map_ids::vehicle_a, RIGHT_ARROW), GREEN);
}

// Perception-only: with no external present, the perception element flows
// through unchanged regardless of priority mode.
TEST(TrafficLightArbiterCoreConfidencePriority, perceptionOnlyPassesThrough)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  arbiter.ingest_perception(
    make_signal(base_time, map_ids::vehicle_b, {make_element(GREEN, CIRCLE)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_b), GREEN);
}

// Off-map id is dropped before reaching priority selection too. Pins the
// off-map guard as mode-independent (Signal Matching side has its own
// counterpart in offMapIdIsDroppedAndReported).
TEST(TrafficLightArbiterCoreConfidencePriority, offMapIdIsDroppedAndReported)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::off_map_probe, {make_element(RED, CIRCLE)}), base_time);

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(find_group(result.output, map_ids::off_map_probe), nullptr);
  EXPECT_EQ(result.off_map_signal_ids, std::vector<lanelet::Id>{map_ids::off_map_probe});
}

// Successive external publishes carrying different ids accumulate in the
// cache; both end up in the final output with their published colors.
TEST(TrafficLightArbiterCoreConfidencePriority, multipleExternalSourcesAccumulate)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(GREEN, CIRCLE)}), base_time);
  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_b, {make_element(RED, CIRCLE)}), base_time);

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), GREEN);
  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_b), RED);
}

// Predictions ride alongside elements: when both sides carry one each,
// the output group holds both, perception-side first. Mode is irrelevant
// — the merge happens before mode-specific element reconciliation, so
// Confidence is a fine host for this pin.
TEST(TrafficLightArbiterCoreConfidencePriority, predictionsFromBothSidesAreMerged)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  arbiter.ingest_perception(make_signal(
    base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)},
    {make_prediction(base_time, INTERNAL_ESTIMATION)}));
  arbiter.ingest_external(
    make_signal(
      base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)},
      {make_prediction(base_time, V2I)}),
    base_time);

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_prediction_source(result.output, map_ids::vehicle_a, 0), INTERNAL_ESTIMATION);
  EXPECT_EQ(observed_prediction_source(result.output, map_ids::vehicle_a, 1), V2I);
}

// Perception-only side: when external is silent, the perception side's
// prediction (with its information_source) still reaches the output.
TEST(TrafficLightArbiterCoreConfidencePriority, perceptionOnlyPredictionPropagates)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  arbiter.ingest_perception(make_signal(
    base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)},
    {make_prediction(base_time, INTERNAL_ESTIMATION)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_prediction_source(result.output, map_ids::vehicle_a, 0), INTERNAL_ESTIMATION);
}

// External-only side: symmetric counterpart — when perception is silent,
// the external side's prediction (with its information_source) reaches
// the output.
TEST(TrafficLightArbiterCoreConfidencePriority, externalOnlyPredictionPropagates)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  arbiter.ingest_external(
    make_signal(
      base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)},
      {make_prediction(base_time, V2I)}),
    base_time);

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_prediction_source(result.output, map_ids::vehicle_a, 0), V2I);
}

// ---------------------------------------------------------------------------
// Mode C: Priority-based — EXTERNAL
// ---------------------------------------------------------------------------

// External wins despite lower confidence: the EXTERNAL setting forces
// the external side to win over a higher-confidence perception.
TEST(TrafficLightArbiterCoreExternalPriority, externalPriorityOverridesConfidence)
{
  auto arbiter = make_arbiter(EXTERNAL, /*enable_signal_matching=*/false);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE, 0.10f)}), base_time);
  arbiter.ingest_perception(
    make_signal(base_time, map_ids::vehicle_a, {make_element(GREEN, CIRCLE, 0.99f)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), RED);
}

// External-only: the EXTERNAL setting has no effect when only one source
// is present.
TEST(TrafficLightArbiterCoreExternalPriority, externalOnlyPassesThrough)
{
  auto arbiter = make_arbiter(EXTERNAL, /*enable_signal_matching=*/false);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)}), base_time);

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), RED);
}

// Perception-only with EXTERNAL priority: with no external side present,
// the EXTERNAL setting has nothing to apply to and the perception element
// reaches the output.
TEST(TrafficLightArbiterCoreExternalPriority, perceptionOnlyPassesThrough)
{
  auto arbiter = make_arbiter(EXTERNAL, /*enable_signal_matching=*/false);

  arbiter.ingest_perception(
    make_signal(base_time, map_ids::vehicle_a, {make_element(GREEN, CIRCLE)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), GREEN);
}

// ---------------------------------------------------------------------------
// Mode D: Priority-based — PERCEPTION
// ---------------------------------------------------------------------------

// Perception wins despite lower confidence: symmetric counterpart of the
// EXTERNAL override case.
TEST(TrafficLightArbiterCorePerceptionPriority, perceptionPriorityOverridesConfidence)
{
  auto arbiter = make_arbiter(PERCEPTION, /*enable_signal_matching=*/false);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE, 0.99f)}), base_time);
  arbiter.ingest_perception(
    make_signal(base_time, map_ids::vehicle_a, {make_element(GREEN, CIRCLE, 0.10f)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), GREEN);
}

// Perception-only: the PERCEPTION setting has no effect when only one
// source is present.
TEST(TrafficLightArbiterCorePerceptionPriority, perceptionOnlyPassesThrough)
{
  auto arbiter = make_arbiter(PERCEPTION, /*enable_signal_matching=*/false);

  arbiter.ingest_perception(
    make_signal(base_time, map_ids::vehicle_a, {make_element(GREEN, CIRCLE)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), GREEN);
}

// External-only with PERCEPTION priority: symmetric counterpart — external
// is the only source, so it passes through.
TEST(TrafficLightArbiterCorePerceptionPriority, externalOnlyPassesThrough)
{
  auto arbiter = make_arbiter(PERCEPTION, /*enable_signal_matching=*/false);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)}), base_time);

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), RED);
}

// ---------------------------------------------------------------------------
// Boundary: map presence boundaries — built outside make_arbiter() because
// each test needs a non-default map configuration (no map, or empty map).
// ---------------------------------------------------------------------------

// arbitrate() before set_map() yields output=std::nullopt. The Node
// distinguishes "no output" (skip publish) from "empty output" (publish
// with zero groups), so the optional must stay disengaged here.
TEST(TrafficLightArbiterCoreBoundary, arbitrateWithoutMapProducesNoOutput)
{
  TrafficLightArbiterCore unconfigured(
    CONFIDENCE, /*enable_signal_matching=*/false, default_external_delay_tolerance,
    default_external_time_tolerance, default_perception_time_tolerance);

  unconfigured.ingest_perception(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)}));

  const auto result = unconfigured.arbitrate();

  EXPECT_FALSE(result.output.has_value());
}

// A map with no TrafficLight regulatory elements yields an output that is
// engaged but empty — the counterpart to the no-map case above. The Node
// publishes a zero-group message here instead of skipping the publish.
TEST(TrafficLightArbiterCoreBoundary, emptyMapProducesEmptyOutput)
{
  TrafficLightArbiterCore arbiter(
    CONFIDENCE, /*enable_signal_matching=*/false, default_external_delay_tolerance,
    default_external_time_tolerance, default_perception_time_tolerance);
  arbiter.set_map(build_empty_map());

  arbiter.ingest_perception(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_group_count(result.output), 0u);
}

// ---------------------------------------------------------------------------
// Perception staleness: when perception's stamp lags the freshest external
// stamp by more than perception_time_tolerance, arbitrate() drops the
// perception side from the current cycle without modifying what has been
// ingested. Tests pin the boundary both inside and outside the tolerance,
// plus the prediction side effect.
// ---------------------------------------------------------------------------

// Outside tolerance (perception is older than tolerance allows): perception
// is excluded from arbitrate() so external alone reaches the output.
TEST(TrafficLightArbiterCorePerceptionStaleness, stalePerceptionIsExcludedFromOutput)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  const auto t_perception =
    base_time - rclcpp::Duration::from_seconds(default_perception_time_tolerance + 1.0);

  arbiter.ingest_perception(
    make_signal(t_perception, map_ids::vehicle_a, {make_element(RED, CIRCLE)}));
  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(GREEN, CIRCLE)}), base_time);

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), GREEN);
}

// Inside tolerance (perception is older but only by half the budget):
// perception remains effective, so CONFIDENCE picks the higher-confidence
// perception element over the lower-confidence external.
TEST(TrafficLightArbiterCorePerceptionStaleness, freshPerceptionRemainsEffective)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  const auto t_perception =
    base_time - rclcpp::Duration::from_seconds(0.5 * default_perception_time_tolerance);

  arbiter.ingest_perception(
    make_signal(t_perception, map_ids::vehicle_a, {make_element(RED, CIRCLE, 0.9f)}));
  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(GREEN, CIRCLE, 0.7f)}), base_time);

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), RED);
}

// External is silent: with no external stamp to compare against, the
// staleness check has nothing to evaluate and the perception side
// reaches the output even when its stamp lags base_time by far more
// than the tolerance.
//
// This asymmetry predates the Core extraction — only the external side
// has ever had a wall-clock admission guard (added in 3dc9605d9 to the
// combined Node+logic code; the perception side was not given the same
// treatment then and has not been since). Whether that is intentional
// (Core is a pure function; absolute staleness is a Node-layer concern)
// or a latent gap (a stalled perception node could keep stale signals
// on the bus indefinitely when no external source is publishing) is an
// open question. This test pins the current behaviour; it does not
// endorse it.
TEST(TrafficLightArbiterCorePerceptionStaleness, perceptionAloneIsNeverStale)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  const auto t_perception =
    base_time - rclcpp::Duration::from_seconds(default_perception_time_tolerance + 10.0);

  arbiter.ingest_perception(
    make_signal(t_perception, map_ids::vehicle_a, {make_element(RED, CIRCLE)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_color(result.output, map_ids::vehicle_a), RED);
}

// Staleness is decided at the message level, not per id: when the
// perception message is stale, its entire content is excluded — even ids
// that the external side does not cover. With perception carrying
// vehicle_a (stale) and external carrying vehicle_b (fresh), vehicle_a
// must be absent from the output and vehicle_b must remain present.
TEST(TrafficLightArbiterCorePerceptionStaleness, stalenessDropsEntirePerceptionMessage)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  const auto t_perception =
    base_time - rclcpp::Duration::from_seconds(default_perception_time_tolerance + 1.0);

  arbiter.ingest_perception(
    make_signal(t_perception, map_ids::vehicle_a, {make_element(RED, CIRCLE)}));
  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_b, {make_element(GREEN, CIRCLE)}), base_time);

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(find_group(result.output, map_ids::vehicle_a), nullptr);
  EXPECT_NE(find_group(result.output, map_ids::vehicle_b), nullptr);
}

// Stale perception drops its predictions from the merge too, so only the
// external-side prediction (V2I) survives in the output group.
TEST(TrafficLightArbiterCorePerceptionStaleness, stalePerceptionPredictionsAreSkipped)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  const auto t_perception =
    base_time - rclcpp::Duration::from_seconds(default_perception_time_tolerance + 1.0);

  arbiter.ingest_perception(make_signal(
    t_perception, map_ids::vehicle_a, {make_element(RED, CIRCLE)},
    {make_prediction(t_perception, INTERNAL_ESTIMATION)}));
  arbiter.ingest_external(
    make_signal(
      base_time, map_ids::vehicle_a, {make_element(GREEN, CIRCLE)},
      {make_prediction(base_time, V2I)}),
    base_time);

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(observed_prediction_source(result.output, map_ids::vehicle_a, 0), V2I);
}

// ---------------------------------------------------------------------------
// latest_input_time: the Node uses this to decide whether its published
// output is behind some input that has arrived but not yet driven a
// publish cycle. The non-trivial piece is selecting the most recent
// stamp across stored sources, so we pin all directions of that
// selection.
// ---------------------------------------------------------------------------

TEST(TrafficLightArbiterCoreLatestInputTime, takesNewerOfPerceptionAndExternal)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  const auto t_external_newer = base_time + rclcpp::Duration::from_seconds(0.5);

  arbiter.ingest_perception(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)}));
  arbiter.ingest_external(
    make_signal(t_external_newer, map_ids::vehicle_a, {make_element(RED, CIRCLE)}),
    t_external_newer);

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(result.latest_input_time, t_external_newer);
}

// Symmetric counterpart: when perception is the newer source, its stamp
// wins even though external is also stored.
TEST(TrafficLightArbiterCoreLatestInputTime, perceptionWinsWhenNewer)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  const auto t_perception_newer = base_time + rclcpp::Duration::from_seconds(0.3);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)}), base_time);
  arbiter.ingest_perception(
    make_signal(t_perception_newer, map_ids::vehicle_a, {make_element(RED, CIRCLE)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(result.latest_input_time, t_perception_newer);
}

// External is silent: with no external stamps to compare against,
// latest_input_time falls back to the perception stamp.
TEST(TrafficLightArbiterCoreLatestInputTime, perceptionStampWhenExternalIsSilent)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  const auto t_perception = base_time + rclcpp::Duration::from_seconds(0.5);

  arbiter.ingest_perception(
    make_signal(t_perception, map_ids::vehicle_a, {make_element(RED, CIRCLE)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(result.latest_input_time, t_perception);
}

// ---------------------------------------------------------------------------
// Timing utilities: mode-agnostic. Behaviour is observed through
// ingest_external's admission-control return (accepted/rejected) and
// ingest_perception's expired-entries return.
// ---------------------------------------------------------------------------

// A stamp close enough to current_time (within external_delay_tolerance)
// is accepted.
TEST(TrafficLightArbiterCoreTiming, ingestExternalAcceptsStampsWithinTolerance)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  const auto t_past_within =
    base_time - rclcpp::Duration::from_seconds(default_external_delay_tolerance - 1.0);

  const auto result = arbiter.ingest_external(
    make_signal(t_past_within, map_ids::vehicle_a, {make_element(RED, CIRCLE)}), base_time);

  EXPECT_TRUE(result.accepted);
}

// A stamp older than current_time by more than external_delay_tolerance
// is rejected.
TEST(TrafficLightArbiterCoreTiming, ingestExternalRejectsTooOldStamps)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  const auto t_past_beyond =
    base_time - rclcpp::Duration::from_seconds(default_external_delay_tolerance + 1.0);

  const auto result = arbiter.ingest_external(
    make_signal(t_past_beyond, map_ids::vehicle_a, {make_element(RED, CIRCLE)}), base_time);

  EXPECT_FALSE(result.accepted);
}

// A stamp ahead of current_time by more than external_delay_tolerance is
// also rejected — the tolerance is applied symmetrically in both
// directions.
TEST(TrafficLightArbiterCoreTiming, ingestExternalRejectsTooFutureStamps)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  const auto t_future_beyond =
    base_time + rclcpp::Duration::from_seconds(default_external_delay_tolerance + 1.0);

  const auto result = arbiter.ingest_external(
    make_signal(t_future_beyond, map_ids::vehicle_a, {make_element(RED, CIRCLE)}), base_time);

  EXPECT_FALSE(result.accepted);
}

// Sweep-on-ingest report: a perception arrival far past
// external_time_tolerance after a stored external evicts that entry,
// and ingest_perception's return value carries the eviction details
// (used by the Node for DEBUG logging).
TEST(TrafficLightArbiterCoreTiming, ingestPerceptionReportsExpiredExternalEntry)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)}), base_time);
  const auto t_perception =
    base_time + rclcpp::Duration::from_seconds(default_external_time_tolerance + 1.0);

  const auto expired = arbiter.ingest_perception(
    make_signal(t_perception, map_ids::vehicle_b, {make_element(GREEN, CIRCLE)}));

  ASSERT_EQ(expired.size(), 1u);
  EXPECT_EQ(expired.front().id, map_ids::vehicle_a);
  EXPECT_GT(expired.front().age, default_external_time_tolerance);
}

// Sweep-on-ingest downstream effect: an external entry evicted by the
// sweep is absent from the next arbitration output, even when the
// triggering perception covers a different id.
TEST(TrafficLightArbiterCoreTiming, evictedExternalEntryAbsentFromOutput)
{
  auto arbiter = make_arbiter(CONFIDENCE, /*enable_signal_matching=*/false);

  arbiter.ingest_external(
    make_signal(base_time, map_ids::vehicle_a, {make_element(RED, CIRCLE)}), base_time);
  const auto t_perception =
    base_time + rclcpp::Duration::from_seconds(default_external_time_tolerance + 1.0);
  arbiter.ingest_perception(
    make_signal(t_perception, map_ids::vehicle_b, {make_element(GREEN, CIRCLE)}));

  const auto result = arbiter.arbitrate();

  EXPECT_EQ(find_group(result.output, map_ids::vehicle_a), nullptr);
}

}  // namespace
