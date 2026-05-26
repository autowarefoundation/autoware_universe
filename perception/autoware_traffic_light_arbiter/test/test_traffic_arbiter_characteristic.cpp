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

#include "autoware/traffic_light_arbiter/traffic_light_arbiter.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/LineStringOrPolygon.h>
#include <lanelet2_core/primitives/Point.h>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

using autoware::traffic_light::TrafficLightArbiter;
using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
using TrafficLightGroupArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
using TrafficLightGroup = autoware_perception_msgs::msg::TrafficLightGroup;
using TrafficLightElement = autoware_perception_msgs::msg::TrafficLightElement;
using PredictedTrafficLightState = autoware_perception_msgs::msg::PredictedTrafficLightState;
using namespace lanelet;  // NOLINT(build/namespaces)
using utils::getId;

// --- Signal IDs --------------------------------------------------------

// Traffic-light regulatory-element IDs that are installed on the test map.
// Kept explicit (rather than auto-allocated) so that values appearing in
// published TrafficLightGroupArray messages and warning logs stay readable.
namespace map_ids
{
constexpr lanelet::Id vehicle_signal_a = 1001;
constexpr lanelet::Id vehicle_signal_b = 1002;
constexpr lanelet::Id vehicle_signal_c = 1003;
constexpr lanelet::Id pedestrian_signal = 2001;
}  // namespace map_ids

// Intentionally not installed on the map; used by tests as the "off-map id"
// probe to exercise the WARN+skip branch in arbitrateAndPublish.
constexpr lanelet::Id kOffMapProbeId = 9999;

// --- Map construction --------------------------------------------------

Point3d makePoint(lanelet::Id id, double x, double y, double z = 0.0)
{
  return Point3d(id, x, y, z);
}

Lanelet makeLanelet(double y_left, double y_right, const std::string & subtype)
{
  LineString3d left(getId(), {makePoint(getId(), 0.0, y_left), makePoint(getId(), 10.0, y_left)});
  LineString3d right(
    getId(), {makePoint(getId(), 0.0, y_right), makePoint(getId(), 10.0, y_right)});
  Lanelet new_lanelet(getId(), left, right);
  new_lanelet.attributes()[AttributeName::Subtype] = subtype;
  new_lanelet.attributes()[AttributeName::Type] = AttributeValueString::Lanelet;
  return new_lanelet;
}

LineString3d makeTrafficLightBulb()
{
  // The exact position does not affect arbiter logic; we pick the lanelet
  // midpoint (x = 5.0 within the 0..10 span) at z = 5.0 above the ground.
  constexpr double kBulbX = 5.0;
  constexpr double kBulbZ = 5.0;
  LineString3d bulb(
    getId(), {makePoint(getId(), kBulbX, 0.0, kBulbZ), makePoint(getId(), kBulbX, 1.0, kBulbZ)});
  bulb.attributes()[AttributeName::Type] = AttributeValueString::TrafficLight;
  return bulb;
}

// Builds the minimal map required to drive every code path under test:
// three road lanelets each carrying one Traffic Light, plus one crosswalk
// lanelet carrying the pedestrian Traffic Light.
//
// Test map summary (top-down view).
// Each lanelet spans x = 0..10 along the travel direction; +y is the left side.
//
//   left bound  right bound   Lanelet      Traffic Light id
//   ----------  -----------   ----------   --------------------------
//        12          11       crosswalk    2001 (pedestrian_signal)
//         8           5       road3        1003 (vehicle_signal_c)
//         4           1       road2        1002 (vehicle_signal_b)
//         0          -3       road1        1001 (vehicle_signal_a)
//
// Probe id intentionally absent from the map: 9999 (kOffMapProbeId)
LaneletMapBin buildMinimalMapBin()
{
  Lanelet road1 = makeLanelet(0.0, -3.0, AttributeValueString::Road);
  Lanelet road2 = makeLanelet(4.0, 1.0, AttributeValueString::Road);
  Lanelet road3 = makeLanelet(8.0, 5.0, AttributeValueString::Road);
  Lanelet crosswalk = makeLanelet(12.0, 11.0, AttributeValueString::Crosswalk);

  road1.addRegulatoryElement(
    TrafficLight::make(
      map_ids::vehicle_signal_a, {}, LineStringsOrPolygons3d{makeTrafficLightBulb()}));
  road2.addRegulatoryElement(
    TrafficLight::make(
      map_ids::vehicle_signal_b, {}, LineStringsOrPolygons3d{makeTrafficLightBulb()}));
  road3.addRegulatoryElement(
    TrafficLight::make(
      map_ids::vehicle_signal_c, {}, LineStringsOrPolygons3d{makeTrafficLightBulb()}));
  crosswalk.addRegulatoryElement(
    TrafficLight::make(
      map_ids::pedestrian_signal, {}, LineStringsOrPolygons3d{makeTrafficLightBulb()}));

  const LaneletMapConstPtr map{
    utils::createMap(Lanelets{road1, road2, road3, crosswalk}).release()};
  auto bin = ::autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  bin.header.frame_id = "base_link";
  return bin;
}

// Variant of buildMinimalMapBin with a single road lanelet but no Traffic
// Light regulatory elements. Used to drive the
// "map_regulatory_elements_set_->empty()" branch in arbitrateAndPublish.
LaneletMapBin buildEmptyMapBin()
{
  Lanelet road = makeLanelet(0.0, -3.0, AttributeValueString::Road);
  const LaneletMapConstPtr map{utils::createMap(Lanelets{road}).release()};
  auto bin = ::autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  bin.header.frame_id = "base_link";
  return bin;
}

// --- Node construction -------------------------------------------------

// Default parameters mirror config/traffic_light_arbiter.param.yaml. We
// re-declare them here instead of reading the YAML so the test is fully
// self-contained and unaffected by production config changes. Only the
// two parameters that any test actually toggles are exposed as arguments.
std::shared_ptr<TrafficLightArbiter> makeArbiter(
  bool enable_signal_matching = false, const std::string & source_priority = "confidence")
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("external_delay_tolerance", 5.0),
    rclcpp::Parameter("external_time_tolerance", 5.0),
    rclcpp::Parameter("perception_time_tolerance", 1.0),
    rclcpp::Parameter("source_priority", source_priority),
    rclcpp::Parameter("enable_signal_matching", enable_signal_matching),
  });
  return std::make_shared<TrafficLightArbiter>(options);
}

// --- Input message builders --------------------------------------------

// Confidence defaults to 1.0f: most tests don't care about the value (the
// arbiter only consults it under the CONFIDENCE branch), so omitting the
// argument signals "this number is irrelevant to the assertion".
TrafficLightElement makeTrafficLightElement(uint8_t color, uint8_t shape, float confidence = 1.0f)
{
  TrafficLightElement element;
  element.color = color;
  element.shape = shape;
  element.status = TrafficLightElement::SOLID_ON;
  element.confidence = confidence;
  return element;
}

PredictedTrafficLightState makeTrafficLightPrediction(
  const builtin_interfaces::msg::Time & stamp, uint8_t color, uint8_t shape,
  const std::string & source)
{
  // The arbiter passes predictions through without inspecting any field
  // value, so the numbers below only need to form a syntactically complete
  // PredictedTrafficLightState.
  constexpr int32_t kPredictionOffsetSec = 10;
  constexpr float kFullCertainty = 1.0f;

  PredictedTrafficLightState prediction;
  prediction.predicted_stamp = stamp;
  prediction.predicted_stamp.sec += kPredictionOffsetSec;
  prediction.simultaneous_elements.push_back(makeTrafficLightElement(color, shape, kFullCertainty));
  prediction.reliability = kFullCertainty;
  prediction.information_source = source;
  return prediction;
}

TrafficLightGroup makeTrafficLightGroup(
  lanelet::Id id, std::vector<TrafficLightElement> elements,
  std::vector<PredictedTrafficLightState> predictions = {})
{
  TrafficLightGroup group;
  group.traffic_light_group_id = id;
  group.elements = std::move(elements);
  group.predictions = std::move(predictions);
  return group;
}

// --- Timestamp utility -------------------------------------------------

builtin_interfaces::msg::Time offsetTime(const rclcpp::Time & base, double seconds)
{
  return (base + rclcpp::Duration::from_seconds(seconds));
}

// --- Output inspection helpers -----------------------------------------

const TrafficLightGroup * findTrafficLightGroup(const TrafficLightGroupArray & msg, lanelet::Id id)
{
  for (const auto & group : msg.traffic_light_groups) {
    if (group.traffic_light_group_id == id) {
      return &group;
    }
  }
  return nullptr;
}

const TrafficLightElement * findTrafficLightElement(const TrafficLightGroup & group, uint8_t shape)
{
  for (const auto & element : group.elements) {
    if (element.shape == shape) {
      return &element;
    }
  }
  return nullptr;
}

// --- Test fixture ------------------------------------------------------

class ArbiterCharacteristic : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Reserve the highest signal id so utils::getId() never returns values
    // that collide with map_ids::* or kOffMapProbeId. Calling registerId(9999)
    // advances the global counter to >= 10000, which is safely above every
    // fixed signal id we use. This is process-wide global state, so doing it
    // once per suite is enough.
    utils::registerId(kOffMapProbeId);

    // map_bin_ is immutable across tests, so build it once for the suite.
    map_bin_ = buildMinimalMapBin();
  }
  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    test_node_ = std::make_shared<rclcpp::Node>("arbiter_characteristic_test");

    map_pub_ =
      test_node_->create_publisher<LaneletMapBin>(kMapTopic, rclcpp::QoS(1).transient_local());
    perception_pub_ =
      test_node_->create_publisher<TrafficLightGroupArray>(kPerceptionTopic, rclcpp::QoS(1));
    external_pub_ =
      test_node_->create_publisher<TrafficLightGroupArray>(kExternalTopic, rclcpp::QoS(1));

    arbitrated_traffic_signal_sub_ = test_node_->create_subscription<TrafficLightGroupArray>(
      kOutputTopic, rclcpp::QoS(1), [this](const TrafficLightGroupArray::ConstSharedPtr msg) {
        latest_arbitrated_traffic_signal_ = *msg;
        ++arbiter_publish_count_;
      });
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
    }
    executor_.reset();
    arbiter_.reset();
    arbitrated_traffic_signal_sub_.reset();
    map_pub_.reset();
    perception_pub_.reset();
    external_pub_.reset();
    test_node_.reset();
  }

  void startArbiter(bool enable_signal_matching, const std::string & source_priority)
  {
    arbiter_ = makeArbiter(enable_signal_matching, source_priority);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(arbiter_);
    executor_->add_node(test_node_);
    spinFor();
  }

  void publishMap() { publishMap(*map_bin_); }

  void publishMap(const LaneletMapBin & bin)
  {
    map_pub_->publish(bin);
    spinFor();
  }
  void publishPerception(const TrafficLightGroupArray & msg)
  {
    perception_pub_->publish(msg);
    spinFor();
  }
  void publishExternal(const TrafficLightGroupArray & msg)
  {
    external_pub_->publish(msg);
    spinFor();
  }

  void spinFor(std::chrono::milliseconds duration = std::chrono::milliseconds(150))
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // Wrapped in std::optional because LaneletMapBin (a generated ROS msg)
  // has no usable default constructor that can be invoked at static storage
  // duration; we initialise it during SetUpTestSuite().
  inline static std::optional<LaneletMapBin> map_bin_;

  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<TrafficLightArbiter> arbiter_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Publisher<LaneletMapBin>::SharedPtr map_pub_;
  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr perception_pub_;
  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr external_pub_;
  rclcpp::Subscription<TrafficLightGroupArray>::SharedPtr arbitrated_traffic_signal_sub_;

  TrafficLightGroupArray latest_arbitrated_traffic_signal_;
  std::size_t arbiter_publish_count_ = 0;

private:
  // Topic names used by the publishers/subscription owned by the fixture.
  static constexpr const char * kMapTopic = "/traffic_light_arbiter/sub/vector_map";
  static constexpr const char * kPerceptionTopic =
    "/traffic_light_arbiter/sub/perception_traffic_signals";
  static constexpr const char * kExternalTopic =
    "/traffic_light_arbiter/sub/external_traffic_signals";
  static constexpr const char * kOutputTopic = "/traffic_light_arbiter/pub/traffic_signals";

protected:
  // Tolerance used by TEST_F bodies when comparing confidence values via
  // EXPECT_NEAR. Kept inside the fixture because it is only meaningful for
  // tests that derive from this class.
  static constexpr float kConfidenceEpsilon = 1e-5f;
};

// ---------------------------------------------------------------------------
// A. Signal Matching mode - arbitration rules across source agreement.
//
// Pins the full Signal Matching behavior spec: the arbiter degrades
// gracefully as the agreement between sources weakens, falling back to
// UNKNOWN when the signals disagree and dropping ids that are not on the
// map. Each validation outcome of SignalMatchValidator is exercised by
// a dedicated test:
//   - matched (color & shape agree) -> perception passes through
//   - color mismatch                -> UNKNOWN over the shared shape
//   - element-count mismatch        -> UNKNOWN over the shape union
//   - off-map id                    -> dropped (WARN+skip)
// (Confidence is omitted in element constructors because Signal Matching
// never reads it; the 1.0f default applies.)
// ---------------------------------------------------------------------------

// Matched: external and perception agree on color and shape. Perception
// passes through verbatim, and predictions from both sides are merged.
TEST_F(ArbiterCharacteristic, signalMatchingMatchedPassesThrough)
{
  // Arrange
  startArbiter(true, "confidence");  // signal matching mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)},
    {makeTrafficLightPrediction(
      external_traffic_signal.stamp, TrafficLightElement::GREEN, TrafficLightElement::CIRCLE,
      PredictedTrafficLightState::INFORMATION_SOURCE_V2I)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)},
    {makeTrafficLightPrediction(
      perception_traffic_signal.stamp, TrafficLightElement::RED, TrafficLightElement::CIRCLE,
      PredictedTrafficLightState::INFORMATION_SOURCE_INTERNAL_ESTIMATION)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert
  ASSERT_GE(arbiter_publish_count_, 1u);
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_a);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 1u);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::RED);
  EXPECT_EQ(group->predictions.size(), 2u);
}

// Color mismatch: external GREEN/CIRCLE vs perception RED/CIRCLE.
// Validator falls back to UNKNOWN over the shared shape.
TEST_F(ArbiterCharacteristic, signalMatchingColorMismatchProducesUnknown)
{
  // Arrange
  startArbiter(true, "confidence");  // signal matching mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_b,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_b,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert
  ASSERT_GE(arbiter_publish_count_, 1u);
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_b);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 1u);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::UNKNOWN);
  EXPECT_EQ(group->elements[0].shape, TrafficLightElement::CIRCLE);
}

// Element-count mismatch: external has only CIRCLE while perception has
// CIRCLE + RIGHT_ARROW. Validator produces UNKNOWN over the shape union.
TEST_F(ArbiterCharacteristic, signalMatchingElementCountMismatchProducesUnknown)
{
  // Arrange
  startArbiter(true, "confidence");  // signal matching mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_c,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_c,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE),
     makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert
  ASSERT_GE(arbiter_publish_count_, 1u);
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_c);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 2u);
  EXPECT_NE(findTrafficLightElement(*group, TrafficLightElement::CIRCLE), nullptr);
  EXPECT_NE(findTrafficLightElement(*group, TrafficLightElement::RIGHT_ARROW), nullptr);
  for (const auto & element : group->elements) {
    EXPECT_EQ(element.color, TrafficLightElement::UNKNOWN);
  }
}

// Off-map id: an id not present in the vector map is silently dropped
// (WARN log only) by add_signal_function before reaching the validator.
// An on-map id is also included to confirm normal ids still pass through.
TEST_F(ArbiterCharacteristic, signalMatchingOffMapIdDropped)
{
  // Arrange
  startArbiter(true, "confidence");  // signal matching mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    kOffMapProbeId,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert
  ASSERT_GE(arbiter_publish_count_, 1u);
  EXPECT_EQ(findTrafficLightGroup(latest_arbitrated_traffic_signal_, kOffMapProbeId), nullptr);
  EXPECT_NE(
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_a), nullptr);
}

// Pedestrian path bypasses element matching; with source_priority=external
// the external value must win even though perception has higher confidence.
TEST_F(ArbiterCharacteristic, signalMatchingPedestrianFallbackUsesSourcePriority)
{
  // Arrange
  startArbiter(true, "external");  // signal matching mode, "external" priority
  publishMap();
  const auto t0 = arbiter_->now();

  // The confidence contrast (low external vs high perception) makes the
  // priority override explicit: the lower-confidence external wins.
  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::pedestrian_signal,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.1f)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::pedestrian_signal,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.9f)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert: external priority wins despite its lower confidence.
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::pedestrian_signal);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 1u);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::RED);
}

// Symmetric counterpart of the external-priority pedestrian test: with
// source_priority=perception, the perception value wins even at lower
// confidence. Pins the PERCEPTION case of get_highest_confidence_signal
// inside SignalMatchValidator's pedestrian branch.
TEST_F(ArbiterCharacteristic, signalMatchingPedestrianPerceptionPriority)
{
  // Arrange
  startArbiter(true, "perception");  // signal matching mode, "perception" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  // The confidence contrast (high external vs low perception) makes the
  // priority override explicit: the lower-confidence perception still wins.
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::pedestrian_signal,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.99f)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::pedestrian_signal,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.10f)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert: perception priority wins despite its lower confidence.
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::pedestrian_signal);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 1u);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::GREEN);
}

// Pedestrian + CONFIDENCE: get_highest_confidence_signal walks each shape and
// picks the element with higher confidence. Bypasses the
// color/shape-equivalence check that non-pedestrian ids must pass.
TEST_F(ArbiterCharacteristic, signalMatchingPedestrianConfidenceMode)
{
  // Arrange
  startArbiter(true, "confidence");  // signal matching mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::pedestrian_signal,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.6f)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::pedestrian_signal,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.9f)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert: CONFIDENCE picks the higher-confidence element (perception 0.9)
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::pedestrian_signal);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 1u);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::RED);
  EXPECT_NEAR(group->elements[0].confidence, 0.9f, kConfidenceEpsilon);
}

// Pedestrian id with a single source: get_highest_confidence_signal early-
// returns the present side and skips the source_priority switch entirely.
// In contrast to non-pedestrian ids (which become UNKNOWN), the pedestrian's
// color/shape passes through unchanged.
TEST_F(ArbiterCharacteristic, signalMatchingPedestrianSingleSourcePasses)
{
  // Arrange: external_priority is set to demonstrate that absent-side handling
  // happens BEFORE the priority switch is consulted.
  startArbiter(true, "external");  // signal matching mode, "external" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::pedestrian_signal,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE)}));

  // Act (no external publish)
  publishPerception(perception_traffic_signal);

  // Assert: perception passes through unchanged (no UNKNOWN translation)
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::pedestrian_signal);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 1u);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::GREEN);
}

// In Signal Matching mode, a non-pedestrian id that arrives on only one side
// must be reported as UNKNOWN: the color/shape data is hidden, only the shape
// mask of the present side survives. Both directions (perception-only and
// external-only) exercise distinct branches in SignalMatchValidator.
TEST_F(ArbiterCharacteristic, signalMatchingSingleSourceNonPedestrianYieldsUnknown)
{
  // Scenario inputs and expectations:
  //
  //   id                external      perception   expected outcome
  //   ----------------  ------------  -----------  -----------------------------
  //   vehicle_signal_a  (none)        RED/CIRCLE   perception-only -> UNKNOWN/CIRCLE
  //   vehicle_signal_b  GREEN/CIRCLE  (none)       external-only   -> UNKNOWN/CIRCLE

  // Arrange
  startArbiter(true, "confidence");  // signal matching mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_b,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert
  {
    const auto * group =
      findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_a);
    ASSERT_NE(group, nullptr);
    ASSERT_EQ(group->elements.size(), 1u);
    EXPECT_EQ(group->elements[0].color, TrafficLightElement::UNKNOWN);
    EXPECT_EQ(group->elements[0].shape, TrafficLightElement::CIRCLE);
  }
  {
    const auto * group =
      findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_b);
    ASSERT_NE(group, nullptr);
    ASSERT_EQ(group->elements.size(), 1u);
    EXPECT_EQ(group->elements[0].color, TrafficLightElement::UNKNOWN);
    EXPECT_EQ(group->elements[0].shape, TrafficLightElement::CIRCLE);
  }
}

// Signal Matching does not consult the confidence field when checking
// equivalence. Two elements with identical color/shape but very different
// confidence values are treated as a match, and perception's value
// (confidence included) propagates verbatim.
TEST_F(ArbiterCharacteristic, signalMatchingIgnoresConfidenceInEquivalence)
{
  // Arrange
  startArbiter(true, "confidence");  // signal matching mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.10f)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.90f)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_a);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 1u);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::RED);
  EXPECT_NEAR(group->elements[0].confidence, 0.90f, kConfidenceEpsilon);
}

// ---------------------------------------------------------------------------
// B. Priority-based mode - per-shape element selection driven by
//    source_priority and confidence.
//
// Pins the arbitration spec when Signal Matching is off: for each
// regulatory-element id the arbiter walks every shape independently and
// resolves the chosen element through a two-stage comparison
// (priority flag first, confidence tiebreaker second). Each scenario is
// exercised by a dedicated test:
//   - both sources agree on shape         -> CONFIDENCE picks higher value
//   - only external contributes (per id)  -> passes through (multi-shape preserved)
//   - only perception contributes (per id) -> passes through
//   - off-map id                          -> dropped (WARN+skip)
// ---------------------------------------------------------------------------

// Both sources contribute the same shape for the same id; under CONFIDENCE
// the higher-confidence element wins (here, perception 0.9 over external 0.7).
TEST_F(ArbiterCharacteristic, priorityBasedConfidencePicksHigherValue)
{
  // Arrange
  startArbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.7f)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.9f)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert
  ASSERT_GE(arbiter_publish_count_, 1u);
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_a);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 1u);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::GREEN);
  EXPECT_NEAR(group->elements[0].confidence, 0.9f, kConfidenceEpsilon);
}

// Only external contributes an id (perception silent). Both shapes survive
// shape-wise selection and pass through unchanged.
TEST_F(ArbiterCharacteristic, priorityBasedExternalOnlyPassesThrough)
{
  // Arrange
  startArbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_b,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.5f),
     makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW, 0.3f)}));

  // Send a separate id from perception so the arbiter publishes after both
  // callbacks have settled (the test then inspects vehicle_signal_b output).
  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.9f)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert
  ASSERT_GE(arbiter_publish_count_, 1u);
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_b);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 2u);
  EXPECT_NE(findTrafficLightElement(*group, TrafficLightElement::CIRCLE), nullptr);
  EXPECT_NE(findTrafficLightElement(*group, TrafficLightElement::RIGHT_ARROW), nullptr);
}

// Only perception contributes an id (external silent). The element flows
// through unmodified.
TEST_F(ArbiterCharacteristic, priorityBasedPerceptionOnlyPassesThrough)
{
  // Arrange
  startArbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  // External must publish something for the arbiter to settle; use a
  // different id so vehicle_signal_c is perception-only.
  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.7f)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_c,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.8f)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert
  ASSERT_GE(arbiter_publish_count_, 1u);
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_c);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 1u);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::GREEN);
  EXPECT_NEAR(group->elements[0].confidence, 0.8f, kConfidenceEpsilon);
}

// An id not in the vector map is silently dropped (WARN log only) by
// add_signal_function before reaching the per-shape selection.
TEST_F(ArbiterCharacteristic, priorityBasedOffMapIdDropped)
{
  // Arrange
  startArbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    kOffMapProbeId,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.9f)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.9f)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert
  ASSERT_GE(arbiter_publish_count_, 1u);
  EXPECT_EQ(findTrafficLightGroup(latest_arbitrated_traffic_signal_, kOffMapProbeId), nullptr);
  EXPECT_NE(
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_a), nullptr);
}

// ---------------------------------------------------------------------------
// C. Focused specifications - boundary conditions, input validation, and
//    isolated arbitration rules.
//
// Each test pins a single contract that the longest-path tests above do not
// exercise: map availability, parameter validation, time tolerances,
// priority overrides, equivalence semantics, and prediction passthrough.
// ---------------------------------------------------------------------------

TEST_F(ArbiterCharacteristic, perceptionBeforeMapProducesNoOutput)
{
  // Arrange (intentionally no publishMap())
  startArbiter(false, "confidence");  // priority-based mode, "confidence" priority

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = arbiter_->now();
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  // Act
  publishPerception(perception_traffic_signal);

  // Assert
  EXPECT_EQ(arbiter_publish_count_, 0u)
    << "arbitrateAndPublish should early-return when no map has been received";
}

// A non-null but signal-free map should yield an empty TrafficLightGroupArray
// (stamp set, no groups). Exercises the
// "map_regulatory_elements_set_->empty()" branch in arbitrateAndPublish.
TEST_F(ArbiterCharacteristic, emptyMapProducesEmptyOutput)
{
  // Arrange: replace the default minimal map with a signal-free one.
  startArbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publishMap(buildEmptyMapBin());

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = arbiter_->now();
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  // Act
  publishPerception(perception_traffic_signal);

  // Assert: arbiter publishes, but the output contains no groups.
  ASSERT_GE(arbiter_publish_count_, 1u);
  EXPECT_EQ(latest_arbitrated_traffic_signal_.traffic_light_groups.size(), 0u);
}

// Pins the input-validation contract: an unknown source_priority string
// must not crash the node and must not silently change behavior; the
// arbiter must fall back to CONFIDENCE. We verify the fallback by feeding
// the typical CONFIDENCE scenario (high-confidence perception wins) and
// confirming perception is selected.
TEST_F(ArbiterCharacteristic, unknownSourcePriorityFallsBackToConfidence)
{
  // Arrange: pass a value not in {"external", "perception", "confidence"}.
  startArbiter(false, "invalid_value");  // priority-based mode, "invalid_value" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.7f)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.9f)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert: CONFIDENCE behavior - the higher-confidence perception wins.
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_a);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 1u);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::GREEN);
  EXPECT_NEAR(group->elements[0].confidence, 0.9f, kConfidenceEpsilon);
}

TEST_F(ArbiterCharacteristic, priorityFlagOverridesHigherConfidence)
{
  // Arrange
  startArbiter(false, "perception");  // priority-based mode, "perception" priority
  publishMap();
  const auto t0 = arbiter_->now();

  // The confidence contrast (high external vs low perception) makes the
  // priority override explicit: the lower-confidence perception still wins.
  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.99f)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.10f)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert: perception priority wins despite its lower confidence.
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_a);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 1u);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::GREEN);
}

// Symmetric counterpart of priorityFlagOverridesHigherConfidence: with
// source_priority=external, the external value wins over a higher-confidence
// perception value. This pins the EXTERNAL branch in arbitrateAndPublish
// ([traffic_light_arbiter.cpp]: source_priority_ == EXTERNAL).
TEST_F(ArbiterCharacteristic, priorityFlagFromExternalOverridesHigherConfidence)
{
  // Arrange
  startArbiter(false, "external");  // priority-based mode, "external" priority
  publishMap();
  const auto t0 = arbiter_->now();

  // The confidence contrast (low external vs high perception) makes the
  // priority override explicit: the lower-confidence external still wins.
  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.10f)}));

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.99f)}));

  // Act
  publishExternal(external_traffic_signal);
  publishPerception(perception_traffic_signal);

  // Assert: external priority wins despite its lower confidence.
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_a);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 1u);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::RED);
}

// Successive external publishes that carry different ids accumulate in the
// arbiter's external_traffic_lights_ map; each arrival triggers a new
// arbitrate run that publishes every currently-stored external signal.
TEST_F(ArbiterCharacteristic, multipleExternalSourcesAccumulate)
{
  // Arrange
  startArbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  // Act 1: publish the first external signal (id vehicle_signal_a).
  TrafficLightGroupArray first_external_traffic_signal;
  first_external_traffic_signal.stamp = t0;
  first_external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE)}));
  publishExternal(first_external_traffic_signal);

  // Assert 1: only vehicle_signal_a is visible after the first publish.
  ASSERT_EQ(latest_arbitrated_traffic_signal_.traffic_light_groups.size(), 1u);
  EXPECT_EQ(
    latest_arbitrated_traffic_signal_.traffic_light_groups[0].traffic_light_group_id,
    map_ids::vehicle_signal_a);

  // Act 2: publish a second external signal carrying a different id.
  TrafficLightGroupArray second_external_traffic_signal;
  second_external_traffic_signal.stamp = t0;
  second_external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_b,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));
  publishExternal(second_external_traffic_signal);

  // Assert 2: both vehicle_signal_a and vehicle_signal_b appear in the output.
  EXPECT_EQ(latest_arbitrated_traffic_signal_.traffic_light_groups.size(), 2u);
  const auto * group_a =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_a);
  const auto * group_b =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_b);
  ASSERT_NE(group_a, nullptr);
  ASSERT_NE(group_b, nullptr);
  EXPECT_EQ(group_a->elements[0].color, TrafficLightElement::GREEN);
  EXPECT_EQ(group_b->elements[0].color, TrafficLightElement::RED);
}

TEST_F(ArbiterCharacteristic, externalDelayToleranceDropsStaleMessage)
{
  // Arrange: establish a perception baseline so we can detect any extra publish.
  startArbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publishMap();

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = arbiter_->now();
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));
  publishPerception(perception_traffic_signal);
  const auto baseline_count = arbiter_publish_count_;
  ASSERT_GE(baseline_count, 1u);

  // external_delay_tolerance defaults to 5.0s; 20s in the past is well past it.
  TrafficLightGroupArray stale_external_traffic_signal;
  stale_external_traffic_signal.stamp = offsetTime(arbiter_->now(), -20.0);
  stale_external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE)}));

  // Act
  publishExternal(stale_external_traffic_signal);

  // Assert
  EXPECT_EQ(arbiter_publish_count_, baseline_count)
    << "Stale external message must be dropped before arbitrateAndPublish runs";
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_a);
  ASSERT_NE(group, nullptr);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::RED)
    << "Last publish must reflect perception";
}

TEST_F(ArbiterCharacteristic, externalTimeToleranceCleanupOnPerception)
{
  // Arrange: seed an external entry that should be purged by the perception
  // arrival's cleanupExpiredExternalSignals (perception is +6s newer).
  startArbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = t0;
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));
  publishExternal(external_traffic_signal);

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = offsetTime(t0, 6.0);
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE)}));

  // Act
  publishPerception(perception_traffic_signal);

  // Assert
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_a);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 1u);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::GREEN);
}

// onExternalMsg compares its own stamp to latest_perception_msg_.stamp; if
// the gap exceeds perception_time_tolerance (default 1.0s) it clears the
// stored perception groups so the upcoming publish reflects only external.
TEST_F(ArbiterCharacteristic, perceptionTimeToleranceClearsLatestPerception)
{
  // Arrange
  startArbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  // Seed a perception value at t0.
  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));
  publishPerception(perception_traffic_signal);

  // External arrives 2.0s after perception (> perception_time_tolerance=1.0,
  // < external_delay_tolerance=5.0 so it is itself accepted).
  TrafficLightGroupArray external_traffic_signal;
  external_traffic_signal.stamp = offsetTime(t0, 2.0);
  external_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE)}));

  // Act
  publishExternal(external_traffic_signal);

  // Assert: stale perception was wiped, output reflects only external.
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_a);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->elements.size(), 1u);
  EXPECT_EQ(group->elements[0].color, TrafficLightElement::GREEN);
}

TEST_F(ArbiterCharacteristic, predictionsFromSingleSidePropagate)
{
  // Arrange
  startArbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publishMap();
  const auto t0 = arbiter_->now();

  TrafficLightGroupArray perception_traffic_signal;
  perception_traffic_signal.stamp = t0;
  perception_traffic_signal.traffic_light_groups.push_back(makeTrafficLightGroup(
    map_ids::vehicle_signal_a,
    {makeTrafficLightElement(TrafficLightElement::RED, TrafficLightElement::CIRCLE)},
    {makeTrafficLightPrediction(
      t0, TrafficLightElement::GREEN, TrafficLightElement::CIRCLE,
      PredictedTrafficLightState::INFORMATION_SOURCE_INTERNAL_ESTIMATION)}));

  // Act
  publishPerception(perception_traffic_signal);

  // Assert
  const auto * group =
    findTrafficLightGroup(latest_arbitrated_traffic_signal_, map_ids::vehicle_signal_a);
  ASSERT_NE(group, nullptr);
  ASSERT_EQ(group->predictions.size(), 1u);
  EXPECT_EQ(
    group->predictions[0].information_source,
    PredictedTrafficLightState::INFORMATION_SOURCE_INTERNAL_ESTIMATION);
}

}  // namespace
