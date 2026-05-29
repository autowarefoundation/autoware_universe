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
constexpr lanelet::Id pedestrian_signal = 2001;

// Intentionally not installed on the map; used by tests as the "off-map id"
// probe to exercise the WARN+skip branch in arbitrateAndPublish.
constexpr lanelet::Id off_map_probe = 9999;
}  // namespace map_ids

// --- Map construction --------------------------------------------------

Point3d make_point(lanelet::Id id, double x, double y, double z = 0.0)
{
  return Point3d(id, x, y, z);
}

Lanelet make_lanelet(double y_left, double y_right, const std::string & subtype)
{
  LineString3d left(getId(), {make_point(getId(), 0.0, y_left), make_point(getId(), 10.0, y_left)});
  LineString3d right(
    getId(), {make_point(getId(), 0.0, y_right), make_point(getId(), 10.0, y_right)});
  Lanelet new_lanelet(getId(), left, right);
  new_lanelet.attributes()[AttributeName::Subtype] = subtype;
  new_lanelet.attributes()[AttributeName::Type] = AttributeValueString::Lanelet;
  return new_lanelet;
}

LineString3d make_traffic_light_bulb()
{
  // The exact position does not affect arbiter logic; we pick the lanelet
  // midpoint (x = 5.0 within the 0..10 span) at z = 5.0 above the ground.
  constexpr double kBulbX = 5.0;
  constexpr double kBulbZ = 5.0;
  LineString3d bulb(
    getId(), {make_point(getId(), kBulbX, 0.0, kBulbZ), make_point(getId(), kBulbX, 1.0, kBulbZ)});
  bulb.attributes()[AttributeName::Type] = AttributeValueString::TrafficLight;
  return bulb;
}

// Builds the minimal map required to drive every code path under test:
// two road lanelets each carrying one Traffic Light, plus one crosswalk
// lanelet carrying the pedestrian Traffic Light.
//
// Test map summary (top-down view).
// Each lanelet spans x = 0..10 along the travel direction; +y is the left side.
//
//   left bound  right bound   Lanelet      Traffic Light id
//   ----------  -----------   ----------   --------------------------
//         8           5       crosswalk    2001 (pedestrian_signal)
//         4           1       road2        1002 (vehicle_signal_b)
//         0          -3       road1        1001 (vehicle_signal_a)
//
// Probe id intentionally absent from the map: 9999 (map_ids::off_map_probe)
LaneletMapBin build_minimal_map_bin()
{
  Lanelet road1 = make_lanelet(0.0, -3.0, AttributeValueString::Road);
  Lanelet road2 = make_lanelet(4.0, 1.0, AttributeValueString::Road);
  Lanelet crosswalk = make_lanelet(8.0, 5.0, AttributeValueString::Crosswalk);

  road1.addRegulatoryElement(
    TrafficLight::make(
      map_ids::vehicle_signal_a, {}, LineStringsOrPolygons3d{make_traffic_light_bulb()}));
  road2.addRegulatoryElement(
    TrafficLight::make(
      map_ids::vehicle_signal_b, {}, LineStringsOrPolygons3d{make_traffic_light_bulb()}));
  crosswalk.addRegulatoryElement(
    TrafficLight::make(
      map_ids::pedestrian_signal, {}, LineStringsOrPolygons3d{make_traffic_light_bulb()}));

  const LaneletMapConstPtr map{utils::createMap(Lanelets{road1, road2, crosswalk}).release()};
  auto bin = ::autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  bin.header.frame_id = "base_link";
  return bin;
}

// Variant of build_minimal_map_bin with a single road lanelet but no Traffic
// Light regulatory elements. Used to drive the
// "map_regulatory_elements_set_->empty()" branch in arbitrateAndPublish.
LaneletMapBin build_empty_map_bin()
{
  Lanelet road = make_lanelet(0.0, -3.0, AttributeValueString::Road);
  const LaneletMapConstPtr map{utils::createMap(Lanelets{road}).release()};
  auto bin = ::autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  bin.header.frame_id = "base_link";
  return bin;
}

// --- Input message builders --------------------------------------------

// Confidence defaults to 1.0f because most tests do not assert on it.
TrafficLightElement make_traffic_light_element(
  uint8_t color, uint8_t shape, float confidence = 1.0f)
{
  TrafficLightElement element;
  element.color = color;
  element.shape = shape;
  element.status = TrafficLightElement::SOLID_ON;
  element.confidence = confidence;
  return element;
}

// Build a PredictedTrafficLightState with the given stamp. Color, shape,
// and source default to placeholder values because most tests only assert
// presence or count of predictions; tests that pin a specific source
// (e.g., predictionsFromSingleSidePropagate) pass it explicitly.
PredictedTrafficLightState make_traffic_light_prediction(
  const builtin_interfaces::msg::Time & stamp, uint8_t color = TrafficLightElement::UNKNOWN,
  uint8_t shape = TrafficLightElement::CIRCLE,
  const std::string & source = PredictedTrafficLightState::INFORMATION_SOURCE_INTERNAL_ESTIMATION)
{
  // The arbiter passes predictions through without inspecting any field
  // value, so the numbers below only need to form a syntactically complete
  // PredictedTrafficLightState.
  constexpr int32_t kPredictionOffsetSec = 10;
  constexpr float kFullCertainty = 1.0f;

  PredictedTrafficLightState prediction;
  prediction.predicted_stamp = stamp;
  prediction.predicted_stamp.sec += kPredictionOffsetSec;
  prediction.simultaneous_elements.push_back(
    make_traffic_light_element(color, shape, kFullCertainty));
  prediction.reliability = kFullCertainty;
  prediction.information_source = source;
  return prediction;
}

TrafficLightGroup make_traffic_light_group(
  lanelet::Id id, std::vector<TrafficLightElement> elements,
  std::vector<PredictedTrafficLightState> predictions = {})
{
  TrafficLightGroup group;
  group.traffic_light_group_id = id;
  group.elements = std::move(elements);
  group.predictions = std::move(predictions);
  return group;
}

// Compose a TrafficLightGroupArray with one group in a single expression.
TrafficLightGroupArray make_signal_array(
  const rclcpp::Time & stamp, lanelet::Id id, std::vector<TrafficLightElement> elements,
  std::vector<PredictedTrafficLightState> predictions = {})
{
  TrafficLightGroupArray msg;
  msg.stamp = stamp;
  msg.traffic_light_groups.push_back(
    make_traffic_light_group(id, std::move(elements), std::move(predictions)));
  return msg;
}

// Multi-group variant for tests that publish several ids in one message.
TrafficLightGroupArray make_signal_array(
  const rclcpp::Time & stamp, std::vector<TrafficLightGroup> groups)
{
  TrafficLightGroupArray msg;
  msg.stamp = stamp;
  msg.traffic_light_groups = std::move(groups);
  return msg;
}

// --- Timestamp utility -------------------------------------------------

builtin_interfaces::msg::Time offset_time(const rclcpp::Time & base, double seconds)
{
  return (base + rclcpp::Duration::from_seconds(seconds));
}

// --- Test fixture ------------------------------------------------------

class ArbiterCharacteristic : public ::testing::Test
{
protected:
  // --- Lifecycle ---------------------------------------------------------
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    // Reserve the highest signal id so utils::getId() never returns values
    // that collide with map_ids::* or map_ids::off_map_probe. Calling registerId(9999)
    // advances the global counter to >= 10000, which is safely above every
    // fixed signal id we use. This is process-wide global state, so doing it
    // once per suite is enough.
    utils::registerId(map_ids::off_map_probe);

    // map_bin_ is immutable across tests, so build it once for the suite.
    map_bin_ = build_minimal_map_bin();
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

  // --- Arrange helpers ---------------------------------------------------
  void start_arbiter(bool enable_signal_matching, const std::string & source_priority)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("external_delay_tolerance", kDefaultExternalDelayTolerance),
      rclcpp::Parameter("external_time_tolerance", kDefaultExternalTimeTolerance),
      rclcpp::Parameter("perception_time_tolerance", kDefaultPerceptionTimeTolerance),
      rclcpp::Parameter("source_priority", source_priority),
      rclcpp::Parameter("enable_signal_matching", enable_signal_matching),
    });
    arbiter_ = std::make_shared<TrafficLightArbiter>(options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(arbiter_);
    executor_->add_node(test_node_);
    spin_for();
    t0_ = arbiter_->now();
  }

  void publish_map() { publish_map(*map_bin_); }

  void publish_map(const LaneletMapBin & bin)
  {
    map_pub_->publish(bin);
    spin_for();
  }

  // --- Act helpers -------------------------------------------------------
  void publish_perception(const TrafficLightGroupArray & msg)
  {
    perception_pub_->publish(msg);
    spin_for();
  }
  void publish_external(const TrafficLightGroupArray & msg)
  {
    external_pub_->publish(msg);
    spin_for();
  }

  void spin_for(std::chrono::milliseconds duration = std::chrono::milliseconds(150))
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // --- Assert helpers ----------------------------------------------------
  // Look up a TrafficLightGroup or TrafficLightElement in the latest
  // arbitrated output. Returns nullptr when not present.
  const TrafficLightGroup * find_traffic_light_group(lanelet::Id id) const
  {
    for (const auto & group : latest_arbitrated_traffic_signal_.traffic_light_groups) {
      if (group.traffic_light_group_id == id) {
        return &group;
      }
    }
    return nullptr;
  }
  const TrafficLightElement * find_traffic_light_element(
    const TrafficLightGroup & group, uint8_t shape) const
  {
    for (const auto & element : group.elements) {
      if (element.shape == shape) {
        return &element;
      }
    }
    return nullptr;
  }

  // Return the first element's color/shape/confidence for the given id in
  // the latest arbitrated output. When the id is absent or has no
  // elements, return a sentinel value (UINT8_MAX / -1.0f) that is never
  // valid, so EXPECT_EQ at the call site fails loudly.
  uint8_t observed_color(lanelet::Id signal_id) const
  {
    const auto * group = find_traffic_light_group(signal_id);
    return (group && !group->elements.empty()) ? group->elements[0].color : UINT8_MAX;
  }
  uint8_t observed_shape(lanelet::Id signal_id) const
  {
    const auto * group = find_traffic_light_group(signal_id);
    return (group && !group->elements.empty()) ? group->elements[0].shape : UINT8_MAX;
  }
  float observed_confidence(lanelet::Id signal_id) const
  {
    const auto * group = find_traffic_light_group(signal_id);
    return (group && !group->elements.empty()) ? group->elements[0].confidence : -1.0f;
  }
  std::size_t observed_prediction_count(lanelet::Id signal_id) const
  {
    const auto * group = find_traffic_light_group(signal_id);
    return group ? group->predictions.size() : SIZE_MAX;
  }

  // Cardinality of the latest published TrafficLightGroupArray.
  std::size_t observed_group_count() const
  {
    return latest_arbitrated_traffic_signal_.traffic_light_groups.size();
  }

  // Multi-element variants: pick the element with the matching shape.
  std::size_t observed_element_count(lanelet::Id signal_id) const
  {
    const auto * group = find_traffic_light_group(signal_id);
    return group ? group->elements.size() : SIZE_MAX;
  }
  uint8_t observed_color_of_shape(lanelet::Id signal_id, uint8_t shape) const
  {
    const auto * group = find_traffic_light_group(signal_id);
    if (!group) {
      return UINT8_MAX;
    }
    const auto * element = find_traffic_light_element(*group, shape);
    return element ? element->color : UINT8_MAX;
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

  // Reference timestamp captured once in start_arbiter(); tests use it
  // (and offset_time(t0_, ...)) as a fixed clock.
  rclcpp::Time t0_;

private:
  // Topic names used by the publishers/subscription owned by the fixture.
  static constexpr const char * kMapTopic = "/traffic_light_arbiter/sub/vector_map";
  static constexpr const char * kPerceptionTopic =
    "/traffic_light_arbiter/sub/perception_traffic_signals";
  static constexpr const char * kExternalTopic =
    "/traffic_light_arbiter/sub/external_traffic_signals";
  static constexpr const char * kOutputTopic = "/traffic_light_arbiter/pub/traffic_signals";

protected:
  // Tolerance used by EXPECT_NEAR when comparing confidence values.
  static constexpr float kConfidenceEpsilon = 1e-5f;

  // Default tolerance values declared on the arbiter (mirrors
  // config/traffic_light_arbiter.param.yaml).
  static constexpr double kDefaultExternalDelayTolerance = 5.0;
  static constexpr double kDefaultExternalTimeTolerance = 5.0;
  static constexpr double kDefaultPerceptionTimeTolerance = 1.0;
};

// ---------------------------------------------------------------------------
// A. Signal Matching mode (enable_signal_matching=true)
// ---------------------------------------------------------------------------

// Matched: external and perception agree on color and shape. Perception
// passes through verbatim, and predictions from both sides are merged.
TEST_F(ArbiterCharacteristic, signalMatchingMatchedPassesThrough)
{
  // Arrange
  start_arbiter(true, "confidence");  // signal matching mode, "confidence" priority
  publish_map();

  // Act
  publish_external(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)},
    {make_traffic_light_prediction(t0_)}));
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)},
    {make_traffic_light_prediction(t0_)}));

  // Assert
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_a), TrafficLightElement::RED);
  EXPECT_EQ(observed_prediction_count(map_ids::vehicle_signal_a), 2u);
}

// Color mismatch: external GREEN/CIRCLE vs perception RED/CIRCLE.
// Validator falls back to UNKNOWN over the shared shape.
TEST_F(ArbiterCharacteristic, signalMatchingColorMismatchProducesUnknown)
{
  // Arrange
  start_arbiter(true, "confidence");  // signal matching mode, "confidence" priority
  publish_map();

  // Act
  publish_external(make_signal_array(
    t0_, map_ids::vehicle_signal_b,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE)}));
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_b,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  // Assert
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_b), TrafficLightElement::UNKNOWN);
  EXPECT_EQ(observed_shape(map_ids::vehicle_signal_b), TrafficLightElement::CIRCLE);
}

// Element-count mismatch: external has only CIRCLE while perception has
// CIRCLE + RIGHT_ARROW. Validator produces UNKNOWN over the shape union.
TEST_F(ArbiterCharacteristic, signalMatchingElementCountMismatchProducesUnknown)
{
  // Arrange
  start_arbiter(true, "confidence");  // signal matching mode, "confidence" priority
  publish_map();

  // Act
  publish_external(make_signal_array(
    t0_, map_ids::vehicle_signal_b,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_b,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE),
     make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW)}));

  // Assert: shape union has both CIRCLE and RIGHT_ARROW, both as UNKNOWN.
  EXPECT_EQ(observed_element_count(map_ids::vehicle_signal_b), 2u);
  EXPECT_EQ(
    observed_color_of_shape(map_ids::vehicle_signal_b, TrafficLightElement::CIRCLE),
    TrafficLightElement::UNKNOWN);
  EXPECT_EQ(
    observed_color_of_shape(map_ids::vehicle_signal_b, TrafficLightElement::RIGHT_ARROW),
    TrafficLightElement::UNKNOWN);
}

// Off-map id: an id not present in the vector map is silently dropped
// (WARN log only) by add_signal_function before reaching the validator.
// An on-map id is also included to confirm normal ids still pass through.
TEST_F(ArbiterCharacteristic, signalMatchingOffMapIdDropped)
{
  // Arrange
  start_arbiter(true, "confidence");  // signal matching mode, "confidence" priority
  publish_map();

  // Act
  publish_external(make_signal_array(
    t0_, {make_traffic_light_group(
            map_ids::off_map_probe,
            {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}),
          make_traffic_light_group(
            map_ids::vehicle_signal_a,
            {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)})}));
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  // Assert
  EXPECT_EQ(find_traffic_light_group(map_ids::off_map_probe), nullptr);
  EXPECT_NE(find_traffic_light_group(map_ids::vehicle_signal_a), nullptr);
}

// Pedestrian path bypasses element matching; with source_priority=external
// the external value must win even though perception has higher confidence.
TEST_F(ArbiterCharacteristic, signalMatchingPedestrianFallbackUsesSourcePriority)
{
  // Arrange
  start_arbiter(true, "external");  // signal matching mode, "external" priority
  publish_map();

  // Act: confidence contrast (low external vs high perception) makes the
  // priority override explicit: the lower-confidence external wins.
  publish_external(make_signal_array(
    t0_, map_ids::pedestrian_signal,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.1f)}));
  publish_perception(make_signal_array(
    t0_, map_ids::pedestrian_signal,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.9f)}));

  // Assert: external priority wins despite its lower confidence.
  EXPECT_EQ(observed_color(map_ids::pedestrian_signal), TrafficLightElement::RED);
}

// Symmetric counterpart of the external-priority pedestrian test: with
// source_priority=perception, the perception value wins even at lower
// confidence. Pins the PERCEPTION case of get_highest_confidence_signal
// inside SignalMatchValidator's pedestrian branch.
TEST_F(ArbiterCharacteristic, signalMatchingPedestrianPerceptionPriority)
{
  // Arrange
  start_arbiter(true, "perception");  // signal matching mode, "perception" priority
  publish_map();

  // Act: confidence contrast (high external vs low perception) makes the
  // priority override explicit: the lower-confidence perception still wins.
  publish_external(make_signal_array(
    t0_, map_ids::pedestrian_signal,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.99f)}));
  publish_perception(make_signal_array(
    t0_, map_ids::pedestrian_signal,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.10f)}));

  // Assert: perception priority wins despite its lower confidence.
  EXPECT_EQ(observed_color(map_ids::pedestrian_signal), TrafficLightElement::GREEN);
}

// Pedestrian + CONFIDENCE: get_highest_confidence_signal walks each shape and
// picks the element with higher confidence. Bypasses the
// color/shape-equivalence check that non-pedestrian ids must pass.
TEST_F(ArbiterCharacteristic, signalMatchingPedestrianConfidenceMode)
{
  // Arrange
  start_arbiter(true, "confidence");  // signal matching mode, "confidence" priority
  publish_map();

  // Act
  publish_external(make_signal_array(
    t0_, map_ids::pedestrian_signal,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.6f)}));
  publish_perception(make_signal_array(
    t0_, map_ids::pedestrian_signal,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.9f)}));

  // Assert: CONFIDENCE picks the higher-confidence element (perception 0.9)
  EXPECT_EQ(observed_color(map_ids::pedestrian_signal), TrafficLightElement::RED);
  EXPECT_NEAR(observed_confidence(map_ids::pedestrian_signal), 0.9f, kConfidenceEpsilon);
}

// Pedestrian id with a single source: get_highest_confidence_signal early-
// returns the present side and skips the source_priority switch entirely.
// In contrast to non-pedestrian ids (which become UNKNOWN), the pedestrian's
// color/shape passes through unchanged.
TEST_F(ArbiterCharacteristic, signalMatchingPedestrianSingleSourcePasses)
{
  // Arrange: external_priority is set to demonstrate that absent-side handling
  // happens BEFORE the priority switch is consulted.
  start_arbiter(true, "external");  // signal matching mode, "external" priority
  publish_map();

  // Act (no external publish)
  publish_perception(make_signal_array(
    t0_, map_ids::pedestrian_signal,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE)}));

  // Assert: perception passes through unchanged (no UNKNOWN translation)
  EXPECT_EQ(observed_color(map_ids::pedestrian_signal), TrafficLightElement::GREEN);
}

// In Signal Matching mode, a non-pedestrian id that arrives on only one side
// must be reported as UNKNOWN: the color/shape data is hidden, only the shape
// mask of the present side survives. Both directions (perception-only and
// external-only) exercise distinct branches in SignalMatchValidator.
TEST_F(ArbiterCharacteristic, signalMatchingSingleSourceNonPedestrianYieldsUnknown)
{
  // Arrange
  start_arbiter(true, "confidence");  // signal matching mode, "confidence" priority
  publish_map();

  // Act: each side carries a different id, so each id has only one source
  // (vehicle_signal_a is perception-only, vehicle_signal_b is external-only).
  publish_external(make_signal_array(
    t0_, map_ids::vehicle_signal_b,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE)}));
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  // Assert
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_a), TrafficLightElement::UNKNOWN);
  EXPECT_EQ(observed_shape(map_ids::vehicle_signal_a), TrafficLightElement::CIRCLE);
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_b), TrafficLightElement::UNKNOWN);
  EXPECT_EQ(observed_shape(map_ids::vehicle_signal_b), TrafficLightElement::CIRCLE);
}

// Signal Matching does not consult the confidence field when checking
// equivalence. Two elements with identical color/shape but very different
// confidence values are treated as a match, and perception's value
// (confidence included) propagates verbatim.
TEST_F(ArbiterCharacteristic, signalMatchingIgnoresConfidenceInEquivalence)
{
  // Arrange
  start_arbiter(true, "confidence");  // signal matching mode, "confidence" priority
  publish_map();

  // Act
  publish_external(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.10f)}));
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.90f)}));

  // Assert
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_a), TrafficLightElement::RED);
  EXPECT_NEAR(observed_confidence(map_ids::vehicle_signal_a), 0.90f, kConfidenceEpsilon);
}

// ---------------------------------------------------------------------------
// B. Priority-based mode (enable_signal_matching=false)
// ---------------------------------------------------------------------------

// Both sources contribute the same shape for the same id; under CONFIDENCE
// the higher-confidence element wins (here, perception 0.9 over external 0.7).
TEST_F(ArbiterCharacteristic, priorityBasedConfidencePicksHigherValue)
{
  // Arrange
  start_arbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publish_map();

  // Act
  publish_external(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.7f)}));
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.9f)}));

  // Assert
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_a), TrafficLightElement::GREEN);
  EXPECT_NEAR(observed_confidence(map_ids::vehicle_signal_a), 0.9f, kConfidenceEpsilon);
}

// Only external contributes an id (perception silent). Both shapes survive
// shape-wise selection and pass through unchanged.
TEST_F(ArbiterCharacteristic, priorityBasedExternalOnlyPassesThrough)
{
  // Arrange
  start_arbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publish_map();

  // Act
  publish_external(make_signal_array(
    t0_, map_ids::vehicle_signal_b,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.5f),
     make_traffic_light_element(
       TrafficLightElement::GREEN, TrafficLightElement::RIGHT_ARROW, 0.3f)}));
  // Send a separate id from perception so the arbiter publishes after both
  // callbacks have settled (the test then inspects vehicle_signal_b output).
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.9f)}));

  // Assert: external-only passes through with both shapes and colors preserved.
  EXPECT_EQ(observed_element_count(map_ids::vehicle_signal_b), 2u);
  EXPECT_EQ(
    observed_color_of_shape(map_ids::vehicle_signal_b, TrafficLightElement::CIRCLE),
    TrafficLightElement::RED);
  EXPECT_EQ(
    observed_color_of_shape(map_ids::vehicle_signal_b, TrafficLightElement::RIGHT_ARROW),
    TrafficLightElement::GREEN);
}

// Only perception contributes an id (external silent). The element flows
// through unmodified.
TEST_F(ArbiterCharacteristic, priorityBasedPerceptionOnlyPassesThrough)
{
  // Arrange
  start_arbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publish_map();

  // Act: external must publish something for the arbiter to settle; use a
  // different id so vehicle_signal_b is perception-only.
  publish_external(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.7f)}));
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_b,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.8f)}));

  // Assert
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_b), TrafficLightElement::GREEN);
  EXPECT_NEAR(observed_confidence(map_ids::vehicle_signal_b), 0.8f, kConfidenceEpsilon);
}

// An id not in the vector map is silently dropped (WARN log only) by
// add_signal_function before reaching the per-shape selection.
TEST_F(ArbiterCharacteristic, priorityBasedOffMapIdDropped)
{
  // Arrange
  start_arbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publish_map();

  // Act
  publish_external(make_signal_array(
    t0_, map_ids::off_map_probe,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.9f)}));
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.9f)}));

  // Assert
  EXPECT_EQ(find_traffic_light_group(map_ids::off_map_probe), nullptr);
  EXPECT_NE(find_traffic_light_group(map_ids::vehicle_signal_a), nullptr);
}

// ---------------------------------------------------------------------------
// C. Focused specifications — boundary cases, time tolerances,
//    input validation, priority overrides, predictions.
// ---------------------------------------------------------------------------

// Before the vector_map arrives, arbitrateAndPublish early-returns and no
// TrafficLightGroupArray is emitted.
TEST_F(ArbiterCharacteristic, perceptionBeforeMapProducesNoOutput)
{
  // Arrange (intentionally no publish_map())
  start_arbiter(false, "confidence");  // priority-based mode, "confidence" priority

  // Act
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  // Assert: arbitrateAndPublish should early-return when no map has been
  // received. Content alone cannot prove this (a default-constructed
  // message looks identical to no publish at all), so we read the counter.
  EXPECT_EQ(arbiter_publish_count_, 0u);
}

// A non-null but signal-free map should yield an empty TrafficLightGroupArray
// (stamp set, no groups). Exercises the
// "map_regulatory_elements_set_->empty()" branch in arbitrateAndPublish.
TEST_F(ArbiterCharacteristic, emptyMapProducesEmptyOutput)
{
  // Arrange: replace the default minimal map with a signal-free one.
  start_arbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publish_map(build_empty_map_bin());

  // Act
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  // Assert: arbiter publishes, but the output contains no groups. The
  // counter read distinguishes "publish happened with zero groups" (the
  // spec) from "no publish at all" (which would also leave groups empty).
  ASSERT_GE(arbiter_publish_count_, 1u);
  EXPECT_EQ(observed_group_count(), 0u);
}

// An unknown source_priority string must fall back to CONFIDENCE. Verified
// by feeding a CONFIDENCE scenario and confirming the higher-confidence
// perception wins.
TEST_F(ArbiterCharacteristic, unknownSourcePriorityFallsBackToConfidence)
{
  // Arrange: pass a value not in {"external", "perception", "confidence"}.
  start_arbiter(false, "invalid_value");  // priority-based mode, "invalid_value" priority
  publish_map();

  // Act
  publish_external(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.7f)}));
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.9f)}));

  // Assert: CONFIDENCE behavior - the higher-confidence perception wins.
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_a), TrafficLightElement::GREEN);
  EXPECT_NEAR(observed_confidence(map_ids::vehicle_signal_a), 0.9f, kConfidenceEpsilon);
}

// With source_priority=perception, the perception value wins even when its
// confidence is much lower than external's. Pins the PERCEPTION priority
// branch in arbitrateAndPublish.
TEST_F(ArbiterCharacteristic, priorityFlagOverridesHigherConfidence)
{
  // Arrange
  start_arbiter(false, "perception");  // priority-based mode, "perception" priority
  publish_map();

  // Act: confidence contrast (high external vs low perception) makes the
  // priority override explicit: the lower-confidence perception still wins.
  publish_external(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.99f)}));
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.10f)}));

  // Assert: perception priority wins despite its lower confidence.
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_a), TrafficLightElement::GREEN);
}

// Symmetric counterpart of priorityFlagOverridesHigherConfidence: with
// source_priority=external, the external value wins over a higher-confidence
// perception value.
TEST_F(ArbiterCharacteristic, priorityFlagFromExternalOverridesHigherConfidence)
{
  // Arrange
  start_arbiter(false, "external");  // priority-based mode, "external" priority
  publish_map();

  // Act: confidence contrast (low external vs high perception) makes the
  // priority override explicit: the lower-confidence external still wins.
  publish_external(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE, 0.10f)}));
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE, 0.99f)}));

  // Assert: external priority wins despite its lower confidence.
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_a), TrafficLightElement::RED);
}

// Successive external publishes that carry different ids accumulate in the
// arbiter's external_traffic_lights_ map; both end up in the final output
// with their published values preserved.
TEST_F(ArbiterCharacteristic, multipleExternalSourcesAccumulate)
{
  // Arrange
  start_arbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publish_map();

  // Act: publish two external signals carrying different ids.
  publish_external(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE)}));
  publish_external(make_signal_array(
    t0_, map_ids::vehicle_signal_b,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  // Assert: both ids accumulated, each id retains its published color.
  EXPECT_EQ(observed_group_count(), 2u);
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_a), TrafficLightElement::GREEN);
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_b), TrafficLightElement::RED);
}

// An external message whose stamp is older than external_delay_tolerance
// is dropped before reaching arbitrateAndPublish, so the last published
// content stays as the earlier perception value.
TEST_F(ArbiterCharacteristic, externalDelayToleranceDropsStaleMessage)
{
  // Arrange: seed a perception value (RED). If the stale external below were
  // ever accepted, the output would change to GREEN — the content check at
  // the bottom is what pins "stale dropped".
  start_arbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publish_map();
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  // Act: stamp is past external_delay_tolerance in the past.
  publish_external(make_signal_array(
    offset_time(t0_, -(kDefaultExternalDelayTolerance + 1.0)), map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE)}));

  // Assert: the stale GREEN never propagates; the published color stays RED.
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_a), TrafficLightElement::RED);
}

// When perception arrives past external_time_tolerance after a stored
// external entry, cleanupExpiredExternalSignals purges that entry so the
// upcoming publish reflects only perception.
TEST_F(ArbiterCharacteristic, externalTimeToleranceCleanupOnPerception)
{
  // Arrange: seed an external entry that should be purged by the perception
  // arrival's cleanupExpiredExternalSignals (perception is past
  // external_time_tolerance newer than the stored external).
  start_arbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publish_map();
  publish_external(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  // Act
  publish_perception(make_signal_array(
    offset_time(t0_, kDefaultExternalTimeTolerance + 1.0), map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE)}));

  // Assert
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_a), TrafficLightElement::GREEN);
}

// onExternalMsg compares its own stamp to latest_perception_msg_.stamp; if
// the gap exceeds perception_time_tolerance it clears the stored perception
// groups so the upcoming publish reflects only external.
TEST_F(ArbiterCharacteristic, perceptionTimeToleranceClearsLatestPerception)
{
  // Arrange
  start_arbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publish_map();

  // Seed a perception value at t0_.
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)}));

  // Act: external arrives past perception_time_tolerance after perception
  // (so latest perception is cleared) but within external_delay_tolerance
  // so external itself is accepted.
  publish_external(make_signal_array(
    offset_time(t0_, kDefaultPerceptionTimeTolerance + 1.0), map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::GREEN, TrafficLightElement::CIRCLE)}));

  // Assert: stale perception was wiped, output reflects only external.
  EXPECT_EQ(observed_color(map_ids::vehicle_signal_a), TrafficLightElement::GREEN);
}

// Predictions sent on only one side propagate to the arbitrated output
// with their information_source preserved.
TEST_F(ArbiterCharacteristic, predictionsFromSingleSidePropagate)
{
  // Arrange
  start_arbiter(false, "confidence");  // priority-based mode, "confidence" priority
  publish_map();

  // Act
  publish_perception(make_signal_array(
    t0_, map_ids::vehicle_signal_a,
    {make_traffic_light_element(TrafficLightElement::RED, TrafficLightElement::CIRCLE)},
    {make_traffic_light_prediction(
      t0_, TrafficLightElement::GREEN, TrafficLightElement::CIRCLE,
      PredictedTrafficLightState::INFORMATION_SOURCE_INTERNAL_ESTIMATION)}));

  // Assert
  ASSERT_EQ(observed_prediction_count(map_ids::vehicle_signal_a), 1u);
  const auto * group = find_traffic_light_group(map_ids::vehicle_signal_a);
  EXPECT_EQ(
    group->predictions[0].information_source,
    PredictedTrafficLightState::INFORMATION_SOURCE_INTERNAL_ESTIMATION);
}

}  // namespace
