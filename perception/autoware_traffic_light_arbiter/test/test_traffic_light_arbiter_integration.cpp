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

// Node-level integration test for TrafficLightArbiterNode: the ROS layer only —
// input topics reach the core, parameters route in, the output is republished,
// and the Node's publish/skip decisions hold. The arbitration *rules* live
// ROS-free in test_traffic_light_arbiter.cpp; a test that observes a rule
// here does so only to prove the wire carries it, and names the owning core test.

#include "autoware/traffic_light_arbiter/traffic_light_arbiter_node.hpp"

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
#include <string>
#include <thread>

namespace
{

using autoware::traffic_light::TrafficLightArbiterNode;
using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
using TrafficLightGroupArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
using TrafficLightGroup = autoware_perception_msgs::msg::TrafficLightGroup;
using TrafficLightElement = autoware_perception_msgs::msg::TrafficLightElement;

constexpr lanelet::Id vehicle_signal = 1001;

// A LaneletMapBin with one road lanelet carrying a single traffic light
// (vehicle_signal). The arbiter only reads the signal-id set, so the geometry
// is arbitrary. getId() hands out only a few low ids here, so it never collides
// with the fixed vehicle_signal (1001) — no registerId reservation needed.
LaneletMapBin build_minimal_map_bin()
{
  using namespace lanelet;  // NOLINT(build/namespaces)
  using utils::getId;

  // Road
  LineString3d left(getId(), {Point3d(getId(), 0.0, 0.0), Point3d(getId(), 1.0, 0.0)});
  LineString3d right(getId(), {Point3d(getId(), 0.0, 1.0), Point3d(getId(), 1.0, 1.0)});
  Lanelet road(getId(), left, right);
  road.attributes()[AttributeName::Type] = AttributeValueString::Lanelet;
  road.attributes()[AttributeName::Subtype] = AttributeValueString::Road;

  // Traffic light
  LineString3d bulb(getId(), {Point3d(getId(), 0.0, 0.0, 5.0), Point3d(getId(), 1.0, 0.0, 5.0)});
  bulb.attributes()[AttributeName::Type] = AttributeValueString::TrafficLight;
  road.addRegulatoryElement(TrafficLight::make(vehicle_signal, {}, LineStringsOrPolygons3d{bulb}));

  const LaneletMapConstPtr map{utils::createMap(Lanelets{road}).release()};
  auto bin = ::autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  bin.header.frame_id = "base_link";
  return bin;
}

class ArbiterIntegration : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    test_node_ = std::make_shared<rclcpp::Node>("arbiter_integration_test");

    map_pub_ =
      test_node_->create_publisher<LaneletMapBin>(map_topic, rclcpp::QoS(1).transient_local());
    perception_pub_ =
      test_node_->create_publisher<TrafficLightGroupArray>(perception_topic, rclcpp::QoS(1));
    external_pub_ =
      test_node_->create_publisher<TrafficLightGroupArray>(external_topic, rclcpp::QoS(1));

    arbitrated_traffic_signal_sub_ = test_node_->create_subscription<TrafficLightGroupArray>(
      output_topic, rclcpp::QoS(1), [this](const TrafficLightGroupArray::ConstSharedPtr msg) {
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

  // Parameters are set in code; each test picks the priority/matching combo it pins.
  void start_arbiter(bool enable_signal_matching, const std::string & source_priority)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("external_delay_tolerance", default_external_delay_tolerance),
      rclcpp::Parameter("external_time_tolerance", default_external_time_tolerance),
      rclcpp::Parameter("perception_time_tolerance", default_perception_time_tolerance),
      rclcpp::Parameter("source_priority", source_priority),
      rclcpp::Parameter("enable_signal_matching", enable_signal_matching),
    });
    arbiter_ = std::make_shared<TrafficLightArbiterNode>(options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(arbiter_->get_node_base_interface());
    executor_->add_node(test_node_);
    spin_for();
    // t0 tracks the node clock (not a constant): external stamps are validated against now().
    t0_ = arbiter_->now();
  }

  // A one-element (CIRCLE) signal for vehicle_signal, stamped at t0_. Only the
  // color and confidence vary between tests, so those are the only arguments.
  TrafficLightGroupArray make_signal(uint8_t color, float confidence = 1.0f) const
  {
    TrafficLightElement element;
    element.color = color;
    element.shape = TrafficLightElement::CIRCLE;
    element.status = TrafficLightElement::SOLID_ON;
    element.confidence = confidence;

    TrafficLightGroup group;
    group.traffic_light_group_id = vehicle_signal;
    group.elements = {element};

    TrafficLightGroupArray msg;
    msg.stamp = t0_;
    msg.traffic_light_groups = {group};
    return msg;
  }

  // Publish the minimal map and settle it before any signal: with no map the
  // arbiter emits nothing (arbitrate() returns nullopt), so a signal sent first
  // would drive no publish and the wait below would time out.
  void publish_map()
  {
    map_pub_->publish(build_minimal_map_bin());
    spin_for();
  }

  // Signal publishers only emit; callers then advance the executor with
  // spin_until_arbiter_publish_count() (positive path) or spin_for() (to prove a publish
  // absent). Separating emit from wait lets tests block on the real publish.
  void publish_perception(const TrafficLightGroupArray & msg) { perception_pub_->publish(msg); }
  void publish_external(const TrafficLightGroupArray & msg) { external_pub_->publish(msg); }

  // Spin until the arbiter has published at least `target` messages (or timeout),
  // returning whether it reached the target so callers ASSERT before reading the
  // latest output. Each accepted signal drives one publish; the map never does.
  bool spin_until_arbiter_publish_count(
    std::size_t target, std::chrono::milliseconds timeout = std::chrono::milliseconds(2000))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (arbiter_publish_count_ < target && std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return arbiter_publish_count_ >= target;
  }

  // Fixed-duration spin for what can't be waited on a publish: settling after
  // setup or map, or asserting the *absence* of a publish.
  void spin_for(std::chrono::milliseconds duration = std::chrono::milliseconds(150))
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  const TrafficLightGroup * find_traffic_light_group(lanelet::Id id) const
  {
    for (const auto & group : latest_arbitrated_traffic_signal_.traffic_light_groups) {
      if (group.traffic_light_group_id == id) {
        return &group;
      }
    }
    return nullptr;
  }

  // Return an impossible sentinel (UINT8_MAX) when the id is absent so a wrong
  // id fails the EXPECT_EQ at the call site loudly.
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

  std::size_t observed_group_count() const
  {
    return latest_arbitrated_traffic_signal_.traffic_light_groups.size();
  }

  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<TrafficLightArbiterNode> arbiter_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Publisher<LaneletMapBin>::SharedPtr map_pub_;
  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr perception_pub_;
  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr external_pub_;
  rclcpp::Subscription<TrafficLightGroupArray>::SharedPtr arbitrated_traffic_signal_sub_;

  TrafficLightGroupArray latest_arbitrated_traffic_signal_;
  std::size_t arbiter_publish_count_ = 0;
  rclcpp::Time t0_;

private:
  static constexpr const char * map_topic = "/traffic_light_arbiter/sub/vector_map";
  static constexpr const char * perception_topic =
    "/traffic_light_arbiter/sub/perception_traffic_signals";
  static constexpr const char * external_topic =
    "/traffic_light_arbiter/sub/external_traffic_signals";
  static constexpr const char * output_topic = "/traffic_light_arbiter/pub/traffic_signals";

  // Shipped defaults from config/traffic_light_arbiter.param.yaml.
  static constexpr double default_external_delay_tolerance = 5.0;
  static constexpr double default_external_time_tolerance = 5.0;
  static constexpr double default_perception_time_tolerance = 1.0;
};

// Both sources carrying the same id collapse into one output group, confirming
// both subscriptions feed the merge. Which element wins is a core concern
// (core_test::picksHigherConfidenceElement), not re-checked here.
TEST_F(ArbiterIntegration, bothSourcesCollapseIntoOneGroup)
{
  start_arbiter(false, "confidence");
  publish_map();

  // Drive one source at a time so both are cached when the second arbitrates.
  publish_external(make_signal(TrafficLightElement::GREEN));
  ASSERT_TRUE(spin_until_arbiter_publish_count(1));
  publish_perception(make_signal(TrafficLightElement::RED));
  ASSERT_TRUE(spin_until_arbiter_publish_count(2));

  EXPECT_EQ(observed_group_count(), 1u);
}

// Perception alone drives a publish, republished on the output topic.
TEST_F(ArbiterIntegration, perceptionOnlyIsRepublished)
{
  start_arbiter(false, "confidence");
  publish_map();

  publish_perception(make_signal(TrafficLightElement::GREEN));
  ASSERT_TRUE(spin_until_arbiter_publish_count(1));

  EXPECT_EQ(observed_color(vehicle_signal), TrafficLightElement::GREEN);
}

// External alone drives a publish, republished on the output topic.
TEST_F(ArbiterIntegration, externalOnlyIsRepublished)
{
  start_arbiter(false, "confidence");
  publish_map();

  publish_external(make_signal(TrafficLightElement::RED));
  ASSERT_TRUE(spin_until_arbiter_publish_count(1));

  EXPECT_EQ(observed_color(vehicle_signal), TrafficLightElement::RED);
}

// No map yet -> no publish. Absence can't be waited on, so settle with a fixed
// spin and confirm the counter never moved; the nullopt the Node keys off is
// pinned by core_test::arbitrateWithoutMapProducesNoOutput.
TEST_F(ArbiterIntegration, outputSuppressedUntilMapArrives)
{
  start_arbiter(false, "confidence");

  publish_perception(make_signal(TrafficLightElement::RED));
  spin_for();

  EXPECT_EQ(arbiter_publish_count_, 0u);
}

// An empty input (map present, no groups) still drives a publish with an empty
// output; the counter distinguishes "published empty" from "never published".
TEST_F(ArbiterIntegration, emptyInputMessageProducesEmptyOutput)
{
  start_arbiter(false, "confidence");
  publish_map();

  TrafficLightGroupArray empty_msg;
  empty_msg.stamp = t0_;
  publish_perception(empty_msg);
  ASSERT_TRUE(spin_until_arbiter_publish_count(1));

  EXPECT_EQ(observed_group_count(), 0u);
}

// enable_signal_matching is routed into the core: a color mismatch collapsing
// to UNKNOWN is SignalMatchValidator's signature, proving the flag reached it.
// The rule itself is owned by core_test::colorMismatchProducesUnknown.
TEST_F(ArbiterIntegration, enableSignalMatchingRoutesToValidator)
{
  start_arbiter(true, "confidence");
  publish_map();

  publish_external(make_signal(TrafficLightElement::GREEN));
  ASSERT_TRUE(spin_until_arbiter_publish_count(1));
  publish_perception(make_signal(TrafficLightElement::RED));
  ASSERT_TRUE(spin_until_arbiter_publish_count(2));

  // The mismatched element stays present with its CIRCLE shape; only the color
  // collapses to UNKNOWN.
  EXPECT_EQ(observed_color(vehicle_signal), TrafficLightElement::UNKNOWN);
  EXPECT_EQ(observed_shape(vehicle_signal), TrafficLightElement::CIRCLE);
}

// source_priority is routed into arbitration: with "perception" it wins over a
// higher-confidence external element. The override rule is owned by
// core_test::perceptionPriorityOverridesConfidence.
TEST_F(ArbiterIntegration, sourcePriorityRoutesIntoArbitration)
{
  start_arbiter(false, "perception");
  publish_map();

  publish_external(make_signal(TrafficLightElement::RED, 0.99f));
  ASSERT_TRUE(spin_until_arbiter_publish_count(1));
  publish_perception(make_signal(TrafficLightElement::GREEN, 0.10f));
  ASSERT_TRUE(spin_until_arbiter_publish_count(2));

  EXPECT_EQ(observed_color(vehicle_signal), TrafficLightElement::GREEN);
}

}  // namespace
