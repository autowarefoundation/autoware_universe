// Copyright 2024 The Autoware Contributors
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

#include "traffic_light_map_visualizer/node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <chrono>
#include <map>
#include <memory>
#include <vector>

using autoware::traffic_light::TrafficLightMapVisualizerNode;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{

struct TestMapResult
{
  LaneletMapBin msg;
  lanelet::Id traffic_light_group_id;
  lanelet::Id red_bulb_id;
  lanelet::Id green_bulb_id;
  lanelet::Id yellow_bulb_id;
};

// Creates the following map structure (XZ side view, all elements at y=2.0):
//
//         Z
//         ^
//    5.0  |     ●  ●  ●      <- lightBulbs
//         |     G  Y  R        green(1.0) yellow(1.5) red(2.0)
//    4.0  |  +--------+      <- traffic light base
//         |
//    0.0  |  L────────R      <- lanelet bounds (y: 0.0 to 10.0)
//         +──────────────> X
//         0.0 1.0 1.5 2.0 3.0
//
// The node only uses lightBulbs (id, position, color attribute).
// Lanelet, traffic light base, and stop line exist solely to satisfy
// the autowareTrafficLights() query.
TestMapResult create_test_map_with_traffic_light()
{
  using lanelet::Lanelet;
  using lanelet::LineString3d;
  using lanelet::LineStringOrPolygon3d;
  using lanelet::Point3d;
  using lanelet::utils::getId;

  // Light bulb points with color attribute (G, Y, R order like a Japanese traffic light)
  Point3d green_bulb(getId(), 1.0, 2.0, 5.0);
  green_bulb.attributes()["color"] = "green";

  Point3d yellow_bulb(getId(), 1.5, 2.0, 5.0);
  yellow_bulb.attributes()["color"] = "yellow";

  Point3d red_bulb(getId(), 2.0, 2.0, 5.0);
  red_bulb.attributes()["color"] = "red";

  // Light bulbs linestring (must have "traffic_light_id" attribute)
  LineString3d light_bulbs(getId(), {green_bulb, yellow_bulb, red_bulb});
  light_bulbs.attributes()["traffic_light_id"] = "1";

  // Traffic light base linestring (not used by the node, but required by make())
  LineString3d traffic_light_base(
    getId(), {Point3d(getId(), 0.0, 2.0, 4.0), Point3d(getId(), 3.0, 2.0, 4.0)});

  // AutowareTrafficLight regulatory element (stop_line omitted — optional and unused by the node)
  auto traffic_light_regulatory_element = lanelet::autoware::AutowareTrafficLight::make(
    getId(), lanelet::AttributeMap(), {LineStringOrPolygon3d(traffic_light_base)}, {},
    {light_bulbs});

  // Lanelet to hold the regulatory element (required for query::autowareTrafficLights)
  LineString3d left_bound(
    getId(), {Point3d(getId(), 0.0, 0.0, 0.0), Point3d(getId(), 0.0, 10.0, 0.0)});
  LineString3d right_bound(
    getId(), {Point3d(getId(), 3.0, 0.0, 0.0), Point3d(getId(), 3.0, 10.0, 0.0)});
  Lanelet lanelet(getId(), left_bound, right_bound);
  lanelet.addRegulatoryElement(traffic_light_regulatory_element);

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lanelet);

  TestMapResult result;
  result.msg = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  result.traffic_light_group_id = traffic_light_regulatory_element->id();
  result.red_bulb_id = red_bulb.id();
  result.green_bulb_id = green_bulb.id();
  result.yellow_bulb_id = yellow_bulb.id();
  return result;
}

TrafficLightGroupArray create_traffic_light_msg(
  lanelet::Id group_id, const std::vector<uint8_t> & colors)
{
  TrafficLightGroupArray msg;
  TrafficLightGroup group;
  group.traffic_light_group_id = group_id;
  for (auto color : colors) {
    TrafficLightElement element;
    element.color = color;
    element.shape = TrafficLightElement::CIRCLE;  // not used by the node, but set for completeness
    element.status =
      TrafficLightElement::SOLID_ON;  // not used by the node, but set for completeness
    element.confidence = 1.0f;        // not used by the node, but set for completeness
    group.elements.push_back(element);
  }
  msg.traffic_light_groups.push_back(group);
  return msg;
}

}  // namespace

class TestTrafficLightMapVisualizer : public ::testing::Test
{
protected:
  // rclcpp::init/shutdown are called once per test suite (not per test),
  // because rclcpp::init can only be called once per process.
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  // Node and pub/sub are recreated per test to ensure a clean state.
  // The node under test has no persistent side effects beyond its member variables,
  // so destroying and recreating it is sufficient to isolate each test.
  void SetUp() override
  {
    node_ = std::make_shared<TrafficLightMapVisualizerNode>(rclcpp::NodeOptions());
    test_node_ = std::make_shared<rclcpp::Node>("test_helper");

    map_pub_ = test_node_->create_publisher<LaneletMapBin>(
      "/traffic_light_map_visualizer_node/input/vector_map", rclcpp::QoS{1}.transient_local());
    traffic_light_pub_ = test_node_->create_publisher<TrafficLightGroupArray>(
      "/traffic_light_map_visualizer_node/input/tl_state", 1);
    marker_sub_ = test_node_->create_subscription<MarkerArray>(
      "/traffic_light_map_visualizer_node/output/traffic_light", 1,
      [this](MarkerArray::ConstSharedPtr msg) { received_markers_ = msg; });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(test_node_);
    received_markers_.reset();
  }

  void TearDown() override
  {
    executor_.reset();
    marker_sub_.reset();
    traffic_light_pub_.reset();
    map_pub_.reset();
    test_node_.reset();
    node_.reset();
  }

  bool spin_until_received(std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
  {
    auto start = std::chrono::steady_clock::now();
    while (!received_markers_ && std::chrono::steady_clock::now() - start < timeout) {
      executor_->spin_some(std::chrono::milliseconds(10));
    }
    return received_markers_ != nullptr;
  }

  void spin_some(std::chrono::milliseconds duration = std::chrono::milliseconds(500))
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_some(std::chrono::milliseconds(10));
    }
  }

  void publish_map_and_wait(const LaneletMapBin & map_msg)
  {
    map_pub_->publish(map_msg);
    spin_some();
  }

  void publish_traffic_light_and_wait(lanelet::Id group_id, const std::vector<uint8_t> & colors)
  {
    traffic_light_pub_->publish(create_traffic_light_msg(group_id, colors));
    ASSERT_TRUE(spin_until_received());
  }

  std::shared_ptr<TrafficLightMapVisualizerNode> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Publisher<LaneletMapBin>::SharedPtr map_pub_;
  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr traffic_light_pub_;
  rclcpp::Subscription<MarkerArray>::SharedPtr marker_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  MarkerArray::ConstSharedPtr received_markers_;
};

// Characterization: RED traffic light with matching map produces one red sphere marker
TEST_F(TestTrafficLightMapVisualizer, RedSignalProducesOneRedMarker)
{
  // Arrange
  auto test_map = create_test_map_with_traffic_light();
  publish_map_and_wait(test_map.msg);

  // Act
  publish_traffic_light_and_wait(test_map.traffic_light_group_id, {TrafficLightElement::RED});

  // Assert
  ASSERT_EQ(received_markers_->markers.size(), 1u);

  const auto & m = received_markers_->markers[0];
  EXPECT_EQ(m.header.frame_id, "map");
  EXPECT_EQ(m.ns, "traffic_light");
  EXPECT_EQ(m.type, Marker::SPHERE);
  EXPECT_EQ(m.id, static_cast<int32_t>(test_map.red_bulb_id));
  EXPECT_DOUBLE_EQ(m.pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(m.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(m.pose.position.z, 5.0);
  EXPECT_FLOAT_EQ(m.color.r, 1.0f);
  EXPECT_FLOAT_EQ(m.color.g, 0.0f);
  EXPECT_FLOAT_EQ(m.color.b, 0.0f);
}

// Characterization: GREEN traffic light produces one green marker
TEST_F(TestTrafficLightMapVisualizer, GreenSignalProducesOneGreenMarker)
{
  // Arrange
  auto test_map = create_test_map_with_traffic_light();
  publish_map_and_wait(test_map.msg);

  // Act
  publish_traffic_light_and_wait(test_map.traffic_light_group_id, {TrafficLightElement::GREEN});

  // Assert
  ASSERT_EQ(received_markers_->markers.size(), 1u);
  const auto & m = received_markers_->markers[0];
  EXPECT_EQ(m.id, static_cast<int32_t>(test_map.green_bulb_id));
  EXPECT_DOUBLE_EQ(m.pose.position.x, 1.0);
  EXPECT_FLOAT_EQ(m.color.r, 0.0f);
  EXPECT_FLOAT_EQ(m.color.g, 1.0f);
  EXPECT_FLOAT_EQ(m.color.b, 0.0f);
}

// Characterization: AMBER traffic light produces one yellow marker
TEST_F(TestTrafficLightMapVisualizer, AmberSignalProducesOneYellowMarker)
{
  // Arrange
  auto test_map = create_test_map_with_traffic_light();
  publish_map_and_wait(test_map.msg);

  // Act
  publish_traffic_light_and_wait(test_map.traffic_light_group_id, {TrafficLightElement::AMBER});

  // Assert
  ASSERT_EQ(received_markers_->markers.size(), 1u);
  const auto & m = received_markers_->markers[0];
  EXPECT_EQ(m.id, static_cast<int32_t>(test_map.yellow_bulb_id));
  EXPECT_DOUBLE_EQ(m.pose.position.x, 1.5);
  EXPECT_FLOAT_EQ(m.color.r, 1.0f);
  EXPECT_FLOAT_EQ(m.color.g, 1.0f);
  EXPECT_FLOAT_EQ(m.color.b, 0.0f);
}

// Characterization: all three colors produce three markers with correct colors
TEST_F(TestTrafficLightMapVisualizer, AllColorsProduceThreeMarkers)
{
  // Arrange
  auto test_map = create_test_map_with_traffic_light();
  publish_map_and_wait(test_map.msg);

  // Act
  publish_traffic_light_and_wait(
    test_map.traffic_light_group_id,
    {TrafficLightElement::RED, TrafficLightElement::GREEN, TrafficLightElement::AMBER});

  // Assert
  ASSERT_EQ(received_markers_->markers.size(), 3u);

  std::map<int32_t, const Marker *> markers_by_bulb_id;
  for (const auto & m : received_markers_->markers) {
    markers_by_bulb_id[m.id] = &m;
  }

  // Red
  {
    auto it = markers_by_bulb_id.find(static_cast<int32_t>(test_map.red_bulb_id));
    ASSERT_NE(it, markers_by_bulb_id.end());
    EXPECT_FLOAT_EQ(it->second->color.r, 1.0f);
    EXPECT_FLOAT_EQ(it->second->color.g, 0.0f);
    EXPECT_FLOAT_EQ(it->second->color.b, 0.0f);
  }

  // Green
  {
    auto it = markers_by_bulb_id.find(static_cast<int32_t>(test_map.green_bulb_id));
    ASSERT_NE(it, markers_by_bulb_id.end());
    EXPECT_FLOAT_EQ(it->second->color.r, 0.0f);
    EXPECT_FLOAT_EQ(it->second->color.g, 1.0f);
    EXPECT_FLOAT_EQ(it->second->color.b, 0.0f);
  }

  // Yellow
  {
    auto it = markers_by_bulb_id.find(static_cast<int32_t>(test_map.yellow_bulb_id));
    ASSERT_NE(it, markers_by_bulb_id.end());
    EXPECT_FLOAT_EQ(it->second->color.r, 1.0f);
    EXPECT_FLOAT_EQ(it->second->color.g, 1.0f);
    EXPECT_FLOAT_EQ(it->second->color.b, 0.0f);
  }
}

// Characterization: non-matching group ID → empty MarkerArray
TEST_F(TestTrafficLightMapVisualizer, NonMatchingGroupIdProducesEmptyMarkers)
{
  // Arrange
  auto test_map = create_test_map_with_traffic_light();
  publish_map_and_wait(test_map.msg);

  // Act
  const lanelet::Id non_existent_id = test_map.traffic_light_group_id + 1;
  publish_traffic_light_and_wait(non_existent_id, {TrafficLightElement::RED});

  // Assert
  EXPECT_EQ(received_markers_->markers.size(), 0u);
}
