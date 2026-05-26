// Copyright 2026 Tier IV, Inc.
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

#include "traffic_light_map_visualizer/traffic_light_visualizer_node.hpp"

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
#include <memory>
#include <utility>

using autoware::traffic_light::TrafficLightMapVisualizerNode;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using visualization_msgs::msg::MarkerArray;

namespace
{

struct TestMap
{
  LaneletMapBin msg;
  lanelet::Id traffic_light_group_id;
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
// Mirrors the characterization test's map shape so the smoke test exercises a
// realistic G/Y/R bulb set; only marker count is asserted, the rich behavior
// (color/position) is left to test_traffic_light_map_visualizer.cpp.
TestMap make_test_map()
{
  using lanelet::Lanelet;
  using lanelet::LineString3d;
  using lanelet::LineStringOrPolygon3d;
  using lanelet::Point3d;
  using lanelet::utils::getId;

  Point3d green_bulb(getId(), 1.0, 2.0, 5.0);
  green_bulb.attributes()["color"] = "green";

  Point3d yellow_bulb(getId(), 1.5, 2.0, 5.0);
  yellow_bulb.attributes()["color"] = "yellow";

  Point3d red_bulb(getId(), 2.0, 2.0, 5.0);
  red_bulb.attributes()["color"] = "red";

  LineString3d light_bulbs(getId(), {green_bulb, yellow_bulb, red_bulb});
  light_bulbs.attributes()["traffic_light_id"] = "1";

  LineString3d traffic_light_base(
    getId(), {Point3d(getId(), 0.0, 2.0, 4.0), Point3d(getId(), 3.0, 2.0, 4.0)});

  auto traffic_light = lanelet::autoware::AutowareTrafficLight::make(
    getId(), lanelet::AttributeMap(), {LineStringOrPolygon3d(traffic_light_base)}, {},
    {light_bulbs});

  LineString3d left_bound(
    getId(), {Point3d(getId(), 0.0, 0.0, 0.0), Point3d(getId(), 0.0, 10.0, 0.0)});
  LineString3d right_bound(
    getId(), {Point3d(getId(), 3.0, 0.0, 0.0), Point3d(getId(), 3.0, 10.0, 0.0)});
  Lanelet lanelet(getId(), left_bound, right_bound);
  lanelet.addRegulatoryElement(traffic_light);

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lanelet);

  TestMap result;
  result.msg = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  result.traffic_light_group_id = traffic_light->id();
  return result;
}

TrafficLightGroupArray make_all_colors_detection(lanelet::Id group_id)
{
  TrafficLightGroupArray msg;
  TrafficLightGroup group;
  group.traffic_light_group_id = group_id;
  for (auto color :
       {TrafficLightElement::RED, TrafficLightElement::GREEN, TrafficLightElement::AMBER}) {
    TrafficLightElement element;
    element.color = color;
    group.elements.push_back(element);
  }
  msg.traffic_light_groups.push_back(group);
  return msg;
}

}  // namespace

class TestTrafficLightMapVisualizerNodeSmoke : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

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

  void spin_some(std::chrono::milliseconds duration = std::chrono::milliseconds(500))
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_some(std::chrono::milliseconds(10));
    }
  }

  bool spin_until_received(std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
  {
    auto start = std::chrono::steady_clock::now();
    while (!received_markers_ && std::chrono::steady_clock::now() - start < timeout) {
      executor_->spin_some(std::chrono::milliseconds(10));
    }
    return received_markers_ != nullptr;
  }

  std::shared_ptr<TrafficLightMapVisualizerNode> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Publisher<LaneletMapBin>::SharedPtr map_pub_;
  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr traffic_light_pub_;
  rclcpp::Subscription<MarkerArray>::SharedPtr marker_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  MarkerArray::ConstSharedPtr received_markers_;
};

// Smoke: data flows from map + detection inputs through the Node and produces
// the expected number of markers on the output topic. Detailed marker contents
// (color, position) are covered by the characterization tests in
// test_traffic_light_map_visualizer.cpp.
TEST_F(TestTrafficLightMapVisualizerNodeSmoke, MapAndDetectionProduceMarkers)
{
  const auto test_map = make_test_map();
  map_pub_->publish(test_map.msg);
  spin_some();

  traffic_light_pub_->publish(make_all_colors_detection(test_map.traffic_light_group_id));
  ASSERT_TRUE(spin_until_received());

  EXPECT_GT(received_markers_->markers.size(), 0u);
}

// Smoke: detection arriving before the map is silently dropped, guarded by the
// `if (!visualizer_) return;` check in detected_traffic_lights_callback. This
// path is only reachable at the node level; unit and characterization tests
// always supply the map first.
TEST_F(TestTrafficLightMapVisualizerNodeSmoke, DetectionBeforeMapProducesNoMarkers)
{
  // Publish detection without ever publishing the map. The group id is
  // arbitrary because the callback returns before looking it up.
  traffic_light_pub_->publish(make_all_colors_detection(/*arbitrary*/ 1));
  spin_some();

  EXPECT_EQ(received_markers_, nullptr);
}
