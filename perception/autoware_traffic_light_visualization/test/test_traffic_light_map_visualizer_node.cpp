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

struct MinimalMap
{
  LaneletMapBin msg;
  lanelet::Id traffic_light_group_id;
};

// Minimal lanelet map: 1 lanelet holding 1 AutowareTrafficLight regulatory
// element with a single RED bulb. Smaller than the characterization test's
// 3-color map; just enough to exercise the Node's pub/sub wiring.
MinimalMap make_minimal_map()
{
  using lanelet::Lanelet;
  using lanelet::LineString3d;
  using lanelet::LineStringOrPolygon3d;
  using lanelet::Point3d;
  using lanelet::utils::getId;

  Point3d red_bulb(getId(), 0.0, 0.0, 1.0);
  red_bulb.attributes()["color"] = "red";

  LineString3d light_bulbs(getId(), {red_bulb});
  light_bulbs.attributes()["traffic_light_id"] = "1";

  LineString3d traffic_light_base(
    getId(), {Point3d(getId(), 0.0, 0.0, 0.0), Point3d(getId(), 1.0, 0.0, 0.0)});

  auto regulatory_element = lanelet::autoware::AutowareTrafficLight::make(
    getId(), lanelet::AttributeMap(), {LineStringOrPolygon3d(traffic_light_base)}, {},
    {light_bulbs});

  LineString3d left_bound(
    getId(), {Point3d(getId(), 0.0, 0.0, 0.0), Point3d(getId(), 0.0, 1.0, 0.0)});
  LineString3d right_bound(
    getId(), {Point3d(getId(), 1.0, 0.0, 0.0), Point3d(getId(), 1.0, 1.0, 0.0)});
  Lanelet lanelet(getId(), left_bound, right_bound);
  lanelet.addRegulatoryElement(regulatory_element);

  auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lanelet);

  MinimalMap result;
  result.msg = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
  result.traffic_light_group_id = regulatory_element->id();
  return result;
}

TrafficLightGroupArray make_red_detection(lanelet::Id group_id)
{
  TrafficLightGroupArray msg;
  TrafficLightGroup group;
  group.traffic_light_group_id = group_id;
  TrafficLightElement element;
  element.color = TrafficLightElement::RED;
  group.elements.push_back(element);
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
// at least one marker on the output topic. Detailed marker contents are
// covered by the characterization tests in test_traffic_light_map_visualizer.cpp.
TEST_F(TestTrafficLightMapVisualizerNodeSmoke, MapAndDetectionProduceMarker)
{
  const auto minimal_map = make_minimal_map();
  map_pub_->publish(minimal_map.msg);
  spin_some();

  traffic_light_pub_->publish(make_red_detection(minimal_map.traffic_light_group_id));
  ASSERT_TRUE(spin_until_received());

  EXPECT_EQ(received_markers_->markers.size(), 1u);
}
