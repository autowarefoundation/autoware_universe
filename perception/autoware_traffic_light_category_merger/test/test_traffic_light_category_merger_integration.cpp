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

//
// Integration tests for TrafficLightCategoryMergerNode.
//
// These stand up the real node and drive it over its ROS interface: a car
// TrafficLightArray and a pedestrian TrafficLightArray are published on the two
// input topics, the node's message_filters ApproximateTime sync wires them
// through signals_callback, and the test observes the merged array published on
// the output topic. This exercises the full pub/sub + sync path that a unit-level
// callback invocation would bypass.
//
// Scope is intentionally narrow -- a typical (car, pedestrian) round trip and the
// both-empty case. The merge semantics themselves (ordering, header source,
// element preservation) are owned by the ROS-free core test
// (test_traffic_light_category_merger) and are only spot-checked here.
//

#include "../src/traffic_light_category_merger_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/traffic_light.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>

namespace
{
namespace tl = autoware::traffic_light;

using tier4_perception_msgs::msg::TrafficLight;
using tier4_perception_msgs::msg::TrafficLightArray;
using tier4_perception_msgs::msg::TrafficLightElement;

using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;
using std::placeholders::_1;

constexpr char car_topic[] = "/input/car_signals";
constexpr char pedestrian_topic[] = "/input/pedestrian_signals";
constexpr char output_topic[] = "/output/traffic_signals";

std_msgs::msg::Header make_header()
{
  std_msgs::msg::Header header;
  header.frame_id = "camera";
  header.stamp.sec = 123;
  header.stamp.nanosec = 456;
  return header;
}

// A signal carrying a single GREEN/CIRCLE element so the element payload is
// observable end to end.
TrafficLight make_signal(int64_t id, uint8_t type)
{
  TrafficLight signal;
  signal.traffic_light_id = id;
  signal.traffic_light_type = type;
  TrafficLightElement element;
  element.color = TrafficLightElement::GREEN;
  element.shape = TrafficLightElement::CIRCLE;
  element.status = TrafficLightElement::SOLID_ON;
  element.confidence = 0.9;
  signal.elements.push_back(element);
  return signal;
}

// Subscribes to the node's output and records the merged array it publishes. A
// named member callback keeps the capture lambda-free.
class OutputCapture
{
public:
  void on_signals(TrafficLightArray::ConstSharedPtr msg)
  {
    signals = *msg;
    got_signals = true;
  }

  bool got_signals = false;
  TrafficLightArray signals;
};

class IntegrationTest : public ::testing::Test
{
protected:
  std::shared_ptr<tl::TrafficLightCategoryMergerNode> make_node_under_test()
  {
    rclcpp::NodeOptions options;
    node_ = std::make_shared<tl::TrafficLightCategoryMergerNode>(options);
    return node_;
  }

  // Publish (car, pedestrian) on the input topics and capture the node's merged
  // output. The inputs are re-published in a wait loop so the test is robust
  // against pub/sub discovery latency; identical stamps make every pair an
  // ApproximateTime match. A missing publish is a setup failure, surfaced by
  // throwing (helpers stay free of gtest assertions).
  TrafficLightArray run(const TrafficLightArray & car, const TrafficLightArray & pedestrian)
  {
    OutputCapture capture;
    auto tester = std::make_shared<rclcpp::Node>("integration_tester");
    auto signal_sub = tester->create_subscription<TrafficLightArray>(
      output_topic, rclcpp::QoS{1}, std::bind(&OutputCapture::on_signals, &capture, _1));
    auto car_pub = tester->create_publisher<TrafficLightArray>(car_topic, rclcpp::QoS{1});
    auto pedestrian_pub =
      tester->create_publisher<TrafficLightArray>(pedestrian_topic, rclcpp::QoS{1});

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_);
    exec.add_node(tester);

    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (!capture.got_signals && std::chrono::steady_clock::now() < deadline) {
      car_pub->publish(car);
      pedestrian_pub->publish(pedestrian);
      exec.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    if (!capture.got_signals) {
      throw std::runtime_error(
        "run(): node under test published no traffic_signals within the timeout");
    }
    return capture.signals;
  }

  std::shared_ptr<tl::TrafficLightCategoryMergerNode> node_;
};

// --------------------------------------------------------------------------
// Typical round trip: a car array with one signal and a pedestrian array with
// one signal published on the inputs come back merged into one output array of
// two signals, with the car array's header propagated and the element payloads
// preserved.
// --------------------------------------------------------------------------
TEST_F(IntegrationTest, TypicalCarPedestrianRoundTrip)
{
  // Arrange
  make_node_under_test();
  TrafficLightArray car;
  car.header = make_header();  // same stamp as pedestrian -> ApproximateTime match
  car.signals.push_back(make_signal(/*id=*/1, TrafficLight::CAR_TRAFFIC_LIGHT));
  TrafficLightArray pedestrian;
  pedestrian.header = make_header();
  pedestrian.signals.push_back(make_signal(/*id=*/2, TrafficLight::PEDESTRIAN_TRAFFIC_LIGHT));

  // Act
  const auto out = run(car, pedestrian);

  // Assert: header comes from the car array; both signals are present.
  EXPECT_EQ(out.header, car.header);
  ASSERT_EQ(out.signals.size(), 2u);
  EXPECT_EQ(out.signals[0].traffic_light_id, 1);
  EXPECT_EQ(out.signals[0].traffic_light_type, TrafficLight::CAR_TRAFFIC_LIGHT);
  EXPECT_EQ(out.signals[1].traffic_light_id, 2);
  EXPECT_EQ(out.signals[1].traffic_light_type, TrafficLight::PEDESTRIAN_TRAFFIC_LIGHT);
  ASSERT_EQ(out.signals[0].elements.size(), 1u);
  EXPECT_EQ(out.signals[0].elements[0].color, TrafficLightElement::GREEN);
}

// --------------------------------------------------------------------------
// Both inputs empty: the node still publishes a (sync-matched) empty output
// array carrying the car array's header.
// --------------------------------------------------------------------------
TEST_F(IntegrationTest, BothEmptyPublishesEmptyArrayWithCarHeader)
{
  // Arrange
  make_node_under_test();
  TrafficLightArray car;
  car.header = make_header();
  TrafficLightArray pedestrian;
  pedestrian.header = make_header();

  // Act
  const auto out = run(car, pedestrian);

  // Assert
  EXPECT_EQ(out.header, car.header);
  EXPECT_EQ(out.signals.size(), 0u);
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
