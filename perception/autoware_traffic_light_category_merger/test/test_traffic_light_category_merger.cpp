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
// Unit tests for the ROS-free merge core (TrafficLightCategoryMerger).
//
// merge() is exercised directly over in-memory TrafficLightArray messages, with
// no rclcpp, no topics and no message_filters sync. The assertions are on the
// returned array (header source, signal ordering, element preservation) rather
// than on what a node happens to publish -- the pub/sub + sync path is covered
// by test_traffic_light_category_merger_integration.
//

#include "../src/traffic_light_category_merger.hpp"

#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/traffic_light.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#include <gtest/gtest.h>

#include <cstdint>

namespace
{
namespace tl = autoware::traffic_light;

using tier4_perception_msgs::msg::TrafficLight;
using tier4_perception_msgs::msg::TrafficLightArray;
using tier4_perception_msgs::msg::TrafficLightElement;

std_msgs::msg::Header make_header(const char * frame_id, int32_t sec)
{
  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp.sec = sec;
  header.stamp.nanosec = 0;
  return header;
}

// A signal carrying a single element so element preservation is observable.
TrafficLight make_signal(int64_t id, uint8_t type, uint8_t color)
{
  TrafficLight signal;
  signal.traffic_light_id = id;
  signal.traffic_light_type = type;
  TrafficLightElement element;
  element.color = color;
  element.shape = TrafficLightElement::CIRCLE;
  element.status = TrafficLightElement::SOLID_ON;
  element.confidence = 0.75;
  signal.elements.push_back(element);
  return signal;
}

// --------------------------------------------------------------------------
// Typical merge: car signals come first, pedestrian signals after, every signal
// preserved in order with its elements intact.
// --------------------------------------------------------------------------
TEST(TrafficLightCategoryMerger, ConcatenatesCarThenPedestrian)
{
  // Arrange
  TrafficLightArray car;
  car.header = make_header("car_frame", 10);
  car.signals.push_back(make_signal(1, TrafficLight::CAR_TRAFFIC_LIGHT, TrafficLightElement::RED));
  car.signals.push_back(
    make_signal(2, TrafficLight::CAR_TRAFFIC_LIGHT, TrafficLightElement::GREEN));
  TrafficLightArray pedestrian;
  pedestrian.header = make_header("pedestrian_frame", 20);
  pedestrian.signals.push_back(
    make_signal(3, TrafficLight::PEDESTRIAN_TRAFFIC_LIGHT, TrafficLightElement::AMBER));

  // Act
  const auto out = tl::TrafficLightCategoryMerger().merge(car, pedestrian);

  // Assert: car signals precede pedestrian signals, in input order.
  ASSERT_EQ(out.signals.size(), 3u);
  EXPECT_EQ(out.signals[0].traffic_light_id, 1);
  EXPECT_EQ(out.signals[1].traffic_light_id, 2);
  EXPECT_EQ(out.signals[2].traffic_light_id, 3);
  EXPECT_EQ(out.signals[2].traffic_light_type, TrafficLight::PEDESTRIAN_TRAFFIC_LIGHT);

  // Assert: element payloads are carried through untouched.
  ASSERT_EQ(out.signals[0].elements.size(), 1u);
  EXPECT_EQ(out.signals[0].elements[0].color, TrafficLightElement::RED);
  EXPECT_EQ(out.signals[2].elements[0].color, TrafficLightElement::AMBER);
}

// --------------------------------------------------------------------------
// The output header is always taken from the car array, never the pedestrian
// array.
// --------------------------------------------------------------------------
TEST(TrafficLightCategoryMerger, OutputHeaderComesFromCarArray)
{
  TrafficLightArray car;
  car.header = make_header("car_frame", 10);
  TrafficLightArray pedestrian;
  pedestrian.header = make_header("pedestrian_frame", 20);

  const auto out = tl::TrafficLightCategoryMerger().merge(car, pedestrian);

  EXPECT_EQ(out.header.frame_id, "car_frame");
  EXPECT_EQ(out.header.stamp.sec, 10);
}

// --------------------------------------------------------------------------
// An empty pedestrian array yields exactly the car signals.
// --------------------------------------------------------------------------
TEST(TrafficLightCategoryMerger, EmptyPedestrianKeepsCarSignals)
{
  TrafficLightArray car;
  car.header = make_header("car_frame", 10);
  car.signals.push_back(make_signal(1, TrafficLight::CAR_TRAFFIC_LIGHT, TrafficLightElement::RED));
  TrafficLightArray pedestrian;
  pedestrian.header = make_header("pedestrian_frame", 20);

  const auto out = tl::TrafficLightCategoryMerger().merge(car, pedestrian);

  ASSERT_EQ(out.signals.size(), 1u);
  EXPECT_EQ(out.signals[0].traffic_light_id, 1);
}

// --------------------------------------------------------------------------
// An empty car array yields exactly the pedestrian signals, but the header still
// comes from the (empty) car array.
// --------------------------------------------------------------------------
TEST(TrafficLightCategoryMerger, EmptyCarKeepsPedestrianSignalsAndCarHeader)
{
  TrafficLightArray car;
  car.header = make_header("car_frame", 10);
  TrafficLightArray pedestrian;
  pedestrian.header = make_header("pedestrian_frame", 20);
  pedestrian.signals.push_back(
    make_signal(3, TrafficLight::PEDESTRIAN_TRAFFIC_LIGHT, TrafficLightElement::GREEN));

  const auto out = tl::TrafficLightCategoryMerger().merge(car, pedestrian);

  ASSERT_EQ(out.signals.size(), 1u);
  EXPECT_EQ(out.signals[0].traffic_light_id, 3);
  EXPECT_EQ(out.header.frame_id, "car_frame");
}

// --------------------------------------------------------------------------
// Both arrays empty: an empty output carrying the car header.
// --------------------------------------------------------------------------
TEST(TrafficLightCategoryMerger, BothEmptyYieldsEmptyWithCarHeader)
{
  TrafficLightArray car;
  car.header = make_header("car_frame", 10);
  TrafficLightArray pedestrian;
  pedestrian.header = make_header("pedestrian_frame", 20);

  const auto out = tl::TrafficLightCategoryMerger().merge(car, pedestrian);

  EXPECT_EQ(out.signals.size(), 0u);
  EXPECT_EQ(out.header.frame_id, "car_frame");
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
