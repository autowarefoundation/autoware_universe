// Copyright 2025 TIER IV, Inc.
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

#include "autoware_crosswalk_traffic_light_estimator/node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <chrono>
#include <memory>
#include <thread>

using autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimatorNode;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_perception_msgs::msg::TrafficLightGroupArray;

class CrosswalkTrafficLightEstimatorIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.append_parameter_override("use_last_detect_color", true);
    options.append_parameter_override("use_pedestrian_signal_detect", true);
    options.append_parameter_override("last_detect_color_hold_time", 2.0);
    options.append_parameter_override("last_colors_hold_time", 1.0);

    node_ = std::make_shared<CrosswalkTrafficLightEstimatorNode>(options);
    test_node_ = std::make_shared<rclcpp::Node>("test_node");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(test_node_);

    map_pub_ = test_node_->create_publisher<LaneletMapBin>(
      "/crosswalk_traffic_light_estimator/input/vector_map", rclcpp::QoS(1).transient_local());
    signal_pub_ = test_node_->create_publisher<TrafficLightGroupArray>(
      "/crosswalk_traffic_light_estimator/input/classified/traffic_signals", rclcpp::QoS(1));

    message_received_ = false;
    output_sub_ = test_node_->create_subscription<TrafficLightGroupArray>(
      "/crosswalk_traffic_light_estimator/output/traffic_signals", rclcpp::QoS(1),
      [this](const TrafficLightGroupArray::SharedPtr msg) {
        received_msg_ = msg;
        message_received_ = true;
      });
  }

  void TearDown() override
  {
    executor_.reset();
    node_.reset();
    test_node_.reset();
    rclcpp::shutdown();
  }

  /// @brief Create a lanelet map with a vehicle road and a perpendicular crosswalk.
  /// The vehicle lanelet (east-west, "straight") overlaps geometrically with the crosswalk
  /// (north-south), so conflictingInGraph detects the relationship.
  ///
  ///   y
  ///   8 +              cl2 -------- cr2
  ///     |               | crosswalk |
  ///   3 + vl1 ----------|---(VTL)---|----------- vl2    (vehicle left bound)
  ///     |               |           |
  ///   0 +               |           |
  ///     |               |           |
  ///  -3 + vr1 ----------|-----------|----------- vr2    (vehicle right bound)
  ///     |               |           |
  ///  -8 +              cl1 -(CTL)- cr1
  ///     +---+---+---+--+----+------+---+---+--> x
  ///     0               13  15     17      30
  ///
  ///   Vehicle lanelet: subtype="road", turn_direction="straight", TL=vehicle_tl_re_id_
  ///   Crosswalk lanelet: subtype="crosswalk", TL=crosswalk_tl_re_id_
  ///   (VTL): vehicle traffic light + stop line at x=13
  LaneletMapBin createMap()
  {
    auto map = std::make_shared<lanelet::LaneletMap>();

    // --- Vehicle lanelet (east-west road, turn_direction="straight") ---
    lanelet::Point3d vl1(lanelet::utils::getId(), 0.0, 3.0, 0.0);
    lanelet::Point3d vl2(lanelet::utils::getId(), 30.0, 3.0, 0.0);
    lanelet::Point3d vr1(lanelet::utils::getId(), 0.0, -3.0, 0.0);
    lanelet::Point3d vr2(lanelet::utils::getId(), 30.0, -3.0, 0.0);

    lanelet::LineString3d vehicle_left(lanelet::utils::getId(), {vl1, vl2});
    lanelet::LineString3d vehicle_right(lanelet::utils::getId(), {vr1, vr2});

    lanelet::Lanelet vehicle_lanelet(lanelet::utils::getId(), vehicle_left, vehicle_right);
    vehicle_lanelet.attributes()["subtype"] = "road";
    vehicle_lanelet.attributes()["turn_direction"] = "straight";

    // Vehicle traffic light regulatory element
    lanelet::Point3d vtl_p(lanelet::utils::getId(), 13.0, 3.0, 5.0);
    lanelet::LineString3d vehicle_tl_shape(lanelet::utils::getId(), {vtl_p});

    lanelet::Point3d vsl1(lanelet::utils::getId(), 13.0, 3.0, 0.0);
    lanelet::Point3d vsl2(lanelet::utils::getId(), 13.0, -3.0, 0.0);
    lanelet::LineString3d vehicle_stop_line(lanelet::utils::getId(), {vsl1, vsl2});

    vehicle_tl_re_id_ = lanelet::utils::getId();
    auto vehicle_tl_re = lanelet::TrafficLight::make(
      vehicle_tl_re_id_, lanelet::AttributeMap(), {vehicle_tl_shape}, vehicle_stop_line);
    vehicle_lanelet.addRegulatoryElement(vehicle_tl_re);

    // --- Crosswalk lanelet (north-south, overlaps vehicle lanelet at x=[13,17]) ---
    lanelet::Point3d cl1(lanelet::utils::getId(), 13.0, -8.0, 0.0);
    lanelet::Point3d cl2(lanelet::utils::getId(), 13.0, 8.0, 0.0);
    lanelet::Point3d cr1(lanelet::utils::getId(), 17.0, -8.0, 0.0);
    lanelet::Point3d cr2(lanelet::utils::getId(), 17.0, 8.0, 0.0);

    lanelet::LineString3d crosswalk_left(lanelet::utils::getId(), {cl1, cl2});
    lanelet::LineString3d crosswalk_right(lanelet::utils::getId(), {cr1, cr2});

    lanelet::Lanelet crosswalk_lanelet(lanelet::utils::getId(), crosswalk_left, crosswalk_right);
    crosswalk_lanelet.attributes()["subtype"] = "crosswalk";

    // Crosswalk traffic light regulatory element
    lanelet::Point3d ctl_p(lanelet::utils::getId(), 15.0, -8.0, 3.0);
    lanelet::LineString3d crosswalk_tl_shape(lanelet::utils::getId(), {ctl_p});

    lanelet::Point3d csl1(lanelet::utils::getId(), 13.0, -3.0, 0.0);
    lanelet::Point3d csl2(lanelet::utils::getId(), 17.0, -3.0, 0.0);
    lanelet::LineString3d crosswalk_stop_line(lanelet::utils::getId(), {csl1, csl2});

    crosswalk_tl_re_id_ = lanelet::utils::getId();
    auto crosswalk_tl_re = lanelet::TrafficLight::make(
      crosswalk_tl_re_id_, lanelet::AttributeMap(), {crosswalk_tl_shape}, crosswalk_stop_line);
    crosswalk_lanelet.addRegulatoryElement(crosswalk_tl_re);

    map->add(vehicle_lanelet);
    map->add(crosswalk_lanelet);

    auto msg = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
    msg.header.frame_id = "map";
    return msg;
  }

  TrafficLightGroupArray createTrafficSignal(lanelet::Id tl_id, uint8_t color)
  {
    TrafficLightGroupArray msg;
    msg.stamp = node_->now();

    TrafficLightGroup group;
    group.traffic_light_group_id = tl_id;

    TrafficLightElement element;
    element.color = color;
    element.shape = TrafficLightElement::CIRCLE;
    element.status = TrafficLightElement::SOLID_ON;
    element.confidence = 1.0;

    group.elements.push_back(element);
    msg.traffic_light_groups.push_back(group);
    return msg;
  }

  bool waitForMessage(std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
  {
    auto start = std::chrono::steady_clock::now();
    message_received_ = false;
    while (!message_received_) {
      if (std::chrono::steady_clock::now() - start > timeout) {
        return false;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::shared_ptr<CrosswalkTrafficLightEstimatorNode> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Publisher<LaneletMapBin>::SharedPtr map_pub_;
  rclcpp::Publisher<TrafficLightGroupArray>::SharedPtr signal_pub_;
  rclcpp::Subscription<TrafficLightGroupArray>::SharedPtr output_sub_;
  TrafficLightGroupArray::SharedPtr received_msg_;
  bool message_received_ = false;

  lanelet::Id vehicle_tl_re_id_{0};
  lanelet::Id crosswalk_tl_re_id_{0};

  void publishMap()
  {
    auto map_msg = createMap();
    map_pub_->publish(map_msg);
    spinFor(std::chrono::milliseconds(500));
  }

  void publishTrafficSignal(uint8_t vehicle_color)
  {
    auto signal = createTrafficSignal(vehicle_tl_re_id_, vehicle_color);
    signal_pub_->publish(signal);
    ASSERT_TRUE(waitForMessage());
  }

  /// @brief Find the crosswalk signal color in received_msg_.
  /// Fails the test if not found.
  uint8_t getCrosswalkColor() const
  {
    for (const auto & group : received_msg_->traffic_light_groups) {
      if (group.traffic_light_group_id == crosswalk_tl_re_id_) {
        EXPECT_FALSE(group.elements.empty());
        return group.elements.front().color;
      }
    }
    ADD_FAILURE() << "Crosswalk traffic signal not found in output";
    return TrafficLightElement::UNKNOWN;
  }
};

/// @brief Vehicle straight lanelet GREEN → crosswalk is estimated as RED.
TEST_F(CrosswalkTrafficLightEstimatorIntegrationTest, StraightGreenEstimatesRed)
{
  // Arrange
  publishMap();

  // Act
  publishTrafficSignal(TrafficLightElement::GREEN);
  publishTrafficSignal(TrafficLightElement::GREEN);

  // Assert
  EXPECT_EQ(getCrosswalkColor(), TrafficLightElement::RED);
}

/// @brief Vehicle signal RED → all vehicle lanelets are RED → crosswalk is estimated as UNKNOWN.
TEST_F(CrosswalkTrafficLightEstimatorIntegrationTest, AllRedEstimatesUnknown)
{
  // Arrange
  publishMap();

  // Act
  publishTrafficSignal(TrafficLightElement::RED);
  publishTrafficSignal(TrafficLightElement::RED);

  // Assert
  EXPECT_EQ(getCrosswalkColor(), TrafficLightElement::UNKNOWN);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
