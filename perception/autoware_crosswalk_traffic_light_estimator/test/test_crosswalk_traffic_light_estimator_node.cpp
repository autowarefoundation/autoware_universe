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

#include "../src/node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <chrono>
#include <memory>
#include <thread>

using autoware::crosswalk_traffic_light_estimator::CrosswalkTrafficLightEstimatorNode;
using autoware_map_msgs::msg::LaneletMapBin;
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

  bool wait_for_message(std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
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

  void spin_for(std::chrono::milliseconds duration)
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
};

/// @brief Empty map and empty traffic signal input produces empty traffic signal output.
TEST_F(CrosswalkTrafficLightEstimatorIntegrationTest, EmptyInputProducesEmptyOutput)
{
  // Arrange
  auto empty_lanelet_map = std::make_shared<lanelet::LaneletMap>();
  auto empty_map = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(empty_lanelet_map);
  empty_map.header.frame_id = "map";
  map_pub_->publish(empty_map);
  spin_for(std::chrono::milliseconds(500));

  // Act
  TrafficLightGroupArray empty_signal;
  empty_signal.stamp = node_->now();
  signal_pub_->publish(empty_signal);

  // Assert
  ASSERT_TRUE(wait_for_message());
  EXPECT_TRUE(received_msg_->traffic_light_groups.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
