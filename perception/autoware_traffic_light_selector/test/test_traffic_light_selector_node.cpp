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

#include "../src/traffic_light_selector_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

using autoware::traffic_light::TrafficLightSelectorNode;
using sensor_msgs::msg::CameraInfo;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

namespace
{
// A single timestamp shared by every input so the ApproximateTime synchronizer fires.
const rclcpp::Time input_stamp(1, 0, RCL_ROS_TIME);
}  // namespace

class TrafficLightSelectorIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    test_node_ = std::make_shared<rclcpp::Node>("test_node");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);

    detected_rois_publisher_ = test_node_->create_publisher<DetectedObjectsWithFeature>(
      "input/detected_rois", rclcpp::QoS{1});
    rough_rois_publisher_ =
      test_node_->create_publisher<TrafficLightRoiArray>("input/rough_rois", rclcpp::QoS{1});
    expected_rois_publisher_ =
      test_node_->create_publisher<TrafficLightRoiArray>("input/expect_rois", rclcpp::QoS{1});
    camera_info_publisher_ =
      test_node_->create_publisher<CameraInfo>("input/camera_info", rclcpp::SensorDataQoS());

    output_subscription_ = test_node_->create_subscription<TrafficLightRoiArray>(
      "output/traffic_rois", rclcpp::QoS{1},
      [this](const TrafficLightRoiArray::SharedPtr message) { received_message_ = message; });

    node_ = std::make_shared<TrafficLightSelectorNode>(rclcpp::NodeOptions{});
    executor_->add_node(node_);

    // Wait for connections to be established
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
      if (
        detected_rois_publisher_->get_subscription_count() > 0 &&
        rough_rois_publisher_->get_subscription_count() > 0 &&
        expected_rois_publisher_->get_subscription_count() > 0 &&
        camera_info_publisher_->get_subscription_count() > 0 &&
        output_subscription_->get_publisher_count() > 0) {
        break;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void TearDown() override
  {
    executor_.reset();
    output_subscription_.reset();
    detected_rois_publisher_.reset();
    rough_rois_publisher_.reset();
    expected_rois_publisher_.reset();
    camera_info_publisher_.reset();
    test_node_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  TrafficLightRoiArray::SharedPtr receive_published_message(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
  {
    received_message_.reset();
    auto start = std::chrono::steady_clock::now();
    while (!received_message_) {
      if (std::chrono::steady_clock::now() - start > timeout) {
        return nullptr;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return received_message_;
  }

  // Publishes empty, synchronized inputs on all four input topics.
  void publish_empty_synchronized_inputs()
  {
    DetectedObjectsWithFeature detected_rois;
    detected_rois.header.stamp = input_stamp;
    TrafficLightRoiArray rough_rois;
    rough_rois.header.stamp = input_stamp;
    TrafficLightRoiArray expected_rois;
    expected_rois.header.stamp = input_stamp;
    CameraInfo camera_info;
    camera_info.header.stamp = input_stamp;

    detected_rois_publisher_->publish(detected_rois);
    rough_rois_publisher_->publish(rough_rois);
    expected_rois_publisher_->publish(expected_rois);
    camera_info_publisher_->publish(camera_info);
  }

  std::shared_ptr<TrafficLightSelectorNode> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Publisher<DetectedObjectsWithFeature>::SharedPtr detected_rois_publisher_;
  rclcpp::Publisher<TrafficLightRoiArray>::SharedPtr rough_rois_publisher_;
  rclcpp::Publisher<TrafficLightRoiArray>::SharedPtr expected_rois_publisher_;
  rclcpp::Publisher<CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::Subscription<TrafficLightRoiArray>::SharedPtr output_subscription_;

  TrafficLightRoiArray::SharedPtr received_message_;
};

// Publishing empty, synchronized inputs makes the node emit an empty output array, verifying
// that the node's subscribers, synchronizer and publisher are wired correctly. The selection
// logic itself is covered by the TrafficLightSelector unit tests.
TEST_F(TrafficLightSelectorIntegrationTest, EmptyInputsProduceEmptyOutput)
{
  // Act
  publish_empty_synchronized_inputs();
  const auto result = receive_published_message();

  // Assert
  ASSERT_NE(result, nullptr);
  EXPECT_TRUE(result->rois.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
