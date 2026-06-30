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
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <tier4_perception_msgs/msg/detected_object_with_feature.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

using autoware::traffic_light::TrafficLightSelectorNode;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::RegionOfInterest;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;
using tier4_perception_msgs::msg::TrafficLightRoi;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

namespace
{
// A single timestamp shared by every input so the ApproximateTime synchronizer fires.
const rclcpp::Time input_stamp(1, 0, RCL_ROS_TIME);
}  // namespace

RegionOfInterest make_roi(uint32_t x_offset, uint32_t y_offset, uint32_t width, uint32_t height)
{
  RegionOfInterest roi;
  roi.x_offset = x_offset;
  roi.y_offset = y_offset;
  roi.width = width;
  roi.height = height;
  return roi;
}

bool is_same(const RegionOfInterest & lhs, const RegionOfInterest & rhs)
{
  return lhs.x_offset == rhs.x_offset && lhs.y_offset == rhs.y_offset && lhs.width == rhs.width &&
         lhs.height == rhs.height;
}

CameraInfo make_camera_info(uint32_t width, uint32_t height)
{
  CameraInfo camera_info;
  camera_info.header.stamp = input_stamp;
  camera_info.header.frame_id = "camera";
  camera_info.width = width;
  camera_info.height = height;
  return camera_info;
}

DetectedObjectsWithFeature make_detected_rois(const std::vector<RegionOfInterest> & rois)
{
  DetectedObjectsWithFeature detected_rois;
  detected_rois.header.stamp = input_stamp;
  detected_rois.header.frame_id = "camera";
  for (const auto & roi : rois) {
    DetectedObjectWithFeature feature_object;
    feature_object.feature.roi = roi;
    detected_rois.feature_objects.push_back(feature_object);
  }
  return detected_rois;
}

TrafficLightRoi make_traffic_light_roi(int64_t traffic_light_id, const RegionOfInterest & roi)
{
  TrafficLightRoi traffic_light_roi;
  traffic_light_roi.traffic_light_id = traffic_light_id;
  traffic_light_roi.traffic_light_type = TrafficLightRoi::CAR_TRAFFIC_LIGHT;
  traffic_light_roi.roi = roi;
  return traffic_light_roi;
}

TrafficLightRoiArray make_traffic_light_roi_array(const std::vector<TrafficLightRoi> & rois)
{
  TrafficLightRoiArray traffic_light_roi_array;
  traffic_light_roi_array.header.stamp = input_stamp;
  traffic_light_roi_array.header.frame_id = "camera";
  traffic_light_roi_array.rois = rois;
  return traffic_light_roi_array;
}

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

  // Publishes all four synchronized inputs and waits for the node to emit the output.
  void publish_inputs(
    const DetectedObjectsWithFeature & detected_rois, const TrafficLightRoiArray & rough_rois,
    const TrafficLightRoiArray & expected_rois, const CameraInfo & camera_info)
  {
    detected_rois_publisher_->publish(detected_rois);
    rough_rois_publisher_->publish(rough_rois);
    expected_rois_publisher_->publish(expected_rois);
    camera_info_publisher_->publish(camera_info);
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

// With no expected ROIs the output loop produces nothing, so the node emits an empty array.
TEST_F(TrafficLightSelectorIntegrationTest, EmptyExpectedRoisOutputsEmptyArray)
{
  // Arrange
  const auto detected_rois = make_detected_rois({});
  const auto rough_rois = make_traffic_light_roi_array({});
  const auto expected_rois = make_traffic_light_roi_array({});
  const auto camera_info = make_camera_info(1280, 720);

  // Act
  publish_inputs(detected_rois, rough_rois, expected_rois, camera_info);
  const auto result = receive_published_message();

  // Assert
  ASSERT_NE(result, nullptr);
  EXPECT_TRUE(result->rois.empty());
}

// The detected ROI center lies outside the rough ROI, so no detection is matched. The node still
// emits one entry for the expected traffic light, but with a default (empty) ROI.
TEST_F(TrafficLightSelectorIntegrationTest, DetectionOutsideRoughRoiOutputsDefaultRoi)
{
  // Arrange
  const auto traffic_light_id = 123;
  const auto expected_roi = make_roi(100, 100, 40, 40);
  const auto rough_roi = make_roi(50, 50, 200, 200);
  // Detected ROI center (820, 620) is far outside the rough ROI span (50..250).
  const auto detected_roi = make_roi(800, 600, 40, 40);
  const RegionOfInterest expected_default_roi;

  const auto detected_rois = make_detected_rois({detected_roi});
  const auto rough_rois =
    make_traffic_light_roi_array({make_traffic_light_roi(traffic_light_id, rough_roi)});
  const auto expected_rois =
    make_traffic_light_roi_array({make_traffic_light_roi(traffic_light_id, expected_roi)});
  const auto camera_info = make_camera_info(1280, 720);

  // Act
  publish_inputs(detected_rois, rough_rois, expected_rois, camera_info);
  const auto result = receive_published_message();

  // Assert
  ASSERT_NE(result, nullptr);
  ASSERT_EQ(result->rois.size(), 1u);
  EXPECT_EQ(result->rois.front().traffic_light_id, traffic_light_id);
  EXPECT_TRUE(is_same(result->rois.front().roi, expected_default_roi));
}

// The detected ROI center is inside the rough ROI and overlaps the expected ROI, so
// the selector matches it, accumulates a positive IoU, and assigns the detected ROI to the output.
TEST_F(TrafficLightSelectorIntegrationTest, DetectionInsideRoughRoiIsAssignedToOutput)
{
  // Arrange
  const auto traffic_light_id = 123;
  const auto expected_roi = make_roi(100, 100, 40, 40);
  const auto rough_roi = make_roi(50, 50, 200, 200);
  // Detected ROI center (120, 120) is inside the rough ROI and coincides with the expected ROI.
  const auto detected_roi = make_roi(100, 100, 40, 40);
  const auto expected_assigned_roi = detected_roi;

  const auto detected_rois = make_detected_rois({detected_roi});
  const auto rough_rois =
    make_traffic_light_roi_array({make_traffic_light_roi(traffic_light_id, rough_roi)});
  const auto expected_rois =
    make_traffic_light_roi_array({make_traffic_light_roi(traffic_light_id, expected_roi)});
  const auto camera_info = make_camera_info(1280, 720);

  // Act
  publish_inputs(detected_rois, rough_rois, expected_rois, camera_info);
  const auto result = receive_published_message();

  // Assert
  ASSERT_NE(result, nullptr);
  ASSERT_EQ(result->rois.size(), 1u);
  EXPECT_EQ(result->rois.front().traffic_light_id, traffic_light_id);
  EXPECT_TRUE(is_same(result->rois.front().roi, expected_assigned_roi));
}

// Multiple detected ROIs have their center inside the rough ROI, so they all become selection
// candidates. The selector must pick the one with the highest IoU against the expected ROI. The
// poorly-overlapping candidate is published first to prove the choice is IoU-driven, not
// order-driven.
TEST_F(TrafficLightSelectorIntegrationTest, HighestIouCandidateIsSelectedAmongMultipleRois)
{
  // Arrange
  const auto traffic_light_id = 123;
  const auto expected_roi = make_roi(100, 100, 40, 40);
  const auto rough_roi = make_roi(50, 50, 200, 200);
  // Both detected ROI centers (180, 180) and (120, 120) lie inside the rough ROI span (50..250).
  const auto low_iou_detected_roi = make_roi(140, 140, 80, 80);
  const auto high_iou_detected_roi = make_roi(100, 100, 40, 40);  // coincides with the expected ROI
  const auto expected_assigned_roi = high_iou_detected_roi;

  const auto detected_rois = make_detected_rois({low_iou_detected_roi, high_iou_detected_roi});
  const auto rough_rois =
    make_traffic_light_roi_array({make_traffic_light_roi(traffic_light_id, rough_roi)});
  const auto expected_rois =
    make_traffic_light_roi_array({make_traffic_light_roi(traffic_light_id, expected_roi)});
  const auto camera_info = make_camera_info(1280, 720);

  // Act
  publish_inputs(detected_rois, rough_rois, expected_rois, camera_info);
  const auto result = receive_published_message();

  // Assert
  ASSERT_NE(result, nullptr);
  ASSERT_EQ(result->rois.size(), 1u);
  EXPECT_EQ(result->rois.front().traffic_light_id, traffic_light_id);
  EXPECT_TRUE(is_same(result->rois.front().roi, expected_assigned_roi));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
