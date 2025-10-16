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

#include "../src/traffic_light_multi_camera_fusion_process.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <unordered_map>

TEST(isUnknown, normal)
{
  tier4_perception_msgs::msg::TrafficLight signal;
  tier4_perception_msgs::msg::TrafficLightElement element;
  {
    element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    signal.elements.push_back(element);
  }
  EXPECT_TRUE(autoware::traffic_light::utils::isUnknown(signal));

  {
    signal.elements.clear();
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    signal.elements.push_back(element);
  }
  EXPECT_FALSE(autoware::traffic_light::utils::isUnknown(signal));
}

TEST(at_or, normal)
{
  std::unordered_map<int, int> map;
  map[1] = 2;
  map[3] = 4;
  EXPECT_EQ(autoware::traffic_light::utils::at_or(map, 1, -1), 2);
  EXPECT_EQ(autoware::traffic_light::utils::at_or(map, 2, -1), -1);
  EXPECT_EQ(autoware::traffic_light::utils::at_or(map, 3, -1), 4);
}

namespace same_camera
{

// first condition
TEST(compareRecord, timestamp_check)
{
  // r1 is newer
  autoware::traffic_light::utils::FusionRecord r1;
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(200, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  autoware::traffic_light::utils::FusionRecord r2;
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.8;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), 1);

  // r2 is newer
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(200, 10);
    header.frame_id = "camera0";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.8;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), -1);
}

// second condition
TEST(compareRecord, unknown_check)
{
  // r1 is unknown
  autoware::traffic_light::utils::FusionRecord r1;
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.confidence = 0.0;
    r1.signal.elements.push_back(element);
  }
  autoware::traffic_light::utils::FusionRecord r2;
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.8;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), -1);

  // r2 is unknown
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.confidence = 0.0;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), 1);

  // both of r1 and r2 is unknown
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.confidence = 0.0;
    r1.signal.elements.push_back(element);
  }
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.confidence = 0.0;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), 0);
}

// third condition
TEST(compareRecord, visible_check)
{
  // r1 is better visible on top left
  autoware::traffic_light::utils::FusionRecord r1;
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  autoware::traffic_light::utils::FusionRecord r2;
  {
    const uint32_t boundary_thres = 5;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = boundary_thres;
    r2.roi.roi.y_offset = boundary_thres;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.8;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), 1);

  // r1 is better visible on bottom left
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  {
    const uint32_t boundary_thres = 5;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 1440 - (100 + boundary_thres);
    r2.roi.roi.y_offset = 1080 - (100 + boundary_thres);
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.8;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), 1);

  // r2 is better visible on top left
  {
    const uint32_t boundary_thres = 5;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = boundary_thres;
    r1.roi.roi.y_offset = boundary_thres;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.8;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), -1);

  // r2 is better visible on bottom left
  {
    const uint32_t boundary_thres = 5;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 1440 - (100 + boundary_thres);
    ;
    r1.roi.roi.y_offset = 1080 - (100 + boundary_thres);
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.8;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), -1);
}

// fourth condition
TEST(compareRecord, confidence_check)
{
  // r1 is higher confidence
  autoware::traffic_light::utils::FusionRecord r1;
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  autoware::traffic_light::utils::FusionRecord r2;
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.8;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), 1);

  // r2 is higher confidence
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.95;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), -1);
}

}  // namespace same_camera

namespace different_camera
{

// first condition is nothing in different camera

// second condition
TEST(compareRecord, unknown_check)
{
  // r1 is unknown
  autoware::traffic_light::utils::FusionRecord r1;
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.confidence = 0.0;
    r1.signal.elements.push_back(element);
  }
  autoware::traffic_light::utils::FusionRecord r2;
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera1";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.8;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), -1);

  // r2 is unknown
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera1";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.confidence = 0.0;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), 1);

  // both of r1 and r2 is unknown
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.confidence = 0.0;
    r1.signal.elements.push_back(element);
  }
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera1";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.confidence = 0.0;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), 0);
}

// third condition
TEST(compareRecord, visible_check)
{
  // r1 is better visible on top left
  autoware::traffic_light::utils::FusionRecord r1;
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  autoware::traffic_light::utils::FusionRecord r2;
  {
    const uint32_t boundary_thres = 5;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera1";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = boundary_thres;
    r2.roi.roi.y_offset = boundary_thres;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.8;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), 1);

  // r1 is better visible on bottom left
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  {
    const uint32_t boundary_thres = 5;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera1";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 1440 - (100 + boundary_thres);
    r2.roi.roi.y_offset = 1080 - (100 + boundary_thres);
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.8;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), 1);

  // r2 is better visible on top left
  {
    const uint32_t boundary_thres = 5;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = boundary_thres;
    r1.roi.roi.y_offset = boundary_thres;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera1";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.8;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), -1);

  // r2 is better visible on bottom left
  {
    const uint32_t boundary_thres = 5;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 1440 - (100 + boundary_thres);
    ;
    r1.roi.roi.y_offset = 1080 - (100 + boundary_thres);
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera1";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.8;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), -1);
}

// fourth condition
TEST(compareRecord, confidence_check)
{
  // r1 is higher confidence
  autoware::traffic_light::utils::FusionRecord r1;
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  autoware::traffic_light::utils::FusionRecord r2;
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera1";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.8;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), 1);

  // r2 is higher confidence
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera1";
    // header
    r2.header = header;
    // cam_info
    r2.cam_info.header = header;
    r2.cam_info.width = 1440;
    r2.cam_info.height = 1080;
    // roi
    r2.roi.roi.x_offset = 100;
    r2.roi.roi.y_offset = 100;
    r2.roi.roi.width = 100;
    r2.roi.roi.height = 100;
    // signal
    r2.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.95;
    r2.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::compareRecord(r1, r2), -1);
}

}  // namespace different_camera

TEST(calVisibleScore, normal)
{
  // visible
  autoware::traffic_light::utils::FusionRecord r1;
  {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 100;
    r1.roi.roi.y_offset = 100;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::calVisibleScore(r1), 1);

  // invisible by left top
  {
    const uint32_t boundary_thres = 5;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = boundary_thres;
    r1.roi.roi.y_offset = boundary_thres;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::calVisibleScore(r1), 0);

  // invisible by right top
  {
    const uint32_t boundary_thres = 5;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 1080 - (100 + boundary_thres);
    r1.roi.roi.y_offset = boundary_thres;
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::calVisibleScore(r1), 0);

  // invisible by left bottom
  {
    const uint32_t boundary_thres = 5;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = boundary_thres;
    r1.roi.roi.y_offset = 1080 - (100 + boundary_thres);
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::calVisibleScore(r1), 0);

  // invisible by right bottom
  {
    const uint32_t boundary_thres = 5;
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(100, 10);
    header.frame_id = "camera0";
    // header
    r1.header = header;
    // cam_info
    r1.cam_info.header = header;
    r1.cam_info.width = 1440;
    r1.cam_info.height = 1080;
    // roi
    r1.roi.roi.x_offset = 1440 - (100 + boundary_thres);
    r1.roi.roi.y_offset = 1080 - (100 + boundary_thres);
    r1.roi.roi.width = 100;
    r1.roi.roi.height = 100;
    // signal
    r1.signal.elements.clear();
    tier4_perception_msgs::msg::TrafficLightElement element;
    element.color = tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
    element.confidence = 0.9;
    r1.signal.elements.push_back(element);
  }
  EXPECT_EQ(autoware::traffic_light::utils::calVisibleScore(r1), 0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
