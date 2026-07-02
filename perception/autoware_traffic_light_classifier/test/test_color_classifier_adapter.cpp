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
// ROS adapter tests for ColorClassifier.
//
// Classification behavior is covered ROS-free in test_color_classifier (against
// ColorClassifierCore). This file pins the one adapter concern that cannot move
// to the core because it depends on the node: the images/signals size guard in
// getTrafficSignals. The guard protects the element-merge loop from an
// out-of-range access when the caller's signal count disagrees with the images,
// so a regression here is a memory-safety bug, not just a wrong return value.
//

#include "../src/classifier/color_classifier.hpp"

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_perception_msgs/msg/traffic_light_array.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace
{
namespace tl = autoware::traffic_light;

using tier4_perception_msgs::msg::TrafficLightArray;

// A mismatched images/signals count is rejected before any per-slot access.
TEST(ColorClassifierAdapterTest, MismatchedSizesReturnFalse)
{
  // Arrange
  auto node = std::make_shared<rclcpp::Node>("color_classifier_adapter_test");
  tl::ColorClassifier classifier(node.get());
  const std::vector<cv::Mat> images{cv::Mat(4, 4, CV_8UC3, cv::Scalar(0, 0, 0))};
  TrafficLightArray signals;
  signals.signals.resize(2);  // deliberately != images.size()

  // Act
  const bool ok = classifier.getTrafficSignals(images, signals);

  // Assert
  EXPECT_FALSE(ok);
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
