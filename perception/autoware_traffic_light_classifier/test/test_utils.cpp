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

#include "../src/traffic_light_classifier_process.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgcodecs.hpp>

#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#include <gtest/gtest.h>

#include <string>

bool readImage(const std::string & filename, cv::Mat & rgb_img)
{
  const auto package_dir =
    ament_index_cpp::get_package_share_directory("autoware_traffic_light_classifier");
  const auto path = package_dir + "/test_data/" + filename;
  const cv::Mat img = cv::imread(path);
  if (img.empty()) {
    return false;
  }
  cv::cvtColor(img, rgb_img, cv::COLOR_BGR2RGB);
  return true;
}

TEST(is_harsh_backlight, normal)
{
  cv::Mat rgb_img;
  EXPECT_TRUE(readImage("normal.png", rgb_img));
  const double backlight_threshold = 0.85;
  bool result = autoware::traffic_light::utils::is_harsh_backlight(rgb_img, backlight_threshold);
  EXPECT_FALSE(result);
}

TEST(is_harsh_backlight, backlight_weak)
{
  cv::Mat rgb_img;
  EXPECT_TRUE(readImage("backlight_weak.png", rgb_img));
  const double backlight_threshold = 0.85;
  bool result = autoware::traffic_light::utils::is_harsh_backlight(rgb_img, backlight_threshold);
  EXPECT_FALSE(result);
}

TEST(is_harsh_backlight, backlight_medium)
{
  cv::Mat rgb_img;
  EXPECT_TRUE(readImage("backlight_medium.png", rgb_img));
  const double backlight_threshold = 0.85;
  bool result = autoware::traffic_light::utils::is_harsh_backlight(rgb_img, backlight_threshold);
  EXPECT_FALSE(result);
}

TEST(is_harsh_backlight, backlight_strong)
{
  cv::Mat rgb_img;
  EXPECT_TRUE(readImage("backlight_strong.png", rgb_img));
  const double backlight_threshold = 0.85;
  bool result = autoware::traffic_light::utils::is_harsh_backlight(rgb_img, backlight_threshold);
  EXPECT_TRUE(result);
}

TEST(convertColorStringtoT4, normal)
{
  using tier4_perception_msgs::msg::TrafficLightElement;
  EXPECT_EQ(
    autoware::traffic_light::utils::convertColorStringtoT4("red"), TrafficLightElement::RED);
  EXPECT_EQ(
    autoware::traffic_light::utils::convertColorStringtoT4("yellow"), TrafficLightElement::AMBER);
  EXPECT_EQ(
    autoware::traffic_light::utils::convertColorStringtoT4("green"), TrafficLightElement::GREEN);
  EXPECT_EQ(
    autoware::traffic_light::utils::convertColorStringtoT4("white"), TrafficLightElement::WHITE);
  EXPECT_EQ(  // return UNKNOWN for unknown string
    autoware::traffic_light::utils::convertColorStringtoT4("abcde"), TrafficLightElement::UNKNOWN);
}

TEST(convertShapeStringtoT4, normal)
{
  using tier4_perception_msgs::msg::TrafficLightElement;
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeStringtoT4("circle"), TrafficLightElement::CIRCLE);
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeStringtoT4("left"),
    TrafficLightElement::LEFT_ARROW);
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeStringtoT4("right"),
    TrafficLightElement::RIGHT_ARROW);
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeStringtoT4("straight"),
    TrafficLightElement::UP_ARROW);
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeStringtoT4("up_left"),
    TrafficLightElement::UP_LEFT_ARROW);
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeStringtoT4("up_right"),
    TrafficLightElement::UP_RIGHT_ARROW);
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeStringtoT4("down"),
    TrafficLightElement::DOWN_ARROW);
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeStringtoT4("down_left"),
    TrafficLightElement::DOWN_LEFT_ARROW);
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeStringtoT4("down_right"),
    TrafficLightElement::DOWN_RIGHT_ARROW);
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeStringtoT4("cross"), TrafficLightElement::CROSS);
  EXPECT_EQ(  // return UNKNOWN for unknown string
    autoware::traffic_light::utils::convertShapeStringtoT4("abcde"), TrafficLightElement::UNKNOWN);
}

TEST(convertColorT4toString, normal)
{
  using tier4_perception_msgs::msg::TrafficLightElement;
  EXPECT_EQ(
    autoware::traffic_light::utils::convertColorT4toString(TrafficLightElement::RED), "red");
  EXPECT_EQ(
    autoware::traffic_light::utils::convertColorT4toString(TrafficLightElement::AMBER), "yellow");
  EXPECT_EQ(
    autoware::traffic_light::utils::convertColorT4toString(TrafficLightElement::GREEN), "green");
  EXPECT_EQ(
    autoware::traffic_light::utils::convertColorT4toString(TrafficLightElement::WHITE), "white");
  EXPECT_EQ(  // return "unknown" for unknown enum value
    autoware::traffic_light::utils::convertColorT4toString(99),
    "unknown");
}

TEST(convertShapeT4toString, normal)
{
  using tier4_perception_msgs::msg::TrafficLightElement;
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeT4toString(TrafficLightElement::CIRCLE), "circle");
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeT4toString(TrafficLightElement::LEFT_ARROW),
    "left");
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeT4toString(TrafficLightElement::RIGHT_ARROW),
    "right");
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeT4toString(TrafficLightElement::UP_ARROW),
    "straight");
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeT4toString(TrafficLightElement::UP_LEFT_ARROW),
    "up_left");
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeT4toString(TrafficLightElement::UP_RIGHT_ARROW),
    "up_right");
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeT4toString(TrafficLightElement::DOWN_ARROW),
    "down");
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeT4toString(TrafficLightElement::DOWN_LEFT_ARROW),
    "down_left");
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeT4toString(TrafficLightElement::DOWN_RIGHT_ARROW),
    "down_right");
  EXPECT_EQ(
    autoware::traffic_light::utils::convertShapeT4toString(TrafficLightElement::CROSS), "cross");
  EXPECT_EQ(  // return "unknown" for unknown enum value
    autoware::traffic_light::utils::convertShapeT4toString(99),
    "unknown");
}

TEST(isColorLabel, normal)
{
  EXPECT_TRUE(autoware::traffic_light::utils::isColorLabel("red"));
  EXPECT_TRUE(autoware::traffic_light::utils::isColorLabel("yellow"));
  EXPECT_TRUE(autoware::traffic_light::utils::isColorLabel("green"));
  EXPECT_TRUE(autoware::traffic_light::utils::isColorLabel("white"));
  EXPECT_FALSE(autoware::traffic_light::utils::isColorLabel("circle"));
  EXPECT_FALSE(autoware::traffic_light::utils::isColorLabel("right"));
  EXPECT_FALSE(autoware::traffic_light::utils::isColorLabel("abcde"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
