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

#include "../src/traffic_light_classifier_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgcodecs.hpp>

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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
