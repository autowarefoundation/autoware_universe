// Copyright 2026 TIER IV
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

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_diag.hpp"

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

namespace autoware::pointcloud_preprocessor
{

TEST(PointCloudValidationTest, MissingChannelFieldTest)
{
  sensor_msgs::msg::PointCloud2 cloud_without_channel;
  sensor_msgs::PointCloud2Modifier modifier(cloud_without_channel);
  modifier.setPointCloud2Fields(
    2, "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32, "distance", 1,
    sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_THROW({ validate_pointcloud_fields(cloud_without_channel); }, std::runtime_error);
}

TEST(PointCloudValidationTest, MissingAzimuthFieldTest)
{
  sensor_msgs::msg::PointCloud2 cloud_without_azimuth;
  sensor_msgs::PointCloud2Modifier modifier(cloud_without_azimuth);
  modifier.setPointCloud2Fields(
    2, "channel", 1, sensor_msgs::msg::PointField::UINT16, "distance", 1,
    sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_THROW({ validate_pointcloud_fields(cloud_without_azimuth); }, std::runtime_error);
}

TEST(PointCloudValidationTest, MissingDistanceFieldTest)
{
  sensor_msgs::msg::PointCloud2 cloud_without_distance;
  sensor_msgs::PointCloud2Modifier modifier(cloud_without_distance);
  modifier.setPointCloud2Fields(
    2, "channel", 1, sensor_msgs::msg::PointField::UINT16, "azimuth", 1,
    sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_THROW({ validate_pointcloud_fields(cloud_without_distance); }, std::runtime_error);
}

TEST(PointCloudValidationTest, ValidFieldsTest)
{
  sensor_msgs::msg::PointCloud2 cloud_with_all_fields;
  sensor_msgs::PointCloud2Modifier modifier(cloud_with_all_fields);
  modifier.setPointCloud2Fields(
    3, "channel", 1, sensor_msgs::msg::PointField::UINT16, "azimuth", 1,
    sensor_msgs::msg::PointField::FLOAT32, "distance", 1, sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_NO_THROW({ validate_pointcloud_fields(cloud_with_all_fields); });
}

bool is_same_image(const cv::Mat & img1, const cv::Mat & img2)
{
  if (img1.size() != img2.size() || img1.type() != img2.type()) {
    return false;
  }
  cv::Mat diff;
  cv::compare(img1, img2, diff, cv::CMP_NE);
  return cv::countNonZero(diff) == 0;
}

TEST(MultiFrameDetectionVisualizerTest, ZeroBufferingIntervalReturnSameMask)
{
  // Setup visualizer with zero buffering interval
  MultiFrameDetectionVisualizeConfig config;
  config.buffering_frames = 4;
  config.buffering_interval = 0;
  MultiFrameDetectionVisualizer visualizer(config);
  cv::Mat input_mask(10, 10, CV_8UC1, cv::Scalar(255));

  // Update visualizer and get result
  cv::Mat result = visualizer.update(input_mask);

  // Verify that the result matches the input mask
  EXPECT_TRUE(is_same_image(input_mask, result));
}

TEST(MultiFrameDetectionVisualizerTest, AllPixelsConsistentTest)
{
  MultiFrameDetectionVisualizeConfig config;
  config.buffering_frames = 4;
  config.buffering_interval = 1;
  MultiFrameDetectionVisualizer visualizer(config);
  cv::Mat consistent_mask(10, 10, CV_8UC1, cv::Scalar(255));
  int total_pixels = consistent_mask.rows * consistent_mask.cols;

  // Update with same mask 4 times
  visualizer.update(consistent_mask);
  visualizer.update(consistent_mask);
  visualizer.update(consistent_mask);
  cv::Mat result = visualizer.update(consistent_mask);

  // All pixels should be detected
  EXPECT_EQ(cv::countNonZero(result), total_pixels);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace autoware::pointcloud_preprocessor
