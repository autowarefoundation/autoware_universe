// Copyright 2024 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/blockage_diag/pointcloud2_to_depth_image.hpp"

#include <opencv2/core/mat.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::pointcloud_preprocessor::pointcloud2_to_depth_image
{

class PointCloud2ToDepthImageTest : public ::testing::Test
{
protected:
  // Helper function to create a pointcloud with specified points
  sensor_msgs::msg::PointCloud2 create_pointcloud(
    const std::vector<uint16_t> & channels,
    const std::vector<float> & azimuths_deg,
    const std::vector<float> & distances)
  {
    EXPECT_EQ(channels.size(), azimuths_deg.size());
    EXPECT_EQ(channels.size(), distances.size());

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.height = 1;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      3, "channel", 1, sensor_msgs::msg::PointField::UINT16, "azimuth", 1,
      sensor_msgs::msg::PointField::FLOAT32, "distance", 1, sensor_msgs::msg::PointField::FLOAT32);

    modifier.resize(channels.size());

    sensor_msgs::PointCloud2Iterator<uint16_t> iter_channel(cloud, "channel");
    sensor_msgs::PointCloud2Iterator<float> iter_azimuth(cloud, "azimuth");
    sensor_msgs::PointCloud2Iterator<float> iter_distance(cloud, "distance");

    for (size_t i = 0; i < channels.size(); ++i) {
      *iter_channel = channels[i];
      *iter_azimuth = azimuths_deg[i] * M_PI / 180.0;  // Convert to radians
      *iter_distance = distances[i];

      ++iter_channel;
      ++iter_azimuth;
      ++iter_distance;
    }

    return cloud;
  }
};

// Test empty pointcloud
TEST_F(PointCloud2ToDepthImageTest, EmptyPointCloud)
{
  ConverterConfig config;
  config.horizontal.angle_range_min_deg = -180.0;
  config.horizontal.angle_range_max_deg = 180.0;
  config.horizontal.horizontal_resolution = 60.0;
  config.vertical.vertical_bins = 4;
  config.vertical.is_channel_order_top2down = true;
  config.max_distance_range = 200.0;

  PointCloud2ToDepthImage converter(config);

  auto cloud = create_pointcloud({}, {}, {});
  cv::Mat depth_image = converter.make_normalized_depth_image(cloud);

  // Verify dimensions are correct but all zeros
  EXPECT_EQ(depth_image.rows, 4);
  EXPECT_EQ(depth_image.cols, 6);
  
  // All pixels should be zero
  for (int i = 0; i < depth_image.rows; ++i) {
    for (int j = 0; j < depth_image.cols; ++j) {
      EXPECT_EQ(depth_image.at<uint16_t>(i, j), 0);
    }
  }
}

// Test basic depth image creation with simple configuration
TEST_F(PointCloud2ToDepthImageTest, BasicDepthImageCreation)
{
  ConverterConfig config;
  config.horizontal.angle_range_min_deg = -180.0;
  config.horizontal.angle_range_max_deg = 180.0;
  config.horizontal.horizontal_resolution = 60.0;  // 6 bins
  config.vertical.vertical_bins = 4;
  config.vertical.is_channel_order_top2down = true;
  config.max_distance_range = 200.0;

  PointCloud2ToDepthImage converter(config);

  // Create a simple pointcloud with one point
  std::vector<uint16_t> channels = {0};
  std::vector<float> azimuths_deg = {0.0};
  std::vector<float> distances = {100.0};

  auto cloud = create_pointcloud(channels, azimuths_deg, distances);
  cv::Mat depth_image = converter.make_normalized_depth_image(cloud);

  // Verify dimensions
  EXPECT_EQ(depth_image.rows, 4);  // vertical_bins
  EXPECT_EQ(depth_image.cols, 6);  // 360 / 60 = 6 bins
  EXPECT_EQ(depth_image.type(), CV_16UC1);

  // Verify the depth value at the expected position
  // horizontal_bin for 0 deg = (0 - (-180)) / 60 = 3
  // vertical_bin = 0 (channel 0, top2down)
  uint16_t expected_depth = UINT16_MAX * (1.0 - 100.0 / 200.0);  // normalized
  EXPECT_EQ(depth_image.at<uint16_t>(0, 3), expected_depth);
}

// Test multiple points in different bins
TEST_F(PointCloud2ToDepthImageTest, MultiplePointsInDifferentBins)
{
  ConverterConfig config;
  config.horizontal.angle_range_min_deg = 0.0;
  config.horizontal.angle_range_max_deg = 360.0;
  config.horizontal.horizontal_resolution = 90.0;  // 4 bins
  config.vertical.vertical_bins = 2;
  config.vertical.is_channel_order_top2down = true;
  config.max_distance_range = 100.0;

  PointCloud2ToDepthImage converter(config);

  // Create pointcloud with multiple points in different bins
  // Note: Boundary condition is (azimuth > min_deg) && (azimuth <= max_deg)
  // Also note: distance at max_distance_range results in normalized_depth = 0
  std::vector<uint16_t> channels = {0, 1, 0, 1};
  std::vector<float> azimuths_deg = {1.0, 1.0, 135.0, 135.0};
  std::vector<float> distances = {25.0, 50.0, 75.0, 90.0};  // Use 90 instead of 100 to avoid 0 depth

  auto cloud = create_pointcloud(channels, azimuths_deg, distances);
  cv::Mat depth_image = converter.make_normalized_depth_image(cloud);

  // Verify dimensions
  EXPECT_EQ(depth_image.rows, 2);
  EXPECT_EQ(depth_image.cols, 4);

  // horizontal_bin for 1 deg = (1 - 0) / 90 = 0.01 -> 0
  // horizontal_bin for 135 deg = (135 - 0) / 90 = 1.5 -> 1
  int non_zero_count = 0;
  for (int i = 0; i < depth_image.rows; ++i) {
    for (int j = 0; j < depth_image.cols; ++j) {
      if (depth_image.at<uint16_t>(i, j) > 0) {
        non_zero_count++;
      }
    }
  }
  
  EXPECT_EQ(non_zero_count, 4);
  
  // Verify at least one value at each expected bin
  EXPECT_GT(depth_image.at<uint16_t>(0, 0), 0);  // Channel 0, 1 deg
  EXPECT_GT(depth_image.at<uint16_t>(1, 0), 0);  // Channel 1, 1 deg
  EXPECT_GT(depth_image.at<uint16_t>(0, 1), 0);  // Channel 0, 135 deg
  EXPECT_GT(depth_image.at<uint16_t>(1, 1), 0);  // Channel 1, 135 deg
}

// Test channel order bottom2top
TEST_F(PointCloud2ToDepthImageTest, ChannelOrderBottom2Top)
{
  ConverterConfig config;
  config.horizontal.angle_range_min_deg = 0.0;
  config.horizontal.angle_range_max_deg = 360.0;
  config.horizontal.horizontal_resolution = 90.0;  // 4 bins
  config.vertical.vertical_bins = 3;
  config.vertical.is_channel_order_top2down = false;  // bottom to top
  config.max_distance_range = 100.0;

  PointCloud2ToDepthImage converter(config);

  // Create pointcloud
  std::vector<uint16_t> channels = {0, 1, 2};
  std::vector<float> azimuths_deg = {45.0, 45.0, 45.0};
  std::vector<float> distances = {10.0, 20.0, 30.0};

  auto cloud = create_pointcloud(channels, azimuths_deg, distances);
  cv::Mat depth_image = converter.make_normalized_depth_image(cloud);

  // With bottom2top: vertical_bin = vertical_bins - channel - 1
  // channel 0 -> bin 2, channel 1 -> bin 1, channel 2 -> bin 0
  // horizontal_bin for 45 deg = (45 - 0) / 90 = 0
  EXPECT_EQ(depth_image.at<uint16_t>(2, 0), static_cast<uint16_t>(UINT16_MAX * 0.9));
  EXPECT_EQ(depth_image.at<uint16_t>(1, 0), static_cast<uint16_t>(UINT16_MAX * 0.8));
  EXPECT_EQ(depth_image.at<uint16_t>(0, 0), static_cast<uint16_t>(UINT16_MAX * 0.7));
}

}  // namespace autoware::pointcloud_preprocessor::pointcloud2_to_depth_image

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
