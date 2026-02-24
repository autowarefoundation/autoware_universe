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
  sensor_msgs::msg::PointCloud2 create_pointcloud(
    const std::vector<uint16_t> & channels, const std::vector<float> & azimuths_deg,
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

ConverterConfig default_config()
{
  ConverterConfig config;
  config.horizontal.angle_range_min_deg = 0.0;
  config.horizontal.angle_range_max_deg = 360.0;
  config.horizontal.horizontal_resolution = 90.0;  // 4 bins
  config.vertical.vertical_bins = 2;
  config.vertical.is_channel_order_top2down = true;
  config.max_distance_range = 100.0;
  return config;
}

TEST_F(PointCloud2ToDepthImageTest, EmptyPointCloud)
{
  // Create an empty pointcloud
  sensor_msgs::msg::PointCloud2 empty_cloud = create_pointcloud({}, {}, {});

  // Conversion
  PointCloud2ToDepthImage converter(default_config());
  cv::Mat depth_image = converter.make_normalized_depth_image(empty_cloud);

  // Verify dimensions are correct but all zeros
  EXPECT_EQ(depth_image.rows, 2);
  EXPECT_EQ(depth_image.cols, 4);
  for (int i = 0; i < depth_image.rows; ++i) {
    for (int j = 0; j < depth_image.cols; ++j) {
      EXPECT_EQ(depth_image.at<uint16_t>(i, j), 0);
    }
  }
}

TEST_F(PointCloud2ToDepthImageTest, BasicDepthImageCreation)
{
  // Create a simple pointcloud with one point
  float first_index_angle = 0.1;
  float mid_distance = 50.0;
  sensor_msgs::msg::PointCloud2 cloud = create_pointcloud({0}, {first_index_angle}, {mid_distance});
  // Expected normalized depth calculation
  uint16_t expected_normalized_depth = UINT16_MAX / 2;

  // Conversion
  PointCloud2ToDepthImage converter(default_config());
  cv::Mat depth_image = converter.make_normalized_depth_image(cloud);

  // Verify the depth value at the expected position
  EXPECT_EQ(depth_image.at<uint16_t>(0, 0), expected_normalized_depth);
}

TEST_F(PointCloud2ToDepthImageTest, MultiplePointsInDifferentBins)
{
  // Create pointcloud with multiple points in different bins
  std::vector<uint16_t> channels = {0, 1, 0, 1};
  std::vector<float> azimuths_deg = {1.0, 1.0, 135.0, 135.0};
  std::vector<float> distances = {25.0, 50.0, 75.0, 90.0};
  auto cloud = create_pointcloud(channels, azimuths_deg, distances);

  // Conversion
  PointCloud2ToDepthImage converter(default_config());
  cv::Mat depth_image = converter.make_normalized_depth_image(cloud);

  // Verify value at each expected bin
  EXPECT_GT(depth_image.at<uint16_t>(0, 0), 0);  // Channel 0, 1 deg
  EXPECT_GT(depth_image.at<uint16_t>(1, 0), 0);  // Channel 1, 1 deg
  EXPECT_GT(depth_image.at<uint16_t>(0, 1), 0);  // Channel 0, 135 deg
  EXPECT_GT(depth_image.at<uint16_t>(1, 1), 0);  // Channel 1, 135 deg
}

TEST_F(PointCloud2ToDepthImageTest, ChannelOrderBottom2Top)
{
  // Create config with bottom2top channel order
  ConverterConfig config = default_config();
  config.vertical.vertical_bins = 3;
  config.vertical.is_channel_order_top2down = false;  // bottom to top
  // Create pointcloud with one point in the lowest channel
  std::vector<uint16_t> channels = {2};
  std::vector<float> azimuths_deg = {1.0};
  std::vector<float> distances = {50.0};
  sensor_msgs::msg::PointCloud2 cloud = create_pointcloud(channels, azimuths_deg, distances);
  // Expected normalized depth calculation
  uint16_t expected_normalized_depth = UINT16_MAX / 2;

  // Conversion
  PointCloud2ToDepthImage converter(config);
  cv::Mat depth_image = converter.make_normalized_depth_image(cloud);

  // Verify that the point is placed in the correct vertical bin (bottom row)
  EXPECT_EQ(depth_image.at<uint16_t>(0, 0), expected_normalized_depth);  // Bottom row, first column
}

TEST_F(PointCloud2ToDepthImageTest, FovWrapsAround)
{
  // Create config where FOV wraps around 360 degrees (e.g., 270-90 degrees)
  ConverterConfig config = default_config();
  config.horizontal.angle_range_min_deg = 270.0;  // min > max
  config.horizontal.angle_range_max_deg = 90.0;
  config.horizontal.horizontal_resolution = 90.0;  // 2 bins: [270-360) and [0-90]
  // Create pointcloud with points in the wrapped FOV range
  std::vector<uint16_t> channels = {0};
  std::vector<float> azimuths_deg = {45.0};
  std::vector<float> distances = {50.0};
  sensor_msgs::msg::PointCloud2 cloud = create_pointcloud(channels, azimuths_deg, distances);
  // Expected normalized depth calculation
  uint16_t expected_normalized_depth = UINT16_MAX / 2;
  uint16_t expected_horizontal_bin = 1;

  // Conversion
  PointCloud2ToDepthImage converter(config);
  cv::Mat depth_image = converter.make_normalized_depth_image(cloud);

  // Verify that the point is placed in the correct horizontal bin
  EXPECT_EQ(depth_image.at<uint16_t>(0, expected_horizontal_bin), expected_normalized_depth);
}

}  // namespace autoware::pointcloud_preprocessor::pointcloud2_to_depth_image

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
