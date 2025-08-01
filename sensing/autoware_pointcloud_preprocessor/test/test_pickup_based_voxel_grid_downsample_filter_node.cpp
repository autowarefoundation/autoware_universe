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

#include "autoware/pointcloud_preprocessor/downsample_filter/pickup_based_voxel_grid_downsample_filter_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

// Helper function to create a test point cloud
sensor_msgs::msg::PointCloud2 createTestPointCloud(
  const std::vector<std::array<float, 3>> & points, const std::string & frame_id = "base_link")
{
  auto cloud = sensor_msgs::msg::PointCloud2();
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = rclcpp::Clock().now();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  // Create point cloud with x, y, z fields
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);

  // Add test points
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const auto & point : points) {
    *iter_x = point[0];
    *iter_y = point[1];
    *iter_z = point[2];
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return cloud;
}

// Helper function to extract points from PointCloud2
std::vector<std::array<float, 3>> extractPoints(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<std::array<float, 3>> points;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    points.push_back({*iter_x, *iter_y, *iter_z});
  }

  return points;
}

TEST(DownsampleWithVoxelGridTest, TestBasicDownsampling)
{
  // Create test points that should be downsampled
  // Points within the same voxel should be reduced to one point
  std::vector<std::array<float, 3>> input_points = {
    {0.0f, 0.0f, 0.0f},     // Voxel (0,0,0)
    {0.05f, 0.05f, 0.05f},  // Same voxel as above (within 0.1 voxel size)
    {0.2f, 0.2f, 0.2f},     // Different voxel
    {0.25f, 0.25f, 0.25f},  // Same voxel as above
    {0.5f, 0.5f, 0.5f},     // Different voxel
  };

  auto input_cloud = createTestPointCloud(input_points);

  autoware::pointcloud_preprocessor::VoxelSize voxel_size = {0.1f, 0.1f, 0.1f};
  sensor_msgs::msg::PointCloud2 output_cloud;

  // Call the function under test
  autoware::pointcloud_preprocessor::downsample_with_voxel_grid(
    input_cloud, voxel_size, output_cloud);

  // Extract output points
  auto output_points = extractPoints(output_cloud);

  // Should have 3 points (3 different voxels)
  EXPECT_EQ(output_points.size(), 3);

  // Check that output cloud has correct metadata
  EXPECT_EQ(output_cloud.header.frame_id, input_cloud.header.frame_id);
  EXPECT_EQ(output_cloud.height, 1);
  EXPECT_EQ(output_cloud.width, 3);
  EXPECT_EQ(output_cloud.fields.size(), input_cloud.fields.size());
}

TEST(DownsampleWithVoxelGridTest, TestEmptyPointCloud)
{
  // Create empty point cloud
  std::vector<std::array<float, 3>> input_points = {};
  auto input_cloud = createTestPointCloud(input_points);

  autoware::pointcloud_preprocessor::VoxelSize voxel_size = {0.1f, 0.1f, 0.1f};
  sensor_msgs::msg::PointCloud2 output_cloud;

  // Call the function under test
  autoware::pointcloud_preprocessor::downsample_with_voxel_grid(
    input_cloud, voxel_size, output_cloud);

  // Should have 0 points
  EXPECT_EQ(output_cloud.width, 0);
  EXPECT_EQ(output_cloud.data.size(), 0);
}

TEST(DownsampleWithVoxelGridTest, TestSinglePoint)
{
  // Create point cloud with single point
  std::vector<std::array<float, 3>> input_points = {{1.0f, 2.0f, 3.0f}};
  auto input_cloud = createTestPointCloud(input_points);

  autoware::pointcloud_preprocessor::VoxelSize voxel_size = {0.1f, 0.1f, 0.1f};
  sensor_msgs::msg::PointCloud2 output_cloud;

  // Call the function under test
  autoware::pointcloud_preprocessor::downsample_with_voxel_grid(
    input_cloud, voxel_size, output_cloud);

  // Extract output points
  auto output_points = extractPoints(output_cloud);

  // Should have 1 point
  EXPECT_EQ(output_points.size(), 1);

  // Check that the point is preserved
  EXPECT_FLOAT_EQ(output_points[0][0], 1.0f);
  EXPECT_FLOAT_EQ(output_points[0][1], 2.0f);
  EXPECT_FLOAT_EQ(output_points[0][2], 3.0f);
}

TEST(DownsampleWithVoxelGridTest, TestDifferentVoxelSizes)
{
  // Create test points
  std::vector<std::array<float, 3>> input_points = {
    {0.0f, 0.0f, 0.0f},
    {0.1f, 0.1f, 0.1f},
    {0.2f, 0.2f, 0.2f},
    {0.3f, 0.3f, 0.3f},
  };
  auto input_cloud = createTestPointCloud(input_points);

  // Test with small voxel size - should preserve all points
  {
    autoware::pointcloud_preprocessor::VoxelSize small_voxel = {0.05f, 0.05f, 0.05f};
    sensor_msgs::msg::PointCloud2 output_cloud;
    autoware::pointcloud_preprocessor::downsample_with_voxel_grid(
      input_cloud, small_voxel, output_cloud);

    auto output_points = extractPoints(output_cloud);
    EXPECT_EQ(output_points.size(), 4);  // All points in different voxels
  }

  // Test with large voxel size - should reduce to one point
  {
    autoware::pointcloud_preprocessor::VoxelSize large_voxel = {1.0f, 1.0f, 1.0f};
    sensor_msgs::msg::PointCloud2 output_cloud;
    autoware::pointcloud_preprocessor::downsample_with_voxel_grid(
      input_cloud, large_voxel, output_cloud);

    auto output_points = extractPoints(output_cloud);
    EXPECT_EQ(output_points.size(), 1);  // All points in same voxel
  }
}

TEST(DownsampleWithVoxelGridTest, TestNegativeCoordinates)
{
  // Create test points with negative coordinates
  std::vector<std::array<float, 3>> input_points = {
    {-0.5f, -0.5f, -0.5f},
    {-0.45f, -0.45f, -0.45f},  // Same voxel as above
    {0.5f, 0.5f, 0.5f},
    {0.55f, 0.55f, 0.55f},  // Same voxel as above
  };
  auto input_cloud = createTestPointCloud(input_points);

  autoware::pointcloud_preprocessor::VoxelSize voxel_size = {0.1f, 0.1f, 0.1f};
  sensor_msgs::msg::PointCloud2 output_cloud;

  // Call the function under test
  autoware::pointcloud_preprocessor::downsample_with_voxel_grid(
    input_cloud, voxel_size, output_cloud);

  // Extract output points
  auto output_points = extractPoints(output_cloud);

  // Should have 2 points (2 different voxels)
  EXPECT_EQ(output_points.size(), 2);
}
