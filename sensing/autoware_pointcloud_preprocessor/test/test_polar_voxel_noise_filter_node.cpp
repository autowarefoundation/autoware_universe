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

#include "autoware/pointcloud_preprocessor/outlier_filter/polar_voxel_noise_filter_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

using autoware::pointcloud_preprocessor::PolarVoxelNoiseFilterComponent;

class PolarVoxelNoiseFilterComponentPublic : public PolarVoxelNoiseFilterComponent
{
public:
  using PolarVoxelNoiseFilterComponent::filter;
  using PolarVoxelNoiseFilterComponent::noise_cloud_pub_;
  explicit PolarVoxelNoiseFilterComponentPublic(const rclcpp::NodeOptions & options)
  : PolarVoxelNoiseFilterComponent(options)
  {
  }
};

struct SimplePoint
{
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint8_t return_type;
};

sensor_msgs::msg::PointCloud2 make_test_cloud(const std::vector<SimplePoint> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  cloud.fields.resize(5);

  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;

  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;

  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;

  cloud.fields[3].name = "intensity";
  cloud.fields[3].offset = 12;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[3].count = 1;

  cloud.fields[4].name = "return_type";
  cloud.fields[4].offset = 13;
  cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[4].count = 1;

  cloud.point_step = 14;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);

  for (size_t i = 0; i < points.size(); ++i) {
    const size_t offset = i * cloud.point_step;
    *reinterpret_cast<float *>(&cloud.data[offset + 0]) = points[i].x;
    *reinterpret_cast<float *>(&cloud.data[offset + 4]) = points[i].y;
    *reinterpret_cast<float *>(&cloud.data[offset + 8]) = points[i].z;
    cloud.data[offset + 12] = points[i].intensity;
    cloud.data[offset + 13] = points[i].return_type;
  }

  return cloud;
}

rclcpp::NodeOptions make_node_options()
{
  return rclcpp::NodeOptions()
    .append_parameter_override("publish_noise_cloud", false)
    .append_parameter_override("radial_resolution", 0.5)
    .append_parameter_override("azimuth_resolution", 0.08)
    .append_parameter_override("elevation_resolution", 0.08)
    .append_parameter_override("voxel_points_threshold", 4)
    .append_parameter_override("min_radius", 0.5)
    .append_parameter_override("max_radius", 300.0)
    .append_parameter_override("use_return_type_classification", false)
    .append_parameter_override("filter_secondary_returns", false)
    .append_parameter_override("secondary_noise_threshold", 2)
    .append_parameter_override("avg_intensity_threshold", 0.5)
    .append_parameter_override("primary_return_types", std::vector<int64_t>{1, 6, 8, 10});
}

TEST(PolarVoxelNoiseFilterTest, SimpleMode_RemovesLowIntensitySparseVoxel)
{
  auto options = make_node_options()
                   .append_parameter_override("use_return_type_classification", false)
                   .append_parameter_override("voxel_points_threshold", 2)
                   .append_parameter_override("avg_intensity_threshold", 5.0);
  PolarVoxelNoiseFilterComponentPublic node(options);

  // Voxel A (radius=1): 5 points, avg intensity=20 -> valid
  // Voxel B (radius=3): 1 point, avg intensity=0 -> noise
  const auto input = make_test_cloud(
    {{1.0f, 0.0f, 0.0f, 20, 1},
     {1.0f, 0.0f, 0.0f, 20, 2},
     {1.0f, 0.0f, 0.0f, 20, 1},
     {1.0f, 0.0f, 0.0f, 20, 1},
     {1.0f, 0.0f, 0.0f, 20, 1},
     {3.0f, 0.0f, 0.0f, 0, 2}});
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);

  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);

  EXPECT_EQ(output.width, 5u);  // expect only the 5 points from Voxel A to remain
}

TEST(PolarVoxelNoiseFilterTest, ReturnTypeMode_FilterSecondaryReturnsKeepsOnlyPrimary)
{
  auto options = make_node_options()
                   .append_parameter_override("use_return_type_classification", true)
                   .append_parameter_override("filter_secondary_returns", true)
                   .append_parameter_override("voxel_points_threshold", 1)
                   .append_parameter_override("secondary_noise_threshold", 5)
                   .append_parameter_override("avg_intensity_threshold", 100.0)
                   .append_parameter_override("primary_return_types", std::vector<int64_t>{1});
  PolarVoxelNoiseFilterComponentPublic node(options);

  // Single voxel with 1 primary + 3 secondary.
  // Voxel remains valid, but output should keep only primary when secondary filtering is enabled.
  const auto input = make_test_cloud(
    {{1.0f, 0.0f, 0.0f, 20, 1},
     {1.0f, 0.0f, 0.0f, 20, 2},
     {1.0f, 0.0f, 0.0f, 20, 2},
     {1.0f, 0.0f, 0.0f, 20, 2}});
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);

  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);

  EXPECT_EQ(output.width, 1u);     // expect only one point to remain
  EXPECT_EQ(output.data[13], 1u);  // checking that it was a primary point return_type
}

TEST(PolarVoxelNoiseFilterTest, PublishNoiseCloudTrue_CreatesPublisher)
{
  auto options = make_node_options().append_parameter_override("publish_noise_cloud", true);
  PolarVoxelNoiseFilterComponentPublic node(options);
  EXPECT_NE(node.noise_cloud_pub_, nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
