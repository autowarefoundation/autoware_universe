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

#include "autoware/pointcloud_preprocessor/outlier_filter/polar_voxel_outlier_filter_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

using autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent;

// Subclass to expose protected filter() for testing
class PolarVoxelOutlierFilterComponentPublic : public PolarVoxelOutlierFilterComponent
{
public:
  using PolarVoxelOutlierFilterComponent::filter;
  using PolarVoxelOutlierFilterComponent::noise_cloud_pub_;
  using PolarVoxelOutlierFilterComponent::visibility_;
  explicit PolarVoxelOutlierFilterComponentPublic(const rclcpp::NodeOptions & options)
  : PolarVoxelOutlierFilterComponent(options)
  {
  }
};

// Helper: Make a simple cartesian cloud with 3 points, each with return_type and intensity
sensor_msgs::msg::PointCloud2 make_cartesian_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.height = 1;
  cloud.width = 3;
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

  cloud.fields[3].name = "return_type";
  cloud.fields[3].offset = 12;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[3].count = 1;

  cloud.fields[4].name = "intensity";
  cloud.fields[4].offset = 13;
  cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[4].count = 1;

  cloud.point_step = 14;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);

  float * data = reinterpret_cast<float *>(cloud.data.data());
  data[0] = 1.0f;
  data[1] = 0.0f;
  data[2] = 0.0f;
  cloud.data[12] = 1;   // return_type (primary)
  cloud.data[13] = 15;  // intensity

  data = reinterpret_cast<float *>(&cloud.data[14]);
  data[0] = 2.0f;
  data[1] = 0.0f;
  data[2] = 0.0f;
  cloud.data[26] = 1;   // return_type (primary)
  cloud.data[27] = 15;  // intensity

  data = reinterpret_cast<float *>(&cloud.data[28]);
  data[0] = 2.1f;
  data[1] = 0.0f;
  data[2] = 0.0f;
  cloud.data[40] = 2;  // return_type (secondary)
  cloud.data[41] = 5;  // intensity

  return cloud;
}

// Helper: Make a cloud with polar fields, return_type, and intensity
sensor_msgs::msg::PointCloud2 make_polar_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.height = 1;
  cloud.width = 2;
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  cloud.fields.resize(8);

  cloud.fields[0].name = "distance";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;

  cloud.fields[1].name = "azimuth";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;

  cloud.fields[2].name = "elevation";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;

  cloud.fields[3].name = "x";
  cloud.fields[3].offset = 12;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[3].count = 1;

  cloud.fields[4].name = "y";
  cloud.fields[4].offset = 16;
  cloud.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[4].count = 1;

  cloud.fields[5].name = "z";
  cloud.fields[5].offset = 20;
  cloud.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[5].count = 1;

  cloud.fields[6].name = "return_type";
  cloud.fields[6].offset = 24;
  cloud.fields[6].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[6].count = 1;

  cloud.fields[7].name = "intensity";
  cloud.fields[7].offset = 25;
  cloud.fields[7].datatype = sensor_msgs::msg::PointField::UINT8;
  cloud.fields[7].count = 1;

  cloud.point_step = 26;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);

  float * data = reinterpret_cast<float *>(cloud.data.data());
  data[0] = 5.0f;
  data[1] = 0.0f;
  data[2] = 0.0f;
  data[3] = 5.0f;
  data[4] = 0.0f;
  data[5] = 0.0f;
  cloud.data[24] = 1;   // return_type (primary)
  cloud.data[25] = 10;  // intensity

  data = reinterpret_cast<float *>(&cloud.data[26]);
  data[0] = 2.0f;
  data[1] = 1.0f;
  data[2] = 0.5f;
  data[3] = 2.0f * std::cos(0.5f) * std::cos(1.0f);
  data[4] = 2.0f * std::cos(0.5f) * std::sin(1.0f);
  data[5] = 2.0f * std::sin(0.5f);
  cloud.data[50] = 2;  // return_type (secondary)
  cloud.data[51] = 2;  // intensity

  return cloud;
}

// Helper: NodeOptions with all required parameters set
rclcpp::NodeOptions make_node_options()
{
  return rclcpp::NodeOptions()
    .append_parameter_override("radial_resolution_m", 1.0)
    .append_parameter_override("azimuth_resolution_rad", 1.0)
    .append_parameter_override("elevation_resolution_rad", 1.0)
    .append_parameter_override("voxel_points_threshold", 1)
    .append_parameter_override("min_radius_m", 0.0)
    .append_parameter_override("max_radius_m", 100.0)
    .append_parameter_override("visibility_estimation_max_range_m", 100.0)
    .append_parameter_override("use_return_type_classification", false)
    .append_parameter_override("filter_secondary_returns", false)
    .append_parameter_override("secondary_noise_threshold", 0)
    .append_parameter_override("intensity_threshold", 10)
    .append_parameter_override("primary_return_types", std::vector<int>{1})
    .append_parameter_override("publish_noise_cloud", false)
    .append_parameter_override("visibility_estimation_max_secondary_voxel_count", 100)
    .append_parameter_override("visibility_estimation_only", false)
    .append_parameter_override("visibility_error_threshold", 0.1)
    .append_parameter_override("visibility_warn_threshold", 0.2)
    .append_parameter_override("filter_ratio_error_threshold", 0.1)
    .append_parameter_override("filter_ratio_warn_threshold", 0.2);
}

class PolarVoxelOutlierFilterTest : public ::testing::Test
{
};

TEST_F(PolarVoxelOutlierFilterTest, CartesianCloud_ReturnClassificationFalse)
{
  auto options = make_node_options()
                   .append_parameter_override("use_return_type_classification", false)
                   .append_parameter_override("intensity_threshold", 10);
  PolarVoxelOutlierFilterComponentPublic node(options);
  auto input = make_cartesian_cloud();
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);
  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);
  EXPECT_EQ(output.width, 3u);
  EXPECT_EQ(output.height, 1u);
}

TEST_F(PolarVoxelOutlierFilterTest, CartesianCloud_ReturnClassificationTrue)
{
  auto options = make_node_options()
                   .append_parameter_override("use_return_type_classification", true)
                   .append_parameter_override("primary_return_types", std::vector<int>{1})
                   .append_parameter_override("intensity_threshold", 10);
  PolarVoxelOutlierFilterComponentPublic node(options);
  auto input = make_cartesian_cloud();
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);
  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);
  EXPECT_EQ(output.width, 2u);
  EXPECT_EQ(output.height, 1u);
}

TEST_F(PolarVoxelOutlierFilterTest, CartesianCloud_IntensityThreshold)
{
  auto options = make_node_options().append_parameter_override("intensity_threshold", 5);
  PolarVoxelOutlierFilterComponentPublic node(options);
  auto input = make_cartesian_cloud();
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);
  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);
  EXPECT_EQ(output.width, 1u);
  EXPECT_EQ(output.height, 1u);
}

TEST_F(PolarVoxelOutlierFilterTest, PolarCloud_ReturnClassificationFalse)
{
  auto options = make_node_options()
                   .append_parameter_override("use_return_type_classification", false)
                   .append_parameter_override("intensity_threshold", 10);
  PolarVoxelOutlierFilterComponentPublic node(options);
  auto input = make_polar_cloud();
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);
  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);
  EXPECT_EQ(output.width, 2u);
  EXPECT_EQ(output.height, 1u);
}

TEST_F(PolarVoxelOutlierFilterTest, PolarCloud_ReturnClassificationTrue)
{
  auto options = make_node_options()
                   .append_parameter_override("publish_noise_cloud", true)
                   .append_parameter_override("use_return_type_classification", true)
                   .append_parameter_override("primary_return_types", std::vector<int>{1})
                   .append_parameter_override("intensity_threshold", 10);
  PolarVoxelOutlierFilterComponentPublic node(options);
  auto input = make_polar_cloud();
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);
  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);
  EXPECT_EQ(output.width, 1u);
  EXPECT_EQ(output.height, 1u);
  ASSERT_TRUE(node.noise_cloud_pub_ != nullptr);
}

TEST_F(PolarVoxelOutlierFilterTest, PolarCloud_IntensityThreshold)
{
  auto options = make_node_options().append_parameter_override("intensity_threshold", 5);
  PolarVoxelOutlierFilterComponentPublic node(options);
  auto input = make_polar_cloud();
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);
  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);
  EXPECT_EQ(output.width, 1u);
  EXPECT_EQ(output.height, 1u);
}

TEST_F(PolarVoxelOutlierFilterTest, FilterSecondaryReturns)
{
  auto options = make_node_options()
                   .append_parameter_override("use_return_type_classification", true)
                   .append_parameter_override("filter_secondary_returns", true)
                   .append_parameter_override("primary_return_types", std::vector<int>{1})
                   .append_parameter_override("intensity_threshold", 10);
  PolarVoxelOutlierFilterComponentPublic node(options);
  auto input = make_cartesian_cloud();
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);
  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);
  EXPECT_EQ(output.width, 2u);
}

TEST_F(PolarVoxelOutlierFilterTest, NoiseCloudOutput)
{
  auto options = make_node_options()
                   .append_parameter_override("publish_noise_cloud", true)
                   .append_parameter_override("use_return_type_classification", true)
                   .append_parameter_override("primary_return_types", std::vector<int>{1})
                   .append_parameter_override("intensity_threshold", 10);
  PolarVoxelOutlierFilterComponentPublic node(options);
  auto input = make_cartesian_cloud();
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);
  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);
  ASSERT_TRUE(node.noise_cloud_pub_ != nullptr);
}

TEST_F(PolarVoxelOutlierFilterTest, VisibilityMetric)
{
  auto options = make_node_options()
                   .append_parameter_override("use_return_type_classification", false)
                   .append_parameter_override("intensity_threshold", 10);
  PolarVoxelOutlierFilterComponentPublic node(options);
  auto input = make_cartesian_cloud();
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);
  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);
  ASSERT_TRUE(node.visibility_.has_value());
  EXPECT_GE(node.visibility_.value(), 0.0);
  EXPECT_LE(node.visibility_.value(), 1.0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
