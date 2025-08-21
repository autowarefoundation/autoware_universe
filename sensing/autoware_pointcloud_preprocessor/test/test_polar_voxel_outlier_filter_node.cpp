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
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <vector>

using autoware::pointcloud_preprocessor::PolarVoxelOutlierFilterComponent;

// Testable subclass to access protected members/types
class TestableFilter : public PolarVoxelOutlierFilterComponent
{
public:
  using PolarVoxelOutlierFilterComponent::cartesian_to_polar;
  using PolarVoxelOutlierFilterComponent::CartesianCoordinate;
  using PolarVoxelOutlierFilterComponent::enable_secondary_return_filtering_;
  using PolarVoxelOutlierFilterComponent::filter;
  using PolarVoxelOutlierFilterComponent::has_finite_coordinates;
  using PolarVoxelOutlierFilterComponent::has_sufficient_radius;
  using PolarVoxelOutlierFilterComponent::is_valid_polar_point;
  using PolarVoxelOutlierFilterComponent::is_within_radius_range;
  using PolarVoxelOutlierFilterComponent::max_radius_m_;
  using PolarVoxelOutlierFilterComponent::min_radius_m_;
  using PolarVoxelOutlierFilterComponent::PolarCoordinate;
  using PolarVoxelOutlierFilterComponent::primary_return_types_;
  using PolarVoxelOutlierFilterComponent::use_return_type_classification_;
  using PolarVoxelOutlierFilterComponent::voxel_points_threshold_;

  TestableFilter()
  : PolarVoxelOutlierFilterComponent(
      rclcpp::NodeOptions()
        .append_parameter_override("radial_resolution_m", 1.0)
        .append_parameter_override("azimuth_resolution_rad", 0.1)
        .append_parameter_override("elevation_resolution_rad", 0.1)
        .append_parameter_override("voxel_points_threshold", 1)
        .append_parameter_override("min_radius_m", 0.0)
        .append_parameter_override("max_radius_m", 100.0)
        .append_parameter_override("use_return_type_classification", false)
        .append_parameter_override("filter_secondary_returns", false)
        .append_parameter_override("primary_return_types", std::vector<int>{1, 6, 8, 10})
        .append_parameter_override("secondary_noise_threshold", 2)
        .append_parameter_override("publish_noise_cloud", false)
        .append_parameter_override("visibility_estimation_max_range_m", 20.0)
        .append_parameter_override("visibility_estimation_max_secondary_voxel_count", 500)
        .append_parameter_override("visualization_estimation_only", false)
        .append_parameter_override("filter_ratio_error_threshold", 0.5)
        .append_parameter_override("filter_ratio_warn_threshold", 0.7)
        .append_parameter_override("visibility_error_threshold", 0.8)
        .append_parameter_override("visibility_warn_threshold", 0.9))
  {
  }
};

// Helper cloud generators

sensor_msgs::msg::PointCloud2 make_simple_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.height = 1;
  cloud.width = 3;
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  cloud.fields.resize(3);

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

  cloud.point_step = 12;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);

  // Add three points: (1,0,0), (2,0,0), (3,0,0)
  float * data = reinterpret_cast<float *>(cloud.data.data());
  data[0] = 1.0f;
  data[1] = 0.0f;
  data[2] = 0.0f;
  data[3] = 2.0f;
  data[4] = 0.0f;
  data[5] = 0.0f;
  data[6] = 3.0f;
  data[7] = 0.0f;
  data[8] = 0.0f;

  return cloud;
}

sensor_msgs::msg::PointCloud2 make_cloud_with_return_type()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.height = 1;
  cloud.width = 2;
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  cloud.fields.resize(4);

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

  cloud.point_step = 13;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);

  // First point: (1,0,0), return_type=1 (primary)
  float * data = reinterpret_cast<float *>(cloud.data.data());
  data[0] = 1.0f;
  data[1] = 0.0f;
  data[2] = 0.0f;
  cloud.data[12] = 1;  // return_type

  // Second point: (2,0,0), return_type=2 (not primary)
  data = reinterpret_cast<float *>(&cloud.data[13]);
  data[0] = 2.0f;
  data[1] = 0.0f;
  data[2] = 0.0f;
  cloud.data[25] = 2;  // return_type

  return cloud;
}

sensor_msgs::msg::PointCloud2 make_cloud_with_polar_fields()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.height = 1;
  cloud.width = 1;
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  cloud.fields.resize(6);

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

  cloud.point_step = 24;
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);

  // Set polar values
  float distance = 5.0f;
  float azimuth = 0.1f;
  float elevation = 0.2f;

  // Compute matching Cartesian coordinates
  float x = distance * cos(elevation) * cos(azimuth);
  float y = distance * cos(elevation) * sin(azimuth);
  float z = distance * sin(elevation);

  float * data = reinterpret_cast<float *>(cloud.data.data());
  data[0] = distance;   // distance
  data[1] = azimuth;    // azimuth
  data[2] = elevation;  // elevation
  data[3] = x;          // x
  data[4] = y;          // y
  data[5] = z;          // z

  return cloud;
}

// --- Unit tests for helpers ---

TEST(PolarVoxelOutlierFilter, CartesianToPolar)
{
  TestableFilter filter;
  TestableFilter::CartesianCoordinate cart{3.0, 4.0, 0.0};
  auto polar = filter.cartesian_to_polar(cart);
  EXPECT_NEAR(polar.radius, 5.0, 1e-6);
  EXPECT_NEAR(polar.azimuth, atan2(4.0, 3.0), 1e-6);
  EXPECT_NEAR(polar.elevation, 0.0, 1e-6);
}

TEST(PolarVoxelOutlierFilter, HasFiniteCoordinates)
{
  TestableFilter filter;
  TestableFilter::PolarCoordinate polar{1.0, 2.0, 3.0};
  EXPECT_TRUE(filter.has_finite_coordinates(polar));
  polar.radius = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(filter.has_finite_coordinates(polar));
}

TEST(PolarVoxelOutlierFilter, IsWithinRadiusRange)
{
  TestableFilter filter;
  filter.min_radius_m_ = 1.0;
  filter.max_radius_m_ = 10.0;
  TestableFilter::PolarCoordinate polar{5.0, 0.0, 0.0};
  EXPECT_TRUE(filter.is_within_radius_range(polar));
  polar.radius = 0.5;
  EXPECT_FALSE(filter.is_within_radius_range(polar));
  polar.radius = 11.0;
  EXPECT_FALSE(filter.is_within_radius_range(polar));
}

TEST(PolarVoxelOutlierFilter, HasSufficientRadius)
{
  TestableFilter filter;
  TestableFilter::PolarCoordinate polar{1.0, 0.0, 0.0};
  EXPECT_TRUE(filter.has_sufficient_radius(polar));
  polar.radius = 0.0;
  EXPECT_FALSE(filter.has_sufficient_radius(polar));
}

TEST(PolarVoxelOutlierFilter, IsValidPolarPoint)
{
  TestableFilter filter;
  filter.min_radius_m_ = 1.0;
  filter.max_radius_m_ = 10.0;
  TestableFilter::PolarCoordinate polar{5.0, 0.0, 0.0};
  EXPECT_TRUE(filter.is_valid_polar_point(polar));
  polar.radius = 0.0;
  EXPECT_FALSE(filter.is_valid_polar_point(polar));
  polar.radius = 11.0;
  EXPECT_FALSE(filter.is_valid_polar_point(polar));
  polar.radius = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(filter.is_valid_polar_point(polar));
}

// Helper to test that a single-point cloud passes the filter
void expect_single_point_passes(const std::function<sensor_msgs::msg::PointCloud2()> & make_cloud)
{
  auto node = std::make_shared<TestableFilter>();
  auto input_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(make_cloud());
  sensor_msgs::msg::PointCloud2 output_cloud;
  node->filter(input_cloud, nullptr, output_cloud);

  EXPECT_EQ(output_cloud.width, 1u);
  EXPECT_EQ(output_cloud.height, 1u);
}

void expect_points_pass(
  const std::function<sensor_msgs::msg::PointCloud2()> & make_cloud, size_t expected_count)
{
  auto node = std::make_shared<TestableFilter>();
  auto input_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(make_cloud());
  sensor_msgs::msg::PointCloud2 output_cloud;
  node->filter(input_cloud, nullptr, output_cloud);

  EXPECT_EQ(output_cloud.width, expected_count);
  EXPECT_EQ(output_cloud.height, 1u);
}

TEST(PolarVoxelOutlierFilter, FilterSimpleCloud)
{
  expect_points_pass([] { return make_simple_cloud(); }, 3u);
}

TEST(PolarVoxelOutlierFilter, FilterWithPolarCoordinates)
{
  expect_points_pass([] { return make_cloud_with_polar_fields(); }, 1u);
}

// Unit tests for filter logic

TEST(PolarVoxelOutlierFilter, FilterRemovesAllWithHighThreshold)
{
  auto node = std::make_shared<TestableFilter>();
  node->voxel_points_threshold_ = 10;  // Too high for our 3 points
  auto input_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(make_simple_cloud());
  sensor_msgs::msg::PointCloud2 output_cloud;
  node->filter(input_cloud, nullptr, output_cloud);

  // No points should pass the filter
  EXPECT_EQ(output_cloud.width, 0u);
  EXPECT_EQ(output_cloud.row_step, 0u);
  EXPECT_EQ(output_cloud.data.size(), 0u);
}

TEST(PolarVoxelOutlierFilter, FilterWithReturnType)
{
  auto node = std::make_shared<TestableFilter>();
  node->use_return_type_classification_ = true;
  node->primary_return_types_ = {1};
  auto input_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(make_cloud_with_return_type());
  sensor_msgs::msg::PointCloud2 output_cloud;
  node->filter(input_cloud, nullptr, output_cloud);

  // Only the first point should pass (return_type=1)
  EXPECT_EQ(output_cloud.width, 1u);
  EXPECT_EQ(output_cloud.height, 1u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
