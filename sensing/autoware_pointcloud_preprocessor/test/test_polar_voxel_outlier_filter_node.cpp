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
#include <map>
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

// Voxel 1: primary + 2 secondaries (both meet threshold)
// Voxel 2: primary + 1 secondary (meets threshold)
sensor_msgs::msg::PointCloud2 make_filter_test_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.height = 1;
  cloud.width = 7;
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

  // Voxel 1: primary
  float * data = reinterpret_cast<float *>(cloud.data.data());
  data[0] = 1.0f;
  data[1] = 0.0f;
  data[2] = 0.0f;
  cloud.data[12] = 10;  // intensity (primary)
  cloud.data[13] = 1;   // return_type (primary)

  // Voxel 1: secondary 1 (meets threshold)
  data = reinterpret_cast<float *>(&cloud.data[14]);
  data[0] = 1.0f;
  data[1] = 0.0f;
  data[2] = 0.0f;
  cloud.data[26] = 3;  // intensity (secondary)
  cloud.data[27] = 2;  // return_type (secondary)

  // Voxel 1: secondary 2 (meets threshold)
  data = reinterpret_cast<float *>(&cloud.data[28]);
  data[0] = 1.0f;
  data[1] = 0.0f;
  data[2] = 0.0f;
  cloud.data[40] = 4;  // intensity (secondary)
  cloud.data[41] = 2;  // return_type (secondary)

  // Voxel 1: secondary 3 (meets threshold)
  data = reinterpret_cast<float *>(&cloud.data[42]);
  data[0] = 1.0f;
  data[1] = 0.0f;
  data[2] = 0.0f;
  cloud.data[54] = 2;  // intensity (secondary)
  cloud.data[55] = 2;  // return_type (secondary)

  // Voxel 2: primary
  data = reinterpret_cast<float *>(&cloud.data[56]);
  data[0] = 3.0f;
  data[1] = 0.0f;
  data[2] = 0.0f;
  cloud.data[68] = 10;  // intensity (primary)
  cloud.data[69] = 1;   // return_type (primary)

  // Voxel 2: secondary 1 (meets threshold)
  data = reinterpret_cast<float *>(&cloud.data[70]);
  data[0] = 3.0f;
  data[1] = 0.0f;
  data[2] = 0.0f;
  cloud.data[82] = 3;  // intensity (secondary)
  cloud.data[83] = 2;  // return_type (secondary)

  // Voxel 2: secondary 2 (meets threshold)
  data = reinterpret_cast<float *>(&cloud.data[84]);
  data[0] = 3.0f;
  data[1] = 0.0f;
  data[2] = 0.0f;
  cloud.data[96] = 4;  // intensity (secondary)
  cloud.data[97] = 2;  // return_type (secondary)

  return cloud;
}

struct SimplePoint
{
  float x, y, z;
  uint8_t intensity, return_type;
};

std::vector<SimplePoint> extract_points(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<SimplePoint> points;
  for (size_t i = 0; i < cloud.width; ++i) {
    const uint8_t * point_data = &cloud.data[i * cloud.point_step];
    float x = *reinterpret_cast<const float *>(point_data + cloud.fields[0].offset);
    float y = *reinterpret_cast<const float *>(point_data + cloud.fields[1].offset);
    float z = *reinterpret_cast<const float *>(point_data + cloud.fields[2].offset);
    uint8_t intensity = *(point_data + cloud.fields[3].offset);
    uint8_t return_type = *(point_data + cloud.fields[4].offset);
    points.push_back({x, y, z, intensity, return_type});
  }
  return points;
}

rclcpp::NodeOptions make_node_options()
{
  return rclcpp::NodeOptions()
    .append_parameter_override("publish_noise_cloud", false)
    .append_parameter_override("radial_resolution_m", 0.5)
    .append_parameter_override("azimuth_resolution_rad", 0.0175)
    .append_parameter_override("elevation_resolution_rad", 0.0175)
    .append_parameter_override("voxel_points_threshold", 1)
    .append_parameter_override("min_radius_m", 0.5)
    .append_parameter_override("max_radius_m", 300.0)
    .append_parameter_override("visibility_estimation_max_range_m", 10.0)
    .append_parameter_override("visibility_estimation_min_azimuth_rad", 0.00)
    .append_parameter_override("visibility_estimation_max_azimuth_rad", 3.14)
    .append_parameter_override("visibility_estimation_min_elevation_rad", -1.57)
    .append_parameter_override("visibility_estimation_max_elevation_rad", 1.57)
    .append_parameter_override("visibility_estimation_max_secondary_voxel_count", 2)
    .append_parameter_override("visibility_estimation_only", false)
    .append_parameter_override("use_return_type_classification", true)
    .append_parameter_override("filter_secondary_returns", false)
    .append_parameter_override("secondary_noise_threshold", 1)
    .append_parameter_override("intensity_threshold", 5)
    .append_parameter_override("primary_return_types", std::vector<int64_t>{1, 6, 8, 10})
    .append_parameter_override("visibility_warn_threshold", 0.7)
    .append_parameter_override("filter_ratio_error_threshold", 0.5)
    .append_parameter_override("filter_ratio_warn_threshold", 0.7)
    .append_parameter_override("visibility_error_threshold", 0.8)
    .append_parameter_override("visibility_warn_threshold", 0.9)
    .append_parameter_override("publish_area_marker", false)
    .append_parameter_override("num_frames_hysteresis_transition", 1)
    .append_parameter_override("immediate_report_error", false)
    .append_parameter_override("immediate_relax_state", false);
}

class PolarVoxelOutlierFilterTest : public ::testing::Test
{
};

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}

// Only primary returns in valid voxels are output
TEST_F(PolarVoxelOutlierFilterTest, UseReturnTypeClassification_True_PrimaryOnly)
{
  auto options = make_node_options()
                   .append_parameter_override("use_return_type_classification", true)
                   .append_parameter_override("primary_return_types", std::vector<int64_t>{1})
                   .append_parameter_override("intensity_threshold", 5)
                   .append_parameter_override("voxel_points_threshold", 1)
                   .append_parameter_override("secondary_noise_threshold", 3)
                   .append_parameter_override("filter_secondary_returns", true);
  PolarVoxelOutlierFilterComponentPublic node(options);
  auto input = make_filter_test_cloud();
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);
  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);

  auto pts = extract_points(output);
  ASSERT_EQ(pts.size(), 2u);
  EXPECT_EQ(pts[0].x, 1.0f);
  EXPECT_EQ(pts[0].intensity, 10);
  EXPECT_EQ(pts[0].return_type, 1);
  EXPECT_EQ(pts[1].x, 3.0f);
  EXPECT_EQ(pts[1].intensity, 10);
  EXPECT_EQ(pts[1].return_type, 1);
}

// All points in valid voxels are output, regardless of return_type or intensity
TEST_F(PolarVoxelOutlierFilterTest, UseReturnTypeClassification_False_AllPoints)
{
  auto options = make_node_options()
                   .append_parameter_override("use_return_type_classification", false)
                   .append_parameter_override("intensity_threshold", 5)
                   .append_parameter_override("voxel_points_threshold", 1);
  PolarVoxelOutlierFilterComponentPublic node(options);
  auto input = make_filter_test_cloud();
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);
  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);

  auto pts = extract_points(output);
  ASSERT_EQ(pts.size(), 7u);
  std::vector<std::tuple<float, uint8_t, uint8_t>> expected = {
    {1.0f, 10, 1}, {1.0f, 3, 2}, {1.0f, 4, 2}, {1.0f, 2, 2},
    {3.0f, 10, 1}, {3.0f, 3, 2}, {3.0f, 4, 2}};
  for (const auto & exp : expected) {
    bool found = false;
    for (const auto & pt : pts) {
      if (
        pt.x == std::get<0>(exp) && pt.intensity == std::get<1>(exp) &&
        pt.return_type == std::get<2>(exp)) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found);
  }
}

// Only primary returns in valid voxels are output, noise cloud should contain the rest
TEST_F(PolarVoxelOutlierFilterTest, IntensityThreshold_SecondaryReturn_NoiseCloud)
{
  auto options = make_node_options()
                   .append_parameter_override("use_return_type_classification", true)
                   .append_parameter_override("primary_return_types", std::vector<int64_t>{1})
                   .append_parameter_override("intensity_threshold", 5)
                   .append_parameter_override("voxel_points_threshold", 1)
                   .append_parameter_override("secondary_noise_threshold", 3)
                   .append_parameter_override("filter_secondary_returns", true)
                   .append_parameter_override("publish_noise_cloud", true);
  PolarVoxelOutlierFilterComponentPublic node(options);
  auto input = make_filter_test_cloud();
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);
  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);

  auto pts = extract_points(output);
  ASSERT_EQ(pts.size(), 2u);
  EXPECT_EQ(pts[0].x, 1.0f);
  EXPECT_EQ(pts[0].intensity, 10);
  EXPECT_EQ(pts[0].return_type, 1);
  EXPECT_EQ(pts[1].x, 3.0f);
  EXPECT_EQ(pts[1].intensity, 10);
  EXPECT_EQ(pts[1].return_type, 1);
  ASSERT_TRUE(node.noise_cloud_pub_ != nullptr);
}
// Visibility metric should be 0.5 for this cloud (both voxels counted, both are low visibility)
TEST_F(PolarVoxelOutlierFilterTest, VisibilityMetric)
{
  int intensity_threshold = 5;
  int secondary_noise_threshold = 2;
  auto options =
    make_node_options()
      .append_parameter_override("visibility_estimation_only", true)
      .append_parameter_override("use_return_type_classification", true)
      .append_parameter_override("intensity_threshold", intensity_threshold)
      .append_parameter_override("voxel_points_threshold", 1)
      .append_parameter_override("secondary_noise_threshold", secondary_noise_threshold)
      .append_parameter_override("visibility_estimation_max_secondary_voxel_count", 2);
  PolarVoxelOutlierFilterComponentPublic node(options);
  auto input = make_filter_test_cloud();
  auto input_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(input);
  sensor_msgs::msg::PointCloud2 output;
  node.filter(input_ptr, nullptr, output);

  ASSERT_TRUE(node.visibility_.has_value());
  EXPECT_NEAR(node.visibility_.value(), 0.5, 1e-6);
}
