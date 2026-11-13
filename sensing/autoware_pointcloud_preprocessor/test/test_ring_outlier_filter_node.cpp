// Copyright 2026 The Autoware Contributors
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

#include "autoware/pointcloud_preprocessor/outlier_filter/ring_outlier_filter_node.hpp"

#include <autoware/point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using autoware::point_types::PointXYZIRC;
using autoware::point_types::PointXYZIRCAEDT;
using autoware::pointcloud_preprocessor::RingOutlierFilterComponent;
using autoware::pointcloud_preprocessor::TransformInfo;

// Subclass to expose the protected methods for testing
class RingOutlierFilterComponentPublic : public RingOutlierFilterComponent
{
public:
  using RingOutlierFilterComponent::calculate_visibility_score;
  using RingOutlierFilterComponent::faster_filter;
  explicit RingOutlierFilterComponentPublic(const rclcpp::NodeOptions & options)
  : RingOutlierFilterComponent(options)
  {
  }
};

namespace
{
constexpr int default_vertical_bins = 128;
constexpr int default_horizontal_bins = 36;
constexpr float deg_to_rad = static_cast<float>(M_PI / 180.0);

// Default values from config/ring_outlier_filter_node.param.yaml
rclcpp::NodeOptions make_node_options()
{
  return rclcpp::NodeOptions()
    .append_parameter_override("distance_ratio", 1.03)
    .append_parameter_override("object_length_threshold", 0.05)
    .append_parameter_override("max_rings_num", 128)
    .append_parameter_override("max_points_num_per_ring", 4000)
    .append_parameter_override("publish_outlier_pointcloud", false)
    .append_parameter_override("min_azimuth_deg", 0.0)
    .append_parameter_override("max_azimuth_deg", 360.0)
    .append_parameter_override("max_distance", 12.0)
    .append_parameter_override("vertical_bins", default_vertical_bins)
    .append_parameter_override("horizontal_bins", default_horizontal_bins)
    .append_parameter_override("noise_threshold", 2)
    .append_parameter_override("processing_time_threshold_sec", 0.01);
}

PointXYZIRCAEDT make_point(std::uint16_t channel, float azimuth_rad, float distance)
{
  PointXYZIRCAEDT p;
  p.x = distance * std::cos(azimuth_rad);
  p.y = distance * std::sin(azimuth_rad);
  p.z = 0.0F;
  p.intensity = 10U;
  p.return_type = 1U;
  p.channel = channel;
  p.azimuth = azimuth_rad;
  p.elevation = 0.0F;
  p.distance = distance;
  p.time_stamp = 0U;
  return p;
}

// A short, dense arc on one ring: consecutive points are 0.1 deg apart at the same distance, so
// they form one walk, and the walk is longer than object_length_threshold, so it is kept.
void append_arc(
  pcl::PointCloud<PointXYZIRCAEDT> & cloud, std::uint16_t channel, float start_deg, int num_points)
{
  for (int i = 0; i < num_points; ++i) {
    cloud.push_back(
      make_point(channel, (start_deg + 0.1F * static_cast<float>(i)) * deg_to_rad, 5.0F));
  }
}

sensor_msgs::msg::PointCloud2::SharedPtr to_msg(const pcl::PointCloud<PointXYZIRCAEDT> & cloud)
{
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(cloud, *msg);
  msg->header.frame_id = "base_link";
  return msg;
}

std::vector<PointXYZIRC> extract_points(const sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::PointCloud<PointXYZIRC> cloud;
  pcl::fromROSMsg(msg, cloud);
  return {cloud.points.begin(), cloud.points.end()};
}

float expected_score(int filled_pixels, int horizontal_bins = default_horizontal_bins)
{
  return 1.0F - static_cast<float>(filled_pixels) /
                  static_cast<float>(default_vertical_bins * horizontal_bins);
}
}  // namespace

class RingOutlierFilterTest : public ::testing::Test
{
};

TEST_F(RingOutlierFilterTest, PointsWithRingOutOfRangeAreSkipped)
{
  // Arrange
  // max_rings_num is 4, so only the arc on ring 1 has a valid ring index; the arcs on rings 4 and
  // 65535 must be skipped.
  RingOutlierFilterComponentPublic node(
    make_node_options().append_parameter_override("max_rings_num", 4));
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, 1, 10.0F, 20);
  append_arc(cloud, 4, 10.0F, 20);      // == max_rings_num
  append_arc(cloud, 65535, 10.0F, 20);  // far beyond max_rings_num
  sensor_msgs::msg::PointCloud2 output;

  // Act
  node.faster_filter(to_msg(cloud), nullptr, output, TransformInfo{});

  // Assert
  // Only the points on ring 1 are kept in the output; the other two arcs are skipped.
  const auto points = extract_points(output);
  ASSERT_FALSE(points.empty());
  for (const auto & p : points) {
    EXPECT_EQ(p.channel, 1U);
  }
}

TEST_F(RingOutlierFilterTest, TruncatedLastPointIsNotRead)
{
  // Arrange
  // The data ends with a partial point that stops before its channel field, so reading that
  // channel would go past the end of the buffer.
  RingOutlierFilterComponentPublic node(make_node_options());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, 1, 10.0F, 20);
  auto msg = to_msg(cloud);
  msg->data.resize(msg->data.size() + 10, 0U);
  msg->data.shrink_to_fit();
  sensor_msgs::msg::PointCloud2 output;

  // Act
  node.faster_filter(msg, nullptr, output, TransformInfo{});

  // Assert
  // The partial point is not read; the 20 points on ring 1 are kept.
  const auto points = extract_points(output);
  ASSERT_FALSE(points.empty());
  for (const auto & p : points) {
    EXPECT_EQ(p.channel, 1U);
  }
}

TEST_F(RingOutlierFilterTest, ZeroPointStepGivesEmptyOutput)
{
  // Arrange
  // Making the input invalid by setting point_step to 0
  // On normal callback FilterBase::is_valid() rejects this as an invalid PointCloud2
  // Call faster_filter() directly to test local guard behaviour
  RingOutlierFilterComponentPublic node(make_node_options());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, 1, 10.0F, 20);
  auto msg = to_msg(cloud);
  msg->point_step = 0;
  sensor_msgs::msg::PointCloud2 output;

  // Act
  node.faster_filter(msg, nullptr, output, TransformInfo{});

  // Assert
  // No points are produced
  EXPECT_TRUE(extract_points(output).empty());
}

TEST_F(RingOutlierFilterTest, ChannelAbove255IsKeptInOutput)
{
  // Arrange
  // Channel 300 does not fit in uint8_t; copying only the low byte would give 44.
  RingOutlierFilterComponentPublic node(
    make_node_options().append_parameter_override("max_rings_num", 512));
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, 300, 10.0F, 20);
  sensor_msgs::msg::PointCloud2 output;

  // Act
  node.faster_filter(to_msg(cloud), nullptr, output, TransformInfo{});

  // Assert
  // The output points have the same channel as the input points.
  const auto points = extract_points(output);
  ASSERT_FALSE(points.empty());
  for (const auto & p : points) {
    EXPECT_EQ(p.channel, 300U);
  }
}

TEST_F(RingOutlierFilterTest, VisibilityScoreUsesEveryHorizontalBin)
{
  // Arrange
  // For each horizontal bin in ring 0, insert 3 points at the bin center.
  // Since noise_threshold = 2, each bin is marked as occupied, so exactly 36 pixels are filled.
  RingOutlierFilterComponentPublic node(make_node_options());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  const float bin_width_deg = 360.0F / static_cast<float>(default_horizontal_bins);
  for (int bin = 0; bin < default_horizontal_bins; ++bin) {
    const float center_rad = (static_cast<float>(bin) + 0.5F) * bin_width_deg * deg_to_rad;
    for (int i = 0; i < 3; ++i) {
      cloud.push_back(make_point(0, center_rad, 5.0F));
    }
  }

  // Act
  const float score = node.calculate_visibility_score(*to_msg(cloud));

  // Assert
  // The expected score is 1 - (36 / (128 * 36)) = 127 / 128.
  EXPECT_FLOAT_EQ(score, expected_score(default_horizontal_bins));
}

TEST_F(RingOutlierFilterTest, VisibilityScoreAzimuthJustBelowMaxStaysInLastBin)
{
  // Arrange
  // With 35 bins over 360 deg, (max_azimuth - epsilon) / horizontal_resolution rounds to exactly
  // 35.0 in float, one past the last bin. The expected 3 pixels are the first and last bin of
  // ring 0 and the first bin of ring 1.
  constexpr int horizontal_bins = 35;
  RingOutlierFilterComponentPublic node(
    make_node_options().append_parameter_override("horizontal_bins", horizontal_bins));
  const float max_azimuth = 360.0F * (M_PI / 180.f);
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  for (int i = 0; i < 3; ++i) {
    cloud.push_back(make_point(0, 0.0F, 5.0F));                               // ring 0, first bin
    cloud.push_back(make_point(0, std::nextafter(max_azimuth, 0.0F), 5.0F));  // ring 0, last bin
    cloud.push_back(make_point(1, 0.0F, 5.0F));                               // ring 1, first bin
  }

  // Act
  const float score = node.calculate_visibility_score(*to_msg(cloud));

  // Assert
  // Bins 0 and 34 of ring 0 and bin 0 of ring 1 are filled, so the expected score is
  // 1 - (3 / (128 * 35)) = 1 - 3 / 4480 = 4477 / 4480 = 0.9993303571
  EXPECT_FLOAT_EQ(score, expected_score(3, horizontal_bins));
}

TEST_F(RingOutlierFilterTest, VisibilityScoreIgnoresChannelOutOfRange)
{
  // Arrange
  // Channel 128 equals vertical_bins, so it has no row in the 128 x 36 image. 3 points (more than
  // noise_threshold) are put on ring 0 and on channel 128 at the same azimuth.
  RingOutlierFilterComponentPublic node(make_node_options());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  for (int i = 0; i < 3; ++i) {
    cloud.push_back(make_point(0, 5.0F * deg_to_rad, 5.0F));
    cloud.push_back(
      make_point(default_vertical_bins, 5.0F * deg_to_rad, 5.0F));  // == vertical_bins
  }

  // Act
  const float score = node.calculate_visibility_score(*to_msg(cloud));

  // Assert
  // Only the point on ring 0 is counted.
  EXPECT_FLOAT_EQ(score, expected_score(1));
}

TEST_F(RingOutlierFilterTest, InvalidVisibilityParametersAreRejectedAtStartup)
{
  // Arrange
  // With 0 horizontal bins, the bin width would be infinite and the last-bin clamp would wrap
  // around.
  const auto options = make_node_options().append_parameter_override("horizontal_bins", 0);

  // Act
  std::string error;
  try {
    RingOutlierFilterComponentPublic node(options);
  } catch (const std::invalid_argument & e) {
    error = e.what();
  }

  // Assert
  // The exception message is the same as the one returned by validate_visibility_param().
  EXPECT_EQ(error, "horizontal_bins must be at least 1");
}

TEST_F(RingOutlierFilterTest, ZeroHorizontalBinsUpdateIsRejected)
{
  // Arrange
  // The same invalid value set at runtime: the update must be refused and the previous value kept.
  RingOutlierFilterComponentPublic node(make_node_options());

  // Act
  const auto result = node.set_parameter(rclcpp::Parameter("horizontal_bins", 0));

  // Assert
  // The update is rejected and the previous value is kept.
  EXPECT_FALSE(result.successful);
  EXPECT_EQ(node.get_parameter("horizontal_bins").as_int(), default_horizontal_bins);
}

TEST_F(RingOutlierFilterTest, MinAzimuthNotBelowMaxAzimuthUpdateIsRejected)
{
  // Arrange
  // min_azimuth_deg equal to max_azimuth_deg would make the bin width 0.
  RingOutlierFilterComponentPublic node(make_node_options());

  // Act
  const auto result = node.set_parameter(rclcpp::Parameter("min_azimuth_deg", 360.0));

  // Assert
  // The update is rejected and the previous value is kept.
  EXPECT_FALSE(result.successful);
  EXPECT_DOUBLE_EQ(node.get_parameter("min_azimuth_deg").as_double(), 0.0);
}

TEST_F(RingOutlierFilterTest, AzimuthStepJustBelowOneDegreeKeepsWalkContinuous)
{
  // Arrange
  // A walk continues while the azimuth step is below 1 degree. float(pi / 180) is slightly below
  // the filter's double threshold, so the float just below it is used. The three points form one
  // walk of about 0.17 m, longer than object_length_threshold, so it is kept.
  RingOutlierFilterComponentPublic node(make_node_options());
  const float step = std::nextafter(static_cast<float>(M_PI / 180.0), 0.0F);
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  cloud.push_back(make_point(0, 0.0F, 5.0F));
  cloud.push_back(make_point(0, step, 5.0F));
  cloud.push_back(make_point(0, 2.0F * step, 5.0F));
  sensor_msgs::msg::PointCloud2 output;

  // Act
  node.faster_filter(to_msg(cloud), nullptr, output, TransformInfo{});

  // Assert
  // The three points are kept in the output.
  EXPECT_FALSE(extract_points(output).empty());
}

TEST_F(RingOutlierFilterTest, AzimuthStepJustAboveOneDegreeBreaksWalk)
{
  // Arrange
  // The float just above 1 degree breaks the walk at every point, so each walk is a single point
  // of length 0 and is removed.
  RingOutlierFilterComponentPublic node(make_node_options());
  const float step =
    std::nextafter(static_cast<float>(M_PI / 180.0), std::numeric_limits<float>::infinity());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  cloud.push_back(make_point(0, 0.0F, 5.0F));
  cloud.push_back(make_point(0, step, 5.0F));
  cloud.push_back(make_point(0, 2.0F * step, 5.0F));
  sensor_msgs::msg::PointCloud2 output;

  // Act
  node.faster_filter(to_msg(cloud), nullptr, output, TransformInfo{});

  // Assert
  // The three points are removed from the output.
  EXPECT_TRUE(extract_points(output).empty());
}

TEST_F(RingOutlierFilterTest, DistanceRatioBelowThresholdKeepsWalkContinuous)
{
  // Arrange
  // Each distance step is below distance_ratio (1.03), so the three points form one walk of about
  // 0.58 m and are kept.
  RingOutlierFilterComponentPublic node(make_node_options());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  cloud.push_back(make_point(0, 0.0F * deg_to_rad, 10.0F));
  cloud.push_back(make_point(0, 0.1F * deg_to_rad, 10.29F));
  cloud.push_back(make_point(0, 0.2F * deg_to_rad, 10.58F));
  sensor_msgs::msg::PointCloud2 output;

  // Act
  node.faster_filter(to_msg(cloud), nullptr, output, TransformInfo{});

  // Assert
  // The three points are kept in the output.
  EXPECT_FALSE(extract_points(output).empty());
}

TEST_F(RingOutlierFilterTest, DistanceRatioAtThresholdBreaksWalk)
{
  // Arrange
  // Each distance step is exactly distance_ratio, which is not below it, so each point is its own
  // walk of length 0 and is removed.
  RingOutlierFilterComponentPublic node(make_node_options());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  cloud.push_back(make_point(0, 0.0F * deg_to_rad, 10.0F));
  cloud.push_back(make_point(0, 0.1F * deg_to_rad, 10.3F));
  cloud.push_back(make_point(0, 0.2F * deg_to_rad, 10.6F));
  sensor_msgs::msg::PointCloud2 output;

  // Act
  node.faster_filter(to_msg(cloud), nullptr, output, TransformInfo{});

  // Assert
  // The three points are removed from the output.
  EXPECT_TRUE(extract_points(output).empty());
}

TEST_F(RingOutlierFilterTest, TransformPathAppliesTranslation)
{
  // Arrange
  // The transform is a pure translation, so every output point is its input point shifted by
  // (1.5, -2.0, 0.25).
  RingOutlierFilterComponentPublic node(make_node_options());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, 2, 30.0F, 20);
  const auto input_msg = to_msg(cloud);
  TransformInfo tf;
  tf.need_transform = true;
  tf.eigen_transform = Eigen::Matrix4f::Identity();
  tf.eigen_transform(0, 3) = 1.5F;
  tf.eigen_transform(1, 3) = -2.0F;
  tf.eigen_transform(2, 3) = 0.25F;
  sensor_msgs::msg::PointCloud2 output;

  // Act
  node.faster_filter(input_msg, nullptr, output, tf);

  // Assert
  // The output points are the input points translated by (1.5, -2.0, 0.25).
  const auto points = extract_points(output);
  ASSERT_FALSE(points.empty());
  const auto & in = cloud.points.front();
  const auto & out = points.front();
  EXPECT_NEAR(out.x, in.x + 1.5F, 1e-4F);
  EXPECT_NEAR(out.y, in.y - 2.0F, 1e-4F);
  EXPECT_NEAR(out.z, in.z + 0.25F, 1e-4F);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
