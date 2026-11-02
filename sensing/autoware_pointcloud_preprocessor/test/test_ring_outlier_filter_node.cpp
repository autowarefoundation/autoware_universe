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

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <thread>
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
constexpr int kVerticalBins = 128;
constexpr int kHorizontalBins = 36;
constexpr float kDegToRad = static_cast<float>(M_PI / 180.0);

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
    .append_parameter_override("vertical_bins", kVerticalBins)
    .append_parameter_override("horizontal_bins", kHorizontalBins)
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
      make_point(channel, (start_deg + 0.1F * static_cast<float>(i)) * kDegToRad, 5.0F));
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

std::vector<PointXYZIRCAEDT> extract_input_points(const sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  pcl::fromROSMsg(msg, cloud);
  return {cloud.points.begin(), cloud.points.end()};
}

float expected_score(int filled_pixels, int horizontal_bins = kHorizontalBins)
{
  return 1.0F -
         static_cast<float>(filled_pixels) / static_cast<float>(kVerticalBins * horizontal_bins);
}
}  // namespace

class RingOutlierFilterTest : public ::testing::Test
{
};

TEST_F(RingOutlierFilterTest, PointsWithRingOutOfRangeAreSkipped)
{
  RingOutlierFilterComponentPublic node(
    make_node_options().append_parameter_override("max_rings_num", 4));

  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, 1, 10.0F, 20);
  append_arc(cloud, 4, 10.0F, 20);      // == max_rings_num
  append_arc(cloud, 65535, 10.0F, 20);  // far beyond max_rings_num

  sensor_msgs::msg::PointCloud2 output;
  node.faster_filter(to_msg(cloud), nullptr, output, TransformInfo{});

  // Only the arc on ring 1 can be in the output
  const auto points = extract_points(output);
  ASSERT_FALSE(points.empty());
  for (const auto & p : points) {
    EXPECT_EQ(p.channel, 1U);
  }
}

TEST_F(RingOutlierFilterTest, TruncatedLastPointIsNotRead)
{
  RingOutlierFilterComponentPublic node(make_node_options());

  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, 1, 10.0F, 20);
  auto msg = to_msg(cloud);
  // A trailing partial point that ends before its channel field
  msg->data.resize(msg->data.size() + 10, 0U);
  msg->data.shrink_to_fit();

  sensor_msgs::msg::PointCloud2 output;
  node.faster_filter(msg, nullptr, output, TransformInfo{});

  const auto points = extract_points(output);
  ASSERT_FALSE(points.empty());
  for (const auto & p : points) {
    EXPECT_EQ(p.channel, 1U);
  }
}

TEST_F(RingOutlierFilterTest, ChannelAbove255IsKeptInOutput)
{
  RingOutlierFilterComponentPublic node(
    make_node_options().append_parameter_override("max_rings_num", 512));

  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, 300, 10.0F, 20);

  sensor_msgs::msg::PointCloud2 output;
  node.faster_filter(to_msg(cloud), nullptr, output, TransformInfo{});

  const auto points = extract_points(output);
  ASSERT_FALSE(points.empty());
  for (const auto & p : points) {
    EXPECT_EQ(p.channel, 300U);
  }
}

TEST_F(RingOutlierFilterTest, VisibilityScoreUsesEveryHorizontalBin)
{
  RingOutlierFilterComponentPublic node(make_node_options());

  // Three points (more than noise_threshold) at the center of each of the 36 bins of ring 0
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  const float bin_width_deg = 360.0F / static_cast<float>(kHorizontalBins);
  for (int bin = 0; bin < kHorizontalBins; ++bin) {
    const float center_rad = (static_cast<float>(bin) + 0.5F) * bin_width_deg * kDegToRad;
    for (int i = 0; i < 3; ++i) {
      cloud.push_back(make_point(0, center_rad, 5.0F));
    }
  }

  EXPECT_FLOAT_EQ(node.calculate_visibility_score(*to_msg(cloud)), expected_score(kHorizontalBins));
}

TEST_F(RingOutlierFilterTest, VisibilityScoreAzimuthJustBelowMaxStaysInLastBin)
{
  // With 35 bins over 360 deg, (max_azimuth - epsilon) / horizontal_resolution rounds to exactly
  // 35.0 in float, i.e. one past the last bin of the ring.
  constexpr int horizontal_bins = 35;
  RingOutlierFilterComponentPublic node(
    make_node_options().append_parameter_override("horizontal_bins", horizontal_bins));

  // Computed the same way as in the node
  const float max_azimuth = 360.0F * (M_PI / 180.f);
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  for (int i = 0; i < 3; ++i) {
    cloud.push_back(make_point(0, 0.0F, 5.0F));                               // ring 0, first bin
    cloud.push_back(make_point(0, std::nextafter(max_azimuth, 0.0F), 5.0F));  // ring 0, last bin
    cloud.push_back(make_point(1, 0.0F, 5.0F));                               // ring 1, first bin
  }

  EXPECT_FLOAT_EQ(
    node.calculate_visibility_score(*to_msg(cloud)), expected_score(3, horizontal_bins));
}

TEST_F(RingOutlierFilterTest, VisibilityScoreIgnoresChannelOutOfRange)
{
  RingOutlierFilterComponentPublic node(make_node_options());

  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  for (int i = 0; i < 3; ++i) {
    cloud.push_back(make_point(0, 5.0F * kDegToRad, 5.0F));
    cloud.push_back(make_point(kVerticalBins, 5.0F * kDegToRad, 5.0F));  // == vertical_bins
  }

  float score = 0.0F;
  EXPECT_NO_THROW(score = node.calculate_visibility_score(*to_msg(cloud)));
  EXPECT_FLOAT_EQ(score, expected_score(1));
}

TEST_F(RingOutlierFilterTest, VisibilityScoreAppliesNoiseThreshold)
{
  RingOutlierFilterComponentPublic node(make_node_options());

  // One point per bin is below noise_threshold (=2), so no bins should be counted as filled.
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  const float bin_width_deg = 360.0F / static_cast<float>(kHorizontalBins);
  for (int bin = 0; bin < 5; ++bin) {
    const float center_rad = (static_cast<float>(bin) + 0.5F) * bin_width_deg * kDegToRad;
    cloud.push_back(make_point(0, center_rad, 5.0F));
  }

  EXPECT_FLOAT_EQ(node.calculate_visibility_score(*to_msg(cloud)), 1.0F);
}

TEST_F(RingOutlierFilterTest, ContinuityBreaksAtExactOneDegree)
{
  RingOutlierFilterComponentPublic node(make_node_options());

  // The filter splits a walk unless azimuth_diff < 1 degree, comparing a float azimuth_diff with a
  // double threshold. float(pi / 180) is slightly below that threshold, so "exactly 1 degree" in
  // float would count as continuous. Use the float neighbors on each side of the threshold.
  const float one_deg_f = static_cast<float>(M_PI / 180.0);
  const float just_below_one_deg = std::nextafter(one_deg_f, 0.0F);
  const float just_above_one_deg =
    std::nextafter(one_deg_f, std::numeric_limits<float>::infinity());

  pcl::PointCloud<PointXYZIRCAEDT> continuous_cloud;
  continuous_cloud.push_back(make_point(0, 0.0F * kDegToRad, 5.0F));
  continuous_cloud.push_back(make_point(0, just_below_one_deg, 5.0F));
  continuous_cloud.push_back(make_point(0, 2.0F * just_below_one_deg, 5.0F));

  sensor_msgs::msg::PointCloud2 continuous_output;
  node.faster_filter(to_msg(continuous_cloud), nullptr, continuous_output, TransformInfo{});
  EXPECT_FALSE(extract_points(continuous_output).empty());

  pcl::PointCloud<PointXYZIRCAEDT> boundary_cloud;
  boundary_cloud.push_back(make_point(0, 0.0F, 5.0F));
  boundary_cloud.push_back(make_point(0, just_above_one_deg, 5.0F));
  boundary_cloud.push_back(make_point(0, 2.0F * just_above_one_deg, 5.0F));

  sensor_msgs::msg::PointCloud2 boundary_output;
  node.faster_filter(to_msg(boundary_cloud), nullptr, boundary_output, TransformInfo{});
  EXPECT_TRUE(extract_points(boundary_output).empty());
}

TEST_F(RingOutlierFilterTest, ContinuityBreaksAtExactDistanceRatio)
{
  RingOutlierFilterComponentPublic node(make_node_options());

  pcl::PointCloud<PointXYZIRCAEDT> continuous_cloud;
  continuous_cloud.push_back(make_point(0, 0.0F * kDegToRad, 10.0F));
  continuous_cloud.push_back(make_point(0, 0.1F * kDegToRad, 10.29F));
  continuous_cloud.push_back(make_point(0, 0.2F * kDegToRad, 10.58F));

  sensor_msgs::msg::PointCloud2 continuous_output;
  node.faster_filter(to_msg(continuous_cloud), nullptr, continuous_output, TransformInfo{});
  EXPECT_FALSE(extract_points(continuous_output).empty());

  pcl::PointCloud<PointXYZIRCAEDT> boundary_cloud;
  boundary_cloud.push_back(make_point(0, 0.0F * kDegToRad, 10.0F));
  boundary_cloud.push_back(make_point(0, 0.1F * kDegToRad, 10.3F));
  boundary_cloud.push_back(make_point(0, 0.2F * kDegToRad, 10.6F));

  sensor_msgs::msg::PointCloud2 boundary_output;
  node.faster_filter(to_msg(boundary_cloud), nullptr, boundary_output, TransformInfo{});
  EXPECT_TRUE(extract_points(boundary_output).empty());
}

TEST_F(RingOutlierFilterTest, TransformPathAppliesTranslation)
{
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
  node.faster_filter(input_msg, nullptr, output, tf);

  const auto points = extract_points(output);
  ASSERT_FALSE(points.empty());

  const auto & in = cloud.points.front();
  const auto & out = points.front();
  EXPECT_NEAR(out.x, in.x + 1.5F, 1e-4F);
  EXPECT_NEAR(out.y, in.y - 2.0F, 1e-4F);
  EXPECT_NEAR(out.z, in.z + 0.25F, 1e-4F);
}

TEST_F(RingOutlierFilterTest, OutlierPointcloudPublishesDistinctPointsFromWalk)
{
  auto node = std::make_shared<RingOutlierFilterComponentPublic>(
    make_node_options().append_parameter_override("publish_outlier_pointcloud", true));

  auto listener = std::make_shared<rclcpp::Node>("ring_outlier_filter_test_listener");
  sensor_msgs::msg::PointCloud2::SharedPtr received_outlier;
  auto sub = listener->create_subscription<sensor_msgs::msg::PointCloud2>(
    "debug/ring_outlier_filter", rclcpp::QoS{10},
    [&received_outlier](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      received_outlier = msg;
    });

  (void)sub;

  // Generous deadlines for slow CI machines; both loops exit as soon as the condition is met
  const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (node->count_subscribers("debug/ring_outlier_filter") == 0 &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    rclcpp::spin_some(listener);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(node->count_subscribers("debug/ring_outlier_filter"), 0U);

  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  cloud.push_back(make_point(0, 0.00F * kDegToRad, 5.00F));
  cloud.push_back(make_point(0, 0.01F * kDegToRad, 5.01F));
  cloud.push_back(make_point(0, 0.02F * kDegToRad, 5.02F));
  cloud.push_back(make_point(0, 5.00F * kDegToRad, 5.00F));

  sensor_msgs::msg::PointCloud2 output;
  node->faster_filter(to_msg(cloud), nullptr, output, TransformInfo{});

  const auto receive_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!received_outlier && std::chrono::steady_clock::now() < receive_deadline) {
    rclcpp::spin_some(listener);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_NE(received_outlier, nullptr);
  const auto outlier_points = extract_input_points(*received_outlier);
  ASSERT_EQ(outlier_points.size(), 3U);

  EXPECT_NE(outlier_points[0].x, outlier_points[2].x);
  EXPECT_NE(outlier_points[0].x, outlier_points[1].x);
  EXPECT_NE(outlier_points[1].x, outlier_points[2].x);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
