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

#include "autoware/pointcloud_preprocessor/outlier_filter/dual_return_outlier_filter_node.hpp"

#include <autoware/point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using autoware::point_types::PointXYZIRCAEDT;
using autoware::point_types::ReturnType;
using autoware::pointcloud_preprocessor::DualReturnOutlierFilterComponent;

// Subclass to expose the protected filter() for testing
class DualReturnOutlierFilterComponentPublic : public DualReturnOutlierFilterComponent
{
public:
  using DualReturnOutlierFilterComponent::filter;
  explicit DualReturnOutlierFilterComponentPublic(const rclcpp::NodeOptions & options)
  : DualReturnOutlierFilterComponent(options)
  {
  }
};

namespace
{
constexpr int vertical_bins = 128;
constexpr int horizontal_bins = 36;
constexpr float deg_to_rad = static_cast<float>(M_PI / 180.0);

// Default values from config/dual_return_outlier_filter_node.param.yaml, except roi_mode
rclcpp::NodeOptions make_node_options(const std::string & roi_mode = "No_ROI")
{
  return rclcpp::NodeOptions()
    .append_parameter_override("x_max", 18.0)
    .append_parameter_override("x_min", -12.0)
    .append_parameter_override("y_max", 2.0)
    .append_parameter_override("y_min", -2.0)
    .append_parameter_override("z_max", 10.0)
    .append_parameter_override("z_min", 0.0)
    .append_parameter_override("min_azimuth_deg", 135.0)
    .append_parameter_override("max_azimuth_deg", 225.0)
    .append_parameter_override("max_distance", 12.0)
    .append_parameter_override("vertical_bins", vertical_bins)
    .append_parameter_override("max_azimuth_diff", 0.5)
    .append_parameter_override("weak_first_distance_ratio", 1.004)
    .append_parameter_override("general_distance_ratio", 1.03)
    .append_parameter_override("weak_first_local_noise_threshold", 2)
    .append_parameter_override("roi_mode", roi_mode)
    .append_parameter_override("visibility_error_threshold", 0.5)
    .append_parameter_override("visibility_warn_threshold", 0.7);
}

// z = 1 keeps the point inside the default xyz ROI (z_min < z < z_max) when it is close enough.
PointXYZIRCAEDT make_point(
  float azimuth_deg, float distance, std::uint8_t return_type = ReturnType::DUAL_WEAK_FIRST,
  std::uint16_t channel = 0)
{
  float wrapped_deg = std::fmod(azimuth_deg, 360.0F);
  if (wrapped_deg < 0.0F) {
    wrapped_deg += 360.0F;
  }
  const float azimuth = wrapped_deg * deg_to_rad;
  PointXYZIRCAEDT p;
  p.x = distance * std::cos(azimuth);
  p.y = distance * std::sin(azimuth);
  p.z = 1.0F;
  p.intensity = 10U;
  p.return_type = return_type;
  p.channel = channel;
  p.azimuth = azimuth;
  p.elevation = 0.0F;
  p.distance = distance;
  p.time_stamp = 0U;
  return p;
}

// Points step_deg apart at 5 m. With the default step of 0.1 deg, every point is in one segment.
struct Arc
{
  float start_deg;
  int num_points;
  float step_deg{0.1F};
  std::uint8_t return_type{ReturnType::DUAL_WEAK_FIRST};
};

void append_arc(pcl::PointCloud<PointXYZIRCAEDT> & cloud, const Arc & arc)
{
  for (int i = 0; i < arc.num_points; ++i) {
    cloud.push_back(
      make_point(arc.start_deg + arc.step_deg * static_cast<float>(i), 5.0F, arc.return_type));
  }
}

sensor_msgs::msg::PointCloud2::SharedPtr to_msg(const pcl::PointCloud<PointXYZIRCAEDT> & cloud)
{
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(cloud, *msg);
  msg->header.frame_id = "base_link";
  return msg;
}

std::vector<PointXYZIRCAEDT> run_filter(
  DualReturnOutlierFilterComponentPublic & node, const pcl::PointCloud<PointXYZIRCAEDT> & cloud)
{
  sensor_msgs::msg::PointCloud2 output;
  node.filter(to_msg(cloud), nullptr, output);
  pcl::PointCloud<PointXYZIRCAEDT> filtered;
  pcl::fromROSMsg(output, filtered);
  return {filtered.points.begin(), filtered.points.end()};
}

// Ring 0: a segment and two isolated points in bin 0 (0 to 10 deg), and a segment in bin 2
// (20 to 30 deg). With weak_first_local_noise_threshold = 2, bin 0 is noisy.
pcl::PointCloud<PointXYZIRCAEDT> make_cloud_with_one_noisy_bin(float isolated_distance = 5.0F)
{
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, {1.0F, 6});
  cloud.push_back(make_point(4.0F, isolated_distance));
  cloud.push_back(make_point(7.0F, isolated_distance));
  append_arc(cloud, {21.0F, 6});
  return cloud;
}

// Six neighbouring points at 5 m, followed by isolated points in the same azimuth bin.
pcl::PointCloud<PointXYZIRCAEDT> make_weak_first_ring(int num_noise_points)
{
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, {1.0F, 6});
  float distance = 6.0F;
  for (int i = 0; i < num_noise_points; ++i) {
    cloud.push_back(make_point(2.0F + static_cast<float>(i), distance));
    distance *= 1.2F;
  }
  return cloud;
}

std::vector<PointXYZIRCAEDT> extract_points(const sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  pcl::fromROSMsg(msg, cloud);
  return {cloud.points.begin(), cloud.points.end()};
}

bool contains_distance(const std::vector<PointXYZIRCAEDT> & points, float distance)
{
  return std::any_of(points.begin(), points.end(), [distance](const PointXYZIRCAEDT & p) {
    return std::abs(p.distance - distance) < 1e-4F;
  });
}

struct FilterResult
{
  std::vector<PointXYZIRCAEDT> output;
  std::vector<PointXYZIRCAEDT> noise;
};

// Runs filter() and returns the output cloud and the published noise cloud. Fails the test if the
// noise cloud is not received.
FilterResult run_filter_with_noise(
  const std::string & roi_mode, const pcl::PointCloud<PointXYZIRCAEDT> & cloud)
{
  DualReturnOutlierFilterComponentPublic node(make_node_options(roi_mode));

  auto listener = std::make_shared<rclcpp::Node>("noise_listener");
  sensor_msgs::msg::PointCloud2::SharedPtr received;
  auto subscription = listener->create_subscription<sensor_msgs::msg::PointCloud2>(
    "dual_return_outlier_filter/debug/pointcloud_noise", rclcpp::SensorDataQoS(),
    [&received](sensor_msgs::msg::PointCloud2::SharedPtr msg) { received = msg; });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(listener);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (subscription->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  sensor_msgs::msg::PointCloud2 output;
  do {
    node.filter(to_msg(cloud), nullptr, output);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  } while (!received && std::chrono::steady_clock::now() < deadline);

  EXPECT_NE(received, nullptr) << "No noise cloud was published";
  FilterResult result;
  result.output = extract_points(output);
  if (received) {
    result.noise = extract_points(*received);
  }
  return result;
}
}  // namespace

TEST(DualReturnOutlierFilterTest, WeakFirstSegmentIsKept)
{
  // Arrange
  DualReturnOutlierFilterComponentPublic node(make_node_options());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, {30.0F, 20});

  // Act
  const auto output = run_filter(node, cloud);

  // Assert
  // Every point of the segment is kept.
  EXPECT_EQ(output.size(), cloud.size());
}

TEST(DualReturnOutlierFilterTest, WeakFirstSegmentIsKeptWhenAzimuthDecreases)
{
  // Arrange
  // 31.9 to 30.0 deg
  DualReturnOutlierFilterComponentPublic node(make_node_options());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, {31.9F, 20, -0.1F});

  // Act
  const auto output = run_filter(node, cloud);

  // Assert
  // Every point of the segment is kept.
  EXPECT_EQ(output.size(), cloud.size());
}

TEST(DualReturnOutlierFilterTest, WeakFirstSegmentIsKeptAcrossWrapAroundWhenAzimuthIncreases)
{
  // Arrange
  // 359.0 to 0.9 deg
  DualReturnOutlierFilterComponentPublic node(make_node_options());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, {359.0F, 20, 0.1F});

  // Act
  const auto output = run_filter(node, cloud);

  // Assert
  // Every point of the segment is kept.
  EXPECT_EQ(output.size(), cloud.size());
}

TEST(DualReturnOutlierFilterTest, WeakFirstSegmentIsKeptAcrossWrapAroundWhenAzimuthDecreases)
{
  // Arrange
  // 1.0 to -0.9 deg
  DualReturnOutlierFilterComponentPublic node(make_node_options());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, {1.0F, 20, -0.1F});

  // Act
  const auto output = run_filter(node, cloud);

  // Assert
  // Every point of the segment is kept.
  EXPECT_EQ(output.size(), cloud.size());
}

TEST(DualReturnOutlierFilterTest, IsolatedWeakFirstPointsAreRemoved)
{
  // Arrange
  // Steps of 3 deg are above max_azimuth_diff (0.5 deg), so no point has a neighbour.
  DualReturnOutlierFilterComponentPublic node(make_node_options());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, {30.0F, 3, 3.0F});

  // Act
  const auto output = run_filter(node, cloud);

  // Assert
  // Every point is removed.
  EXPECT_TRUE(output.empty());
}

TEST(DualReturnOutlierFilterTest, SegmentInNoisyBinIsRemoved)
{
  // Arrange
  DualReturnOutlierFilterComponentPublic node(make_node_options());
  const auto cloud = make_cloud_with_one_noisy_bin();

  // Act
  const auto output = run_filter(node, cloud);

  // Assert
  // Only the segment in the quiet bin 2 is kept.
  ASSERT_EQ(output.size(), 6U);
  for (const auto & p : output) {
    EXPECT_GT(p.azimuth, 20.0F * deg_to_rad);
  }
}

TEST(DualReturnOutlierFilterTest, XyzRoiCountsOnlyIsolatedPointsInsideIt)
{
  // Arrange
  // Isolated points at 5 m are inside the xyz ROI. At 30 m, x is above x_max (18 m), so they
  // are not counted, and bin 0 is not noisy.
  DualReturnOutlierFilterComponentPublic node(make_node_options("Fixed_xyz_ROI"));
  const auto cloud_inside = make_cloud_with_one_noisy_bin(5.0F);
  const auto cloud_outside = make_cloud_with_one_noisy_bin(30.0F);

  // Act
  const auto output_inside = run_filter(node, cloud_inside);
  const auto output_outside = run_filter(node, cloud_outside);

  // Assert
  // Inside: the segment in bin 0 is removed. Outside: both segments are kept. The isolated
  // points are removed in both cases.
  EXPECT_EQ(output_inside.size(), 6U);
  EXPECT_EQ(output_outside.size(), 12U);
}

TEST(DualReturnOutlierFilterTest, AzimuthRoiIgnoresPointsOutsideIt)
{
  // Arrange
  // The azimuth ROI is 135 to 225 deg, so the 36 bins are 2.5 deg wide. Inside the ROI, a
  // segment and two isolated points share the bin from 150 to 152.5 deg, away from its edges so
  // that float rounding does not move a point to the next bin. Outside the ROI (30 deg), the same
  // layout has no bin.
  DualReturnOutlierFilterComponentPublic node(make_node_options("Fixed_azimuth_ROI"));
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, {30.5F, 6});
  cloud.push_back(make_point(31.7F, 5.0F));
  cloud.push_back(make_point(32.4F, 5.0F));
  append_arc(cloud, {150.5F, 6});
  cloud.push_back(make_point(151.7F, 5.0F));
  cloud.push_back(make_point(152.4F, 5.0F));

  // Act
  const auto output = run_filter(node, cloud);

  // Assert
  // Only the segment outside the ROI is kept: it has no bin, so it is never in a noisy bin.
  ASSERT_EQ(output.size(), 6U);
  for (const auto & p : output) {
    EXPECT_LT(p.azimuth, 90.0F * deg_to_rad);
  }
}

TEST(DualReturnOutlierFilterTest, OtherReturnsKeepSegmentsAndRemoveIsolatedPoints)
{
  // Arrange
  // Returns other than the weak first go through a plain ring outlier filter with
  // general_distance_ratio, without the noisy-bin check.
  DualReturnOutlierFilterComponentPublic node(make_node_options());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, {30.0F, 20, 0.1F, ReturnType::DUAL_STRONGEST_FIRST});
  cloud.push_back(make_point(40.0F, 5.0F, ReturnType::DUAL_STRONGEST_FIRST));
  cloud.push_back(make_point(43.0F, 5.0F, ReturnType::DUAL_STRONGEST_FIRST));

  // Act
  const auto output = run_filter(node, cloud);

  // Assert
  // The 20 points of the segment are kept, and the 2 isolated points are removed.
  EXPECT_EQ(output.size(), 20U);
}

TEST(DualReturnOutlierFilterTest, PointWithChannelOutOfRangeIsSkipped)
{
  // Arrange
  DualReturnOutlierFilterComponentPublic node(make_node_options());
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  append_arc(cloud, {30.0F, 20});
  cloud.push_back(make_point(
    32.0F, 5.0F, ReturnType::DUAL_WEAK_FIRST, static_cast<std::uint16_t>(vertical_bins)));

  // Act
  std::vector<PointXYZIRCAEDT> output;
  EXPECT_NO_THROW(output = run_filter(node, cloud));

  // Assert
  // The point with the invalid channel is not in the output.
  EXPECT_EQ(output.size(), 20U);
}

TEST(DualReturnOutlierFilterTest, VisibilityAndNoiseCloudReflectNoisyBins)
{
  // Arrange
  auto node = std::make_shared<DualReturnOutlierFilterComponentPublic>(make_node_options());
  auto listener = std::make_shared<rclcpp::Node>("dual_return_outlier_filter_test_listener");
  std::optional<float> visibility;
  std::optional<std::size_t> num_noise_points;
  auto visibility_sub =
    listener->create_subscription<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "dual_return_outlier_filter/debug/visibility", rclcpp::SensorDataQoS(),
      [&](const autoware_internal_debug_msgs::msg::Float32Stamped::ConstSharedPtr msg) {
        visibility = msg->data;
      });
  auto noise_sub = listener->create_subscription<sensor_msgs::msg::PointCloud2>(
    "dual_return_outlier_filter/debug/pointcloud_noise", rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      num_noise_points = static_cast<std::size_t>(msg->width) * msg->height;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(listener);
  const auto cloud = make_cloud_with_one_noisy_bin();

  // Act
  // The publishers are best effort, so filter() runs until the listener has both messages.
  const auto received_both = [&]() { return visibility && num_noise_points; };
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!received_both() && std::chrono::steady_clock::now() < deadline) {
    run_filter(*node, cloud);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Assert
  // Bin 0 of ring 0 is the only noisy pixel. The noise cloud has the 2 isolated points and the
  // 6 points of the segment in bin 0.
  ASSERT_TRUE(visibility.has_value());
  ASSERT_TRUE(num_noise_points.has_value());
  EXPECT_FLOAT_EQ(*visibility, 1.0F - 1.0F / static_cast<float>(vertical_bins * horizontal_bins));
  EXPECT_EQ(*num_noise_points, 8U);
}

TEST(DualReturnOutlierFilterTest, ThrowsWhenAzimuthRoiCrossesZero)
{
  // Arrange
  // An azimuth ROI from 315 to 45 deg crosses 0 deg, which is not supported.
  const auto options = make_node_options("Fixed_azimuth_ROI")
                         .append_parameter_override("min_azimuth_deg", 315.0)
                         .append_parameter_override("max_azimuth_deg", 45.0);

  // Act & Assert
  EXPECT_THROW(DualReturnOutlierFilterComponentPublic node(options), std::invalid_argument);
}

TEST(DualReturnOutlierFilterTest, WeakFirstSegmentInQuietBinIsKept)
{
  // Arrange
  // One noise point in the bin is below weak_first_local_noise_threshold (2), so the bin is quiet
  // and the segment at 5 m is not noise.
  const auto cloud = make_weak_first_ring(1);

  // Act
  const auto result = run_filter_with_noise("No_ROI", cloud);

  // Assert
  // The segment points are kept in the output, and the noise point is not.
  ASSERT_FALSE(result.output.empty());
  for (const auto & p : result.output) {
    EXPECT_FLOAT_EQ(p.distance, 5.0F);
  }
  EXPECT_TRUE(contains_distance(result.output, 5.0F));
  EXPECT_TRUE(contains_distance(result.noise, 6.0F));
}

TEST(DualReturnOutlierFilterTest, WeakFirstSegmentInNoisyBinIsNoise)
{
  // Arrange
  // Two noise points in the bin reach weak_first_local_noise_threshold (2), so the bin is noisy.
  const auto cloud = make_weak_first_ring(2);

  // Act
  const auto result = run_filter_with_noise("No_ROI", cloud);

  // Assert
  // The segment points are removed from the output and published as noise.
  EXPECT_TRUE(result.output.empty());
  EXPECT_TRUE(contains_distance(result.noise, 5.0F));
}

TEST(DualReturnOutlierFilterTest, WeakFirstSegmentInsideXyzRoiIsCountedAsNoise)
{
  // Arrange
  // The bin is noisy as above. All points are inside the default xyz ROI: x is 5 to 9 m, y is
  // below 1 m and z is 1 m, within x (-12, 18), y (-2, 2) and z (0, 10).
  const auto cloud = make_weak_first_ring(2);

  // Act
  const auto result = run_filter_with_noise("Fixed_xyz_ROI", cloud);

  // Assert
  // The segment points inside the ROI are published as noise, as in No_ROI mode.
  EXPECT_TRUE(result.output.empty());
  EXPECT_TRUE(contains_distance(result.noise, 5.0F));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
