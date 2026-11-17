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

#include "autoware/pointcloud_preprocessor/outlier_filter/dual_return_outlier_filter_node.hpp"

#include <autoware/point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
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
// Default values from config/dual_return_outlier_filter_node.param.yaml
rclcpp::NodeOptions make_node_options(const std::string & roi_mode)
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
    .append_parameter_override("vertical_bins", 128)
    .append_parameter_override("max_azimuth_diff", 50.0)
    .append_parameter_override("weak_first_distance_ratio", 1.004)
    .append_parameter_override("general_distance_ratio", 1.03)
    .append_parameter_override("weak_first_local_noise_threshold", 2)
    .append_parameter_override("roi_mode", roi_mode)
    .append_parameter_override("visibility_error_threshold", 0.5)
    .append_parameter_override("visibility_warn_threshold", 0.7);
}

PointXYZIRCAEDT make_weak_first_point(float azimuth_rad, float distance)
{
  PointXYZIRCAEDT p;
  p.x = distance * std::cos(azimuth_rad);
  p.y = distance * std::sin(azimuth_rad);
  p.z = 1.0F;
  p.intensity = 10U;
  p.return_type = ReturnType::DUAL_WEAK_FIRST;
  p.channel = 0U;
  p.azimuth = azimuth_rad;
  p.elevation = 0.0F;
  p.distance = distance;
  p.time_stamp = 0U;
  return p;
}

// Weak first returns on one ring, all within the first of the 36 azimuth bins (0 to 10 deg).
// The points at 5 m continue each other, so they form a segment. Each point after them is more
// than weak_first_distance_ratio away from the next one, so it is not in the segment and is
// counted as noise. num_noise_points sets how many such points there are.
pcl::PointCloud<PointXYZIRCAEDT> make_weak_first_ring(int num_noise_points)
{
  pcl::PointCloud<PointXYZIRCAEDT> cloud;
  for (int i = 0; i < 6; ++i) {
    cloud.push_back(make_weak_first_point(0.01F * static_cast<float>(i + 1), 5.0F));
  }
  // The last point of a ring is only compared with, so one more point than the noise points.
  float distance = 6.0F;
  for (int i = 0; i <= num_noise_points; ++i) {
    cloud.push_back(make_weak_first_point(0.01F * static_cast<float>(i + 7), distance));
    distance *= 1.2F;
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2::SharedPtr to_msg(const pcl::PointCloud<PointXYZIRCAEDT> & cloud)
{
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(cloud, *msg);
  msg->header.frame_id = "base_link";
  return msg;
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
FilterResult run_filter(
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
  node.filter(to_msg(cloud), nullptr, output);
  while (!received && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(10));
  }

  EXPECT_NE(received, nullptr) << "No noise cloud was published";
  FilterResult result;
  result.output = extract_points(output);
  if (received) {
    result.noise = extract_points(*received);
  }
  return result;
}
}  // namespace

class DualReturnOutlierFilterTest : public ::testing::Test
{
};

TEST_F(DualReturnOutlierFilterTest, WeakFirstSegmentInQuietBinIsKept)
{
  // Arrange
  // One noise point in the bin is below weak_first_local_noise_threshold (2), so the bin is quiet
  // and the segment at 5 m is not noise.
  const auto cloud = make_weak_first_ring(1);

  // Act
  const auto result = run_filter("No_ROI", cloud);

  // Assert
  // The segment points are kept in the output, and the noise point is not.
  ASSERT_FALSE(result.output.empty());
  for (const auto & p : result.output) {
    EXPECT_FLOAT_EQ(p.distance, 5.0F);
  }
  EXPECT_TRUE(contains_distance(result.output, 5.0F));
  EXPECT_TRUE(contains_distance(result.noise, 6.0F));
}

TEST_F(DualReturnOutlierFilterTest, WeakFirstSegmentInNoisyBinIsNoise)
{
  // Arrange
  // Two noise points in the bin reach weak_first_local_noise_threshold (2), so the bin is noisy.
  const auto cloud = make_weak_first_ring(2);

  // Act
  const auto result = run_filter("No_ROI", cloud);

  // Assert
  // The segment points are removed from the output and published as noise.
  EXPECT_TRUE(result.output.empty());
  EXPECT_TRUE(contains_distance(result.noise, 5.0F));
}

TEST_F(DualReturnOutlierFilterTest, WeakFirstSegmentInsideXyzRoiIsCountedAsNoise)
{
  // Arrange
  // The bin is noisy as above. All points are inside the default xyz ROI: x is 5 to 9 m, y is
  // below 1 m and z is 1 m, within x (-12, 18), y (-2, 2) and z (0, 10).
  const auto cloud = make_weak_first_ring(2);

  // Act
  const auto result = run_filter("Fixed_xyz_ROI", cloud);

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
