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
#include <vector>

class PickupBasedVoxelGridDownsampleFilterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Create node options with parameters
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("voxel_size_x", 0.1);
    node_options.append_parameter_override("voxel_size_y", 0.1);
    node_options.append_parameter_override("voxel_size_z", 0.1);

    // Create the filter node
    filter_node_ = std::make_shared<
      autoware::pointcloud_preprocessor::PickupBasedVoxelGridDownsampleFilterComponent>(
      node_options);

    // Create test node for publishing and subscribing
    test_node_ = std::make_shared<rclcpp::Node>("test_node");

    // Create publisher for input point cloud with appropriate QoS
    input_publisher_ =
      test_node_->create_publisher<sensor_msgs::msg::PointCloud2>("input", rclcpp::SensorDataQoS());

    // Create subscriber for output point cloud with appropriate QoS
    output_subscriber_ = test_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "output", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        received_output_ = true;
        output_msg_ = *msg;
      });

    // Allow time for setup
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<autoware::pointcloud_preprocessor::PickupBasedVoxelGridDownsampleFilterComponent>
    filter_node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr output_subscriber_;

  bool received_output_ = false;
  sensor_msgs::msg::PointCloud2 output_msg_;
};

TEST_F(PickupBasedVoxelGridDownsampleFilterTest, TestPointCloudPublishing)
{
  // Create test point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
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
  modifier.resize(1);  // 3x1 grid of points

  // Publish input point cloud
  input_publisher_->publish(cloud);

  // Spin nodes to process messages
  auto start_time = std::chrono::steady_clock::now();
  auto timeout = std::chrono::seconds(5);

  while (!received_output_ && (std::chrono::steady_clock::now() - start_time) < timeout) {
    rclcpp::spin_some(filter_node_);
    rclcpp::spin_some(test_node_);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  // Check that output was received
  EXPECT_TRUE(received_output_) << "Expected to receive output point cloud";

  // Check output point cloud is same size as input
  if (received_output_) {
    EXPECT_EQ(output_msg_.height, cloud.height);
    EXPECT_EQ(output_msg_.width, cloud.width);
    EXPECT_EQ(output_msg_.data.size(), cloud.data.size());
  }
}

TEST_F(PickupBasedVoxelGridDownsampleFilterTest, TestEmptyPointCloudPublishing)
{
  // Create empty point cloud
  sensor_msgs::msg::PointCloud2 empty_cloud;
  empty_cloud.header.frame_id = "base_link";
  empty_cloud.header.stamp = rclcpp::Clock().now();
  empty_cloud.height = 1;
  empty_cloud.width = 0;
  empty_cloud.is_dense = true;
  empty_cloud.is_bigendian = false;

  // Create point cloud with x, y, z fields but no data
  sensor_msgs::PointCloud2Modifier modifier(empty_cloud);
  modifier.setPointCloud2Fields(
    3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);

  // Don't add any points - keep it empty
  modifier.resize(0);

  // Reset the received flag
  received_output_ = false;

  // Publish empty point cloud
  input_publisher_->publish(empty_cloud);

  // Spin nodes to process messages
  auto start_time = std::chrono::steady_clock::now();
  auto timeout = std::chrono::seconds(5);

  while (!received_output_ && (std::chrono::steady_clock::now() - start_time) < timeout) {
    rclcpp::spin_some(filter_node_);
    rclcpp::spin_some(test_node_);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  // Check that output was received
  EXPECT_TRUE(received_output_) << "Expected to receive empty output point cloud";

  // Check output point cloud is same size as input
  if (received_output_) {
    EXPECT_EQ(output_msg_.height, empty_cloud.height);
    EXPECT_EQ(output_msg_.width, empty_cloud.width);
    EXPECT_EQ(output_msg_.data.size(), empty_cloud.data.size());
  }
}
