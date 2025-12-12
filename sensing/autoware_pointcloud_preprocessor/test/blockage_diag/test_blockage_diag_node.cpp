// Copyright 2025 TIER IV
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

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_diag_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

using PointXYZIRCAEDT = autoware::point_types::PointXYZIRCAEDT;

class BlockageDiagIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Create test parameters
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("angle_range", std::vector<double>{-180.0, 180.0});
    node_options.append_parameter_override("is_channel_order_top2down", true);
    node_options.append_parameter_override("vertical_bins", 32);
    node_options.append_parameter_override("horizontal_resolution", 0.4);
    node_options.append_parameter_override("enable_dust_diag", true);
    node_options.append_parameter_override("dust_ratio_threshold", 0.3);
    node_options.append_parameter_override("dust_count_threshold", 2);
    node_options.append_parameter_override("dust_kernel_size", 3);
    node_options.append_parameter_override("dust_buffering_frames", 5);
    node_options.append_parameter_override("dust_buffering_interval", 2);
    node_options.append_parameter_override("blockage_ratio_threshold", 0.5);
    node_options.append_parameter_override("blockage_count_threshold", 2);
    node_options.append_parameter_override("blockage_kernel", 3);
    node_options.append_parameter_override("blockage_buffering_frames", 5);
    node_options.append_parameter_override("blockage_buffering_interval", 2);
    node_options.append_parameter_override("publish_debug_image", true);
    node_options.append_parameter_override("max_distance_range", 200.0);
    node_options.append_parameter_override("horizontal_ring_id", 16);

    // Create the blockage_diag node
    blockage_diag_node_ =
      std::make_shared<autoware::pointcloud_preprocessor::BlockageDiagComponent>(node_options);

    // Create test node for publishers and subscribers
    test_node_ = std::make_shared<rclcpp::Node>("test_node");

    // Create executor
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(blockage_diag_node_);
    executor_->add_node(test_node_);

    // Start executor in a separate thread
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    // Wait for node setup
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Create publisher for input topic (blockage_diag subscribes to "input")
    input_pub_ = test_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::SensorDataQoS());

    // Create subscriber for output topic (blockage_diag publishes to "output")
    output_received_ = false;
    output_sub_ = test_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "output", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        output_msg_ = msg;
        output_received_ = true;
      });

    // Create subscriber for diagnostics topic
    diagnostics_received_ = false;
    diagnostics_sub_ = test_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10),
      [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        diagnostics_msg_ = msg;
        diagnostics_received_ = true;
      });

    // Wait for publisher/subscriber setup
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    executor_.reset();
    blockage_diag_node_.reset();
    test_node_.reset();
    rclcpp::shutdown();
  }

  sensor_msgs::msg::PointCloud2 create_test_pointcloud(
    const rclcpp::Time & stamp, int num_points = 100)
  {
    pcl::PointCloud<PointXYZIRCAEDT> pcl_cloud;
    pcl_cloud.header.frame_id = "lidar_top";
    pcl_cloud.height = 1;
    pcl_cloud.width = num_points;
    pcl_cloud.is_dense = false;

    // Generate test points
    for (int i = 0; i < num_points; ++i) {
      PointXYZIRCAEDT point;
      float angle = 2.0 * M_PI * i / num_points;
      float distance = 10.0 + (i % 10);

      point.x = distance * std::cos(angle);
      point.y = distance * std::sin(angle);
      point.z = 0.0;
      point.intensity = 100;
      point.return_type = 0;
      point.channel = i % 32;  // 32 channels
      point.azimuth = angle;
      point.elevation = 0.0;
      point.distance = distance;
      point.time_stamp = i * 1000;  // nanoseconds

      pcl_cloud.points.push_back(point);
    }

    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(pcl_cloud, ros_cloud);
    ros_cloud.header.stamp = stamp;
    ros_cloud.header.frame_id = "lidar_top";

    return ros_cloud;
  }

  bool wait_for_output(std::chrono::milliseconds timeout = std::chrono::milliseconds(2000))
  {
    auto start = std::chrono::steady_clock::now();
    output_received_ = false;

    while (!output_received_) {
      if (std::chrono::steady_clock::now() - start > timeout) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return true;
  }

  bool wait_for_diagnostics(std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
  {
    auto start = std::chrono::steady_clock::now();
    diagnostics_received_ = false;

    while (!diagnostics_received_) {
      if (std::chrono::steady_clock::now() - start > timeout) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return true;
  }

  std::shared_ptr<autoware::pointcloud_preprocessor::BlockageDiagComponent> blockage_diag_node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr output_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr output_msg_;
  bool output_received_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostics_msg_;
  bool diagnostics_received_;
};

// Test case: Basic integration test
TEST_F(BlockageDiagIntegrationTest, BasicIntegrationTest)
{
  // Create and publish input pointcloud
  auto timestamp = test_node_->now();
  auto input_cloud = create_test_pointcloud(timestamp, 100);

  input_pub_->publish(input_cloud);

  // Wait for output
  ASSERT_TRUE(wait_for_output()) << "Timeout waiting for output message";

  // Verify output
  ASSERT_NE(output_msg_, nullptr);
  EXPECT_EQ(output_msg_->header.frame_id, "lidar_top");
  EXPECT_GT(output_msg_->width * output_msg_->height, 0);
}

// Test case: Empty pointcloud
TEST_F(BlockageDiagIntegrationTest, EmptyPointcloudTest)
{
  // Create and publish empty pointcloud
  auto timestamp = test_node_->now();
  auto input_cloud = create_test_pointcloud(timestamp, 0);

  input_pub_->publish(input_cloud);

  // Wait for output
  ASSERT_TRUE(wait_for_output()) << "Timeout waiting for output message";

  // Verify output
  ASSERT_NE(output_msg_, nullptr);
  EXPECT_EQ(output_msg_->width * output_msg_->height, 0);
}

// Test case: Multiple pointclouds
TEST_F(BlockageDiagIntegrationTest, MultiplePointcloudsTest)
{
  const int num_clouds = 3;

  for (int i = 0; i < num_clouds; ++i) {
    output_received_ = false;

    auto timestamp = test_node_->now();
    auto input_cloud = create_test_pointcloud(timestamp, 100);

    input_pub_->publish(input_cloud);

    ASSERT_TRUE(wait_for_output()) << "Timeout waiting for output message " << i;
    ASSERT_NE(output_msg_, nullptr);
    EXPECT_GT(output_msg_->width * output_msg_->height, 0);
  }
}

// Test case: Diagnostics STALE test when no input is published
TEST_F(BlockageDiagIntegrationTest, DiagnosticsStaleTest)
{
  // Do not publish any input pointcloud
  // Just wait for diagnostics to be published

  ASSERT_TRUE(wait_for_diagnostics()) << "Timeout waiting for diagnostics message";

  // Verify diagnostics message
  ASSERT_NE(diagnostics_msg_, nullptr);
  ASSERT_GT(diagnostics_msg_->status.size(), 0);

  // Find the blockage_diag status
  bool found_blockage_diag_status = false;
  for (const auto & status : diagnostics_msg_->status) {
    if (status.name.find("BlockageDiag") != std::string::npos ||
        status.name.find("blockage") != std::string::npos) {
      found_blockage_diag_status = true;
      // Check that the level is STALE (3)
      EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::STALE)
        << "Expected STALE level but got: " << static_cast<int>(status.level);
      break;
    }
  }

  ASSERT_TRUE(found_blockage_diag_status) << "Could not find blockage_diag status in diagnostics";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
