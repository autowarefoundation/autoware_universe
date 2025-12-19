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

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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
    node_options.append_parameter_override("vertical_bins", 4);
    node_options.append_parameter_override("horizontal_resolution", 60.0);
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
    node_options.append_parameter_override("horizontal_ring_id", 2);

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
    input_pub_ =
      test_node_->create_publisher<sensor_msgs::msg::PointCloud2>("input", rclcpp::SensorDataQoS());

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

  // Helper function to create a pointcloud with specified horizontal bin coverage
  sensor_msgs::msg::PointCloud2 create_pointcloud(double coverage_ratio)
  {
    const int horizontal_bins = static_cast<int>(360.0 / 60.0);  // 6 bins
    const int vertical_bins = 4;
    const int coverage_bins = static_cast<int>(horizontal_bins * coverage_ratio);
    const int num_points = coverage_bins * vertical_bins;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.height = 1;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
      3, "channel", 1, sensor_msgs::msg::PointField::UINT16, "azimuth", 1,
      sensor_msgs::msg::PointField::FLOAT32, "distance", 1, sensor_msgs::msg::PointField::FLOAT32);

    modifier.resize(num_points);

    sensor_msgs::PointCloud2Iterator<uint16_t> iter_channel(cloud, "channel");
    sensor_msgs::PointCloud2Iterator<float> iter_azimuth(cloud, "azimuth");
    sensor_msgs::PointCloud2Iterator<float> iter_distance(cloud, "distance");

    for (int h = 0; h < coverage_bins; ++h) {
      for (int v = 0; v < vertical_bins; ++v) {
        double azimuth_deg = -180.0 + h * 60.0;
        double azimuth_rad = azimuth_deg * M_PI / 180.0;
        float distance = 50.0;

        *iter_channel = v;
        *iter_azimuth = azimuth_rad;
        *iter_distance = distance;

        ++iter_channel;
        ++iter_azimuth;
        ++iter_distance;
      }
    }

    return cloud;
  }

  // Helper to verify diagnostic status level
  void verify_diagnostic_level(uint8_t expected_level, const std::string & test_description)
  {
    ASSERT_NE(diagnostics_msg_, nullptr) << test_description;
    ASSERT_GT(diagnostics_msg_->status.size(), 0) << test_description;

    bool found = false;
    for (const auto & status : diagnostics_msg_->status) {
      if (status.name.find("blockage") != std::string::npos) {
        found = true;
        EXPECT_EQ(status.level, expected_level)
          << test_description << ": Expected level " << static_cast<int>(expected_level)
          << " but got " << static_cast<int>(status.level);
        break;
      }
    }

    ASSERT_TRUE(found) << test_description << ": Could not find blockage_diag status";
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
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostics_msg_;
  bool diagnostics_received_;
};

// Test case: No input produces STALE diagnostic
TEST_F(BlockageDiagIntegrationTest, DiagnosticsStaleTest)
{
  ASSERT_TRUE(wait_for_diagnostics()) << "Timeout waiting for diagnostics message";
  verify_diagnostic_level(diagnostic_msgs::msg::DiagnosticStatus::STALE, "STALE test");
}

// Test case: Empty pointcloud produces WARN diagnostic
TEST_F(BlockageDiagIntegrationTest, DiagnosticsWarnTest)
{
  auto zero_length_pointcloud = create_pointcloud(0.0);
  input_pub_->publish(zero_length_pointcloud);

  diagnostics_received_ = false;
  ASSERT_TRUE(wait_for_diagnostics()) << "Timeout waiting for diagnostics message";
  verify_diagnostic_level(diagnostic_msgs::msg::DiagnosticStatus::WARN, "WARN test");
}

// Test case: Dense pointcloud produces OK diagnostic
TEST_F(BlockageDiagIntegrationTest, DiagnosticsOKTest)
{
  auto no_blockage_pointcloud = create_pointcloud(1.0);
  input_pub_->publish(no_blockage_pointcloud);

  diagnostics_received_ = false;
  ASSERT_TRUE(wait_for_diagnostics()) << "Timeout waiting for diagnostics message";
  verify_diagnostic_level(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK test");
}

// Test case: Blocked pointcloud produces ERROR diagnostic
TEST_F(BlockageDiagIntegrationTest, DiagnosticsErrorTest)
{
  // Publish multiple frames with blockage to trigger ERROR
  for (int frame = 0; frame < 5; ++frame) {
    auto blocked_pointcloud = create_pointcloud(0.3);
    input_pub_->publish(blocked_pointcloud);

    diagnostics_received_ = false;
    ASSERT_TRUE(wait_for_diagnostics()) << "Timeout waiting for diagnostics on frame " << frame;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  diagnostics_received_ = false;
  ASSERT_TRUE(wait_for_diagnostics()) << "Timeout waiting for diagnostics message";
  verify_diagnostic_level(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "ERROR test");
}

class BlockageDiagValidationTest : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("angle_range", std::vector<double>{-180.0, 180.0});
    node_options.append_parameter_override("is_channel_order_top2down", true);
    node_options.append_parameter_override("vertical_bins", 4);
    node_options.append_parameter_override("horizontal_resolution", 60.0);
    node_options.append_parameter_override("enable_dust_diag", false);
    node_options.append_parameter_override("dust_ratio_threshold", 0.3f);
    node_options.append_parameter_override("dust_count_threshold", 2);
    node_options.append_parameter_override("dust_kernel_size", 3);
    node_options.append_parameter_override("dust_buffering_frames", 5);
    node_options.append_parameter_override("dust_buffering_interval", 2);
    node_options.append_parameter_override("blockage_ratio_threshold", 0.5);
    node_options.append_parameter_override("blockage_count_threshold", 2);
    node_options.append_parameter_override("blockage_kernel", 3);
    node_options.append_parameter_override("blockage_buffering_frames", 5);
    node_options.append_parameter_override("blockage_buffering_interval", 2);
    node_options.append_parameter_override("publish_debug_image", false);
    node_options.append_parameter_override("max_distance_range", 200.0);
    node_options.append_parameter_override("horizontal_ring_id", 2);

    // Create the blockage_diag node
    blockage_diag_node_ =
      std::make_shared<autoware::pointcloud_preprocessor::BlockageDiagComponent>(node_options);
  }

  void TearDown()
  {
    blockage_diag_node_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<autoware::pointcloud_preprocessor::BlockageDiagComponent> blockage_diag_node_;
};

TEST_F(BlockageDiagValidationTest, MissingChannelFieldTest)
{
  sensor_msgs::msg::PointCloud2 cloud_without_channel;
  sensor_msgs::PointCloud2Modifier modifier(cloud_without_channel);
  modifier.setPointCloud2Fields(
    2, "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32, "distance", 1,
    sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_THROW(
    { blockage_diag_node_->validate_pointcloud_fields(cloud_without_channel); },
    std::runtime_error);
}

TEST_F(BlockageDiagValidationTest, MissingAzimuthFieldTest)
{
  sensor_msgs::msg::PointCloud2 cloud_without_azimuth;
  sensor_msgs::PointCloud2Modifier modifier(cloud_without_azimuth);
  modifier.setPointCloud2Fields(
    2, "channel", 1, sensor_msgs::msg::PointField::UINT16, "distance", 1,
    sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_THROW(
    { blockage_diag_node_->validate_pointcloud_fields(cloud_without_azimuth); },
    std::runtime_error);
}

TEST_F(BlockageDiagValidationTest, MissingDistanceFieldTest)
{
  sensor_msgs::msg::PointCloud2 cloud_without_distance;
  sensor_msgs::PointCloud2Modifier modifier(cloud_without_distance);
  modifier.setPointCloud2Fields(
    2, "channel", 1, sensor_msgs::msg::PointField::UINT16, "azimuth", 1,
    sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_THROW(
    { blockage_diag_node_->validate_pointcloud_fields(cloud_without_distance); },
    std::runtime_error);
}

TEST_F(BlockageDiagValidationTest, ValidFieldsTest)
{
  sensor_msgs::msg::PointCloud2 cloud_with_all_fields;
  sensor_msgs::PointCloud2Modifier modifier(cloud_with_all_fields);
  modifier.setPointCloud2Fields(
    3, "channel", 1, sensor_msgs::msg::PointField::UINT16, "azimuth", 1,
    sensor_msgs::msg::PointField::FLOAT32, "distance", 1, sensor_msgs::msg::PointField::FLOAT32);

  EXPECT_NO_THROW({ blockage_diag_node_->validate_pointcloud_fields(cloud_with_all_fields); });
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
