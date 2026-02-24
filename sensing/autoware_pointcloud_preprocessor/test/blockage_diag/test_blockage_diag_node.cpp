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
    node_options.append_parameter_override("dust_kernel_size", 1);
    node_options.append_parameter_override("dust_buffering_frames", 5);
    node_options.append_parameter_override("dust_buffering_interval", 2);
    node_options.append_parameter_override("blockage_ratio_threshold", 0.5);
    node_options.append_parameter_override("blockage_count_threshold", 2);
    node_options.append_parameter_override("blockage_kernel", 1);
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
  }

  void TearDown() override { rclcpp::shutdown(); }

  sensor_msgs::msg::PointCloud2 create_valid_pointcloud()
  {
    const int horizontal_bins = 6;  // 360.0 / 60.0
    const int vertical_bins = 4;
    const int num_points = horizontal_bins * vertical_bins;

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

    for (int h = 0; h < horizontal_bins; ++h) {
      for (int v = 0; v < vertical_bins; ++v) {
        *iter_channel = v;
        *iter_azimuth = (-180.0 + h * 60.0) * M_PI / 180.0;
        *iter_distance = 50.0f;
        ++iter_channel;
        ++iter_azimuth;
        ++iter_distance;
      }
    }

    return cloud;
  }

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
      executor_->spin_some();
    }

    return true;
  }

  std::shared_ptr<autoware::pointcloud_preprocessor::BlockageDiagComponent> blockage_diag_node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_pub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostics_msg_;
  bool diagnostics_received_ = false;
};

TEST_F(BlockageDiagIntegrationTest, DiagnosticsOKTest)
{
  input_pub_->publish(create_valid_pointcloud());

  diagnostics_received_ = false;
  ASSERT_TRUE(wait_for_diagnostics()) << "Timeout waiting for diagnostics message";
  verify_diagnostic_level(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK test");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
