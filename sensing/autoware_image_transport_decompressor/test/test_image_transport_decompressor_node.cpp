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

// Node-level wiring test: decompress() itself is covered by test_image_transport_decompressor.cpp.
// This file only checks that the node subscribes, forwards the "encoding" parameter, publishes
// decompress()'s image when there is one, and stays silent otherwise.

#include "autoware/image_transport_decompressor/image_transport_decompressor_node.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr const char * input_topic = "/image_transport_decompressor/input/compressed_image";
constexpr const char * output_topic = "/image_transport_decompressor/output/raw_image";

constexpr int image_width = 4;
constexpr int image_height = 2;
constexpr int blue = 10;
constexpr int green = 20;
constexpr int red = 30;

std::vector<uint8_t> encode_png(const cv::Mat & image)
{
  std::vector<uint8_t> data;
  if (!cv::imencode(".png", image, data)) {
    throw std::runtime_error("failed to prepare the compressed test input");
  }
  return data;
}

// A bgr8 camera, compressed the way compressed_image_transport writes it.
sensor_msgs::msg::CompressedImage make_bgr8_compressed_image()
{
  sensor_msgs::msg::CompressedImage message;
  message.header.frame_id = "camera";
  message.format = "bgr8; png compressed bgr8";
  message.data =
    encode_png(cv::Mat(image_height, image_width, CV_8UC3, cv::Scalar(blue, green, red)));
  return message;
}

std::vector<uint8_t> leading_bytes(const sensor_msgs::msg::Image & image, const size_t count)
{
  if (image.data.size() < count) {
    return {};
  }
  return std::vector<uint8_t>(
    image.data.begin(), image.data.begin() + static_cast<std::ptrdiff_t>(count));
}
}  // namespace

// Drives the node over real publish/subscribe: the fixture owns the ROS context, the executor
// thread and the test-side pub/sub wiring.
class ImageTransportDecompressorNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    test_control_node_ = std::make_shared<rclcpp::Node>("test_control_node");
    // The node uses SensorDataQoS on both of its endpoints, so the test side has to match it.
    raw_image_subscription_ = test_control_node_->create_subscription<sensor_msgs::msg::Image>(
      output_topic, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(message_mutex_);
        received_image_ = msg;
      });
    compressed_image_publisher_ =
      test_control_node_->create_publisher<sensor_msgs::msg::CompressedImage>(
        input_topic, rclcpp::SensorDataQoS());
    executor_->add_node(test_control_node_);
  }

  void TearDown() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    rclcpp::shutdown();
  }

  // Bring up the decompressor node with the given "encoding" parameter and start spinning.
  void start_decompressor_node(const std::string & encoding_parameter)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"encoding", encoding_parameter}});
    decompressor_node_ =
      std::make_shared<autoware::image_preprocessor::ImageTransportDecompressor>(options);
    executor_->add_node(decompressor_node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    // Poll for discovery instead of sleeping a fixed duration, which varies across machines.
    const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < discovery_deadline) {
      if (
        compressed_image_publisher_->get_subscription_count() > 0 &&
        raw_image_subscription_->get_publisher_count() > 0) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // Publish and return the decompressed image, or nullptr on timeout. The input is republished
  // while waiting because SensorDataQoS is best effort.
  sensor_msgs::msg::Image::SharedPtr publish_and_wait(
    const sensor_msgs::msg::CompressedImage & compressed_image,
    const std::chrono::seconds timeout = std::chrono::seconds(5))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      compressed_image_publisher_->publish(compressed_image);
      for (int i = 0; i < 20; ++i) {
        {
          std::lock_guard<std::mutex> lock(message_mutex_);
          if (received_image_) {
            return received_image_;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

    std::lock_guard<std::mutex> lock(message_mutex_);
    return received_image_;
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  rclcpp::Node::SharedPtr test_control_node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_publisher_;
  std::shared_ptr<autoware::image_preprocessor::ImageTransportDecompressor> decompressor_node_;

  std::mutex message_mutex_;
  sensor_msgs::msg::Image::SharedPtr received_image_;
};

TEST_F(ImageTransportDecompressorNodeTest, PublishesDecompressedImage)
{
  // Arrange
  start_decompressor_node("default");

  // Act
  const auto image = publish_and_wait(make_bgr8_compressed_image());

  // Assert
  ASSERT_NE(image, nullptr) << "no image was published within the timeout";
  EXPECT_EQ(image->header.frame_id, "camera");
  EXPECT_EQ(image->width, static_cast<uint32_t>(image_width));
  EXPECT_EQ(image->height, static_cast<uint32_t>(image_height));
  EXPECT_EQ(image->encoding, "bgr8");
  EXPECT_EQ(leading_bytes(*image, 3), std::vector<uint8_t>({blue, green, red}));
}

TEST_F(ImageTransportDecompressorNodeTest, HonorsEncodingParameter)
{
  // Arrange
  start_decompressor_node("rgb8");

  // Act
  const auto image = publish_and_wait(make_bgr8_compressed_image());

  // Assert
  ASSERT_NE(image, nullptr) << "no image was published within the timeout";
  EXPECT_EQ(image->encoding, "rgb8");
  EXPECT_EQ(leading_bytes(*image, 3), std::vector<uint8_t>({red, green, blue}));
}

TEST_F(ImageTransportDecompressorNodeTest, DropsUndecodableDataWithoutPublishing)
{
  // Arrange
  start_decompressor_node("bgr8");
  sensor_msgs::msg::CompressedImage message;
  message.header.frame_id = "camera";
  message.format = "bgr8; png compressed bgr8";
  message.data = {0x01, 0x02, 0x03, 0x04, 0x05};

  // Act
  const auto image = publish_and_wait(message, std::chrono::seconds(1));

  // Assert
  EXPECT_EQ(image, nullptr) << "an image was published even though the payload cannot be decoded";
}
