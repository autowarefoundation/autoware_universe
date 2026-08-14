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

// Characterization test: it pins the message the node publishes today for every combination of
// source image encoding and "encoding" parameter. Rows marked "KNOWN DEFECT" hand the consumer
// something other than what the camera produced.

#include "autoware/image_transport_decompressor/image_transport_decompressor_node.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
// The node name is fixed, so its topics are known in advance.
constexpr const char * input_topic = "/image_transport_decompressor/input/compressed_image";
constexpr const char * output_topic = "/image_transport_decompressor/output/raw_image";

constexpr int image_width = 4;
constexpr int image_height = 2;

// Pixel values of the image the sender compressed, in the channel order of the compressed stream
// (BGR, as signalled by "compressed bgr8" and friends). The 16-bit variants are scaled by 257 so
// that a lossless reduction to 8 bits yields exactly the 8-bit values, which keeps the expected
// bytes below readable.
constexpr int blue = 10;
constexpr int green = 20;
constexpr int red = 30;
constexpr int alpha = 50;
constexpr int gray = 40;
constexpr int scale = 257;

// What a sender publishes for a camera of the given encoding, using the format strings
// compressed_image_transport emits.
struct SourceImage
{
  std::string format;
  int type;
  cv::Scalar pixel;
};

const std::map<std::string, SourceImage> & source_images()
{
  static const std::map<std::string, SourceImage> images = {
    {"mono8", {"mono8; png compressed ", CV_8UC1, cv::Scalar(gray)}},
    {"mono16", {"mono16; png compressed ", CV_16UC1, cv::Scalar(gray * scale)}},
    {"rgb8", {"rgb8; png compressed bgr8", CV_8UC3, cv::Scalar(blue, green, red)}},
    {"bgr8", {"bgr8; png compressed bgr8", CV_8UC3, cv::Scalar(blue, green, red)}},
    {"rgba8", {"rgba8; png compressed bgra8", CV_8UC4, cv::Scalar(blue, green, red, alpha)}},
    {"bgra8", {"bgra8; png compressed bgra8", CV_8UC4, cv::Scalar(blue, green, red, alpha)}},
    {"rgb16",
     {"rgb16; png compressed bgr16", CV_16UC3,
      cv::Scalar(blue * scale, green * scale, red * scale)}},
    {"bgr16",
     {"bgr16; png compressed bgr16", CV_16UC3,
      cv::Scalar(blue * scale, green * scale, red * scale)}},
    {"rgba16",
     {"rgba16; png compressed bgra16", CV_16UC4,
      cv::Scalar(blue * scale, green * scale, red * scale, alpha * scale)}},
    {"bgra16",
     {"bgra16; png compressed bgra16", CV_16UC4,
      cv::Scalar(blue * scale, green * scale, red * scale, alpha * scale)}},
    {"yuv422", {"yuv422; png compressed bgr8", CV_8UC3, cv::Scalar(blue, green, red)}},
    {"bayer_rggb8", {"bayer_rggb8; png compressed ", CV_8UC1, cv::Scalar(gray)}}};
  return images;
}

// One row of the matrix: which camera published, under which "encoding" parameter, and what the
// node publishes today.
struct DecompressorCase
{
  std::string source_encoding;
  std::string parameter_encoding;

  std::string expected_encoding;
  uint32_t expected_step;
  size_t expected_data_size;
  std::vector<uint8_t> expected_first_pixel;
  // Whether step and data size agree with expected_encoding. False means the message is malformed;
  // true does not mean the pixels are the ones the camera produced.
  bool expected_consistent;
};

// Bytes a pixel of the given encoding occupies, as literals so that the assertions do not depend
// on the code under test.
size_t bytes_per_pixel(const std::string & encoding)
{
  static const std::map<std::string, size_t> sizes = {
    {"mono8", 1}, {"mono16", 2}, {"bayer_rggb8", 1}, {"yuv422", 2}, {"rgb8", 3},   {"bgr8", 3},
    {"rgb16", 6}, {"bgr16", 6},  {"rgba8", 4},       {"bgra8", 4},  {"rgba16", 8}, {"bgra16", 8}};
  const auto size = sizes.find(encoding);
  return size == sizes.end() ? 0 : size->second;
}

// A message is only interpretable if its step and data size match its encoding.
bool is_consistent_with_encoding(const sensor_msgs::msg::Image & image)
{
  const size_t pixel_size = bytes_per_pixel(image.encoding);
  return pixel_size != 0 && image.step == image.width * pixel_size &&
         image.data.size() == static_cast<size_t>(image.height) * image.step;
}

// The payload is always PNG so that the expected pixel values stay exact; the codec named in the
// format field is not what selects the decoder.
sensor_msgs::msg::CompressedImage make_compressed_image(
  const std::string & format, const cv::Mat & image)
{
  sensor_msgs::msg::CompressedImage message;
  message.header.frame_id = "camera";
  message.format = format;
  if (!cv::imencode(".png", image, message.data)) {
    throw std::runtime_error("failed to prepare the compressed test input");
  }
  return message;
}

sensor_msgs::msg::CompressedImage make_compressed_image(const std::string & source_encoding)
{
  const auto & source = source_images().at(source_encoding);
  return make_compressed_image(
    source.format, cv::Mat(image_height, image_width, source.type, source.pixel));
}

std::vector<uint8_t> leading_bytes(const sensor_msgs::msg::Image & image, const size_t count)
{
  if (image.data.size() < count) {
    return {};
  }
  return std::vector<uint8_t>(
    image.data.begin(), image.data.begin() + static_cast<std::ptrdiff_t>(count));
}

// The decoded image is always 8-bit with three channels, or four once an alpha channel is added,
// while the published encoding is the one the camera used. Only the 8-bit color cameras come out
// of that unharmed.
const std::vector<DecompressorCase> default_cases = {
  {"rgb8", "default", "rgb8", 12, 24, {red, green, blue}, true},
  {"bgr8", "default", "bgr8", 12, 24, {blue, green, red}, true},
  // KNOWN DEFECT: the alpha channel of the camera is dropped and replaced by 255.
  {"rgba8", "default", "rgba8", 16, 32, {red, green, blue, 255}, true},
  {"bgra8", "default", "bgra8", 16, 32, {blue, green, red, 255}, true},
  // KNOWN DEFECT: a grayscale camera is published under a grayscale encoding while the payload
  // carries three channels.
  {"mono8", "default", "mono8", 12, 24, {gray, gray, gray}, false},
  {"mono16", "default", "mono16", 12, 24, {gray, gray, gray}, false},
  // KNOWN DEFECT: a 16-bit camera is published under a 16-bit encoding while the payload carries
  // 8-bit samples.
  {"rgb16", "default", "rgb16", 12, 24, {red, green, blue}, false},
  {"bgr16", "default", "bgr16", 12, 24, {blue, green, red}, false},
  {"rgba16", "default", "rgba16", 16, 32, {red, green, blue, 255}, false},
  {"bgra16", "default", "bgra16", 16, 32, {blue, green, red, 255}, false},
  // KNOWN DEFECT: an encoding that is neither grayscale nor RGB/BGR is published unchanged while
  // the payload carries three-channel BGR.
  {"yuv422", "default", "yuv422", 12, 24, {blue, green, red}, false},
  {"bayer_rggb8", "default", "bayer_rggb8", 12, 24, {gray, gray, gray}, false},
};

// Requesting rgb8 makes every message consistent, but only a color camera comes through as it was
// sent.
const std::vector<DecompressorCase> rgb8_cases = {
  {"rgb8", "rgb8", "rgb8", 12, 24, {red, green, blue}, true},
  {"bgr8", "rgb8", "rgb8", 12, 24, {red, green, blue}, true},
  // The sender already converted this camera to BGR, so only the channel order is applied.
  {"yuv422", "rgb8", "rgb8", 12, 24, {red, green, blue}, true},
  // KNOWN DEFECT: forcing a color encoding discards what the camera sent. A grayscale image is
  // inflated to three identical channels, a Bayer image likewise instead of being demosaicked, a
  // 16-bit image keeps only its upper 8 bits, and an alpha channel is dropped.
  {"mono8", "rgb8", "rgb8", 12, 24, {gray, gray, gray}, true},
  {"mono16", "rgb8", "rgb8", 12, 24, {gray, gray, gray}, true},
  {"bayer_rggb8", "rgb8", "rgb8", 12, 24, {gray, gray, gray}, true},
  {"rgba8", "rgb8", "rgb8", 12, 24, {red, green, blue}, true},
  {"bgra8", "rgb8", "rgb8", 12, 24, {red, green, blue}, true},
  {"rgb16", "rgb8", "rgb8", 12, 24, {red, green, blue}, true},
  {"bgr16", "rgb8", "rgb8", 12, 24, {red, green, blue}, true},
  {"rgba16", "rgb8", "rgb8", 12, 24, {red, green, blue}, true},
  {"bgra16", "rgb8", "rgb8", 12, 24, {red, green, blue}, true},
};

// Requesting bgr8 behaves like requesting rgb8, with the channels in the opposite order.
const std::vector<DecompressorCase> bgr8_cases = {
  {"rgb8", "bgr8", "bgr8", 12, 24, {blue, green, red}, true},
  {"bgr8", "bgr8", "bgr8", 12, 24, {blue, green, red}, true},
  {"yuv422", "bgr8", "bgr8", 12, 24, {blue, green, red}, true},
  // KNOWN DEFECT: as above, forcing a color encoding discards what the camera sent.
  {"mono8", "bgr8", "bgr8", 12, 24, {gray, gray, gray}, true},
  {"mono16", "bgr8", "bgr8", 12, 24, {gray, gray, gray}, true},
  {"bayer_rggb8", "bgr8", "bgr8", 12, 24, {gray, gray, gray}, true},
  {"rgba8", "bgr8", "bgr8", 12, 24, {blue, green, red}, true},
  {"bgra8", "bgr8", "bgr8", 12, 24, {blue, green, red}, true},
  {"rgb16", "bgr8", "bgr8", 12, 24, {blue, green, red}, true},
  {"bgr16", "bgr8", "bgr8", 12, 24, {blue, green, red}, true},
  {"rgba16", "bgr8", "bgr8", 12, 24, {blue, green, red}, true},
  {"bgra16", "bgr8", "bgr8", 12, 24, {blue, green, red}, true},
};

std::string case_name(const ::testing::TestParamInfo<DecompressorCase> & info)
{
  return info.param.source_encoding;
}
}  // namespace

// Drives the node over real publish/subscribe: the fixture owns the ROS context, the executor
// thread and the test-side pub/sub wiring.
class ImageTransportDecompressorNodeTestBase : public ::testing::Test
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

class ImageTransportDecompressorNodeTest : public ImageTransportDecompressorNodeTestBase,
                                           public ::testing::WithParamInterface<DecompressorCase>
{
};

TEST_P(ImageTransportDecompressorNodeTest, PublishesDecompressedImage)
{
  // Arrange
  const auto & test_case = GetParam();
  start_decompressor_node(test_case.parameter_encoding);

  // Act
  const auto image = publish_and_wait(make_compressed_image(test_case.source_encoding));

  // Assert
  ASSERT_NE(image, nullptr) << "no image was published within the timeout";

  // The header and the geometry of the camera image are preserved.
  EXPECT_EQ(image->header.frame_id, "camera");
  EXPECT_EQ(image->width, static_cast<uint32_t>(image_width));
  EXPECT_EQ(image->height, static_cast<uint32_t>(image_height));

  EXPECT_EQ(image->encoding, test_case.expected_encoding);
  EXPECT_EQ(image->step, test_case.expected_step);
  EXPECT_EQ(image->data.size(), test_case.expected_data_size);
  EXPECT_EQ(
    leading_bytes(*image, test_case.expected_first_pixel.size()), test_case.expected_first_pixel);

  if (test_case.expected_consistent) {
    EXPECT_TRUE(is_consistent_with_encoding(*image));
  } else {
    // KNOWN DEFECT: the published image declares a pixel size that its payload does not have, so
    // consumers cannot interpret it.
    EXPECT_FALSE(is_consistent_with_encoding(*image))
      << "the published image is now consistent with its encoding, so this expectation should be "
         "replaced by EXPECT_TRUE";
  }
}

INSTANTIATE_TEST_SUITE_P(
  EncodingParameterDefault, ImageTransportDecompressorNodeTest, ::testing::ValuesIn(default_cases),
  case_name);
INSTANTIATE_TEST_SUITE_P(
  EncodingParameterRgb8, ImageTransportDecompressorNodeTest, ::testing::ValuesIn(rgb8_cases),
  case_name);
INSTANTIATE_TEST_SUITE_P(
  EncodingParameterBgr8, ImageTransportDecompressorNodeTest, ::testing::ValuesIn(bgr8_cases),
  case_name);

class ImageTransportDecompressorNodeEdgeCaseTest : public ImageTransportDecompressorNodeTestBase
{
};

// KNOWN DEFECT: when the format field does not name the encoding of the camera image, the requested
// encoding is ignored.
TEST_F(ImageTransportDecompressorNodeEdgeCaseTest, IgnoresEncodingParameterWithoutFormatSeparator)
{
  // Arrange
  start_decompressor_node("rgb8");
  const cv::Mat compressed(image_height, image_width, CV_8UC3, cv::Scalar(blue, green, red));

  // Act
  const auto image = publish_and_wait(make_compressed_image("png", compressed));

  // Assert
  ASSERT_NE(image, nullptr) << "no image was published within the timeout";
  EXPECT_NE(image->encoding, "rgb8")
    << "the requested encoding is now honoured, so this expectation should be replaced by "
       "EXPECT_EQ(image->encoding, \"rgb8\")";
  EXPECT_EQ(image->encoding, "bgr8");
  EXPECT_EQ(leading_bytes(*image, 3), std::vector<uint8_t>({blue, green, red}));
  EXPECT_TRUE(is_consistent_with_encoding(*image));
}

// Undecodable payloads are dropped silently: nothing is published and nothing is logged.
TEST_F(ImageTransportDecompressorNodeEdgeCaseTest, DropsUndecodableDataWithoutPublishing)
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

// KNOWN DEFECT: cv_bridge rejects the 16-bit message, so every consumer using it fails per frame.
TEST_F(ImageTransportDecompressorNodeEdgeCaseTest, MalformedSixteenBitOutputIsRejectedByCvBridge)
{
  // Arrange
  start_decompressor_node("default");

  // Act
  const auto image = publish_and_wait(make_compressed_image("rgb16"));

  // Assert
  ASSERT_NE(image, nullptr) << "no image was published within the timeout";
  EXPECT_FALSE(is_consistent_with_encoding(*image));
  EXPECT_THROW(cv_bridge::toCvCopy(*image, "bgr8"), cv_bridge::Exception);
}

// KNOWN DEFECT: cv_bridge accepts the grayscale message, because a step larger than one row is a
// legal row padding, so the corruption is silent: the left third of every row is stretched by
// three.
TEST_F(ImageTransportDecompressorNodeEdgeCaseTest, MalformedGrayscaleOutputIsSilentlyCorrupted)
{
  // Arrange
  start_decompressor_node("default");
  constexpr int gradient_width = 12;
  cv::Mat source(image_height, gradient_width, CV_8UC1);
  for (int row = 0; row < source.rows; ++row) {
    for (int column = 0; column < source.cols; ++column) {
      source.at<uint8_t>(row, column) = static_cast<uint8_t>(column * 20);
    }
  }

  // Act
  const auto image = publish_and_wait(make_compressed_image("mono8; png compressed ", source));

  // Assert
  ASSERT_NE(image, nullptr) << "no image was published within the timeout";
  EXPECT_FALSE(is_consistent_with_encoding(*image));

  cv_bridge::CvImagePtr received;
  ASSERT_NO_THROW(received = cv_bridge::toCvCopy(*image, "mono8"));
  ASSERT_EQ(received->image.cols, gradient_width);
  const std::vector<uint8_t> received_row(
    received->image.ptr<uint8_t>(0), received->image.ptr<uint8_t>(0) + gradient_width);
  const std::vector<uint8_t> source_row(
    source.ptr<uint8_t>(0), source.ptr<uint8_t>(0) + gradient_width);
  EXPECT_NE(received_row, source_row)
    << "the grayscale round trip is no longer corrupted, so this expectation should be replaced by "
       "EXPECT_EQ";
  EXPECT_EQ(received_row, std::vector<uint8_t>({0, 0, 0, 20, 20, 20, 40, 40, 40, 60, 60, 60}));
}
