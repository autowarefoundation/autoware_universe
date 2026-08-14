// Copyright 2020 Tier IV, Inc.
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

#include "autoware/image_transport_decompressor/image_transport_decompressor_node.hpp"

#include "image_transport_decompressor.hpp"

#include <memory>
#include <string>
#include <utility>

namespace autoware::image_preprocessor
{
ImageTransportDecompressor::ImageTransportDecompressor(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("image_transport_decompressor", node_options),
  encoding_(declare_parameter<std::string>("encoding"))
{
  compressed_image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    "~/input/compressed_image", rclcpp::SensorDataQoS(),
    std::bind(&ImageTransportDecompressor::onCompressedImage, this, std::placeholders::_1));
  raw_image_pub_ =
    create_publisher<sensor_msgs::msg::Image>("~/output/raw_image", rclcpp::SensorDataQoS());
}

void ImageTransportDecompressor::onCompressedImage(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr input_compressed_image_msg)
{
  auto result = image_transport_decompressor::decompress(*input_compressed_image_msg, encoding_);
  if (result.severity == image_transport_decompressor::Severity::Error) {
    RCLCPP_ERROR(get_logger(), "%s", result.message.c_str());
  }
  if (!result.image) {
    return;
  }

  raw_image_pub_->publish(std::make_unique<sensor_msgs::msg::Image>(std::move(*result.image)));
}

}  // namespace autoware::image_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::image_preprocessor::ImageTransportDecompressor)
