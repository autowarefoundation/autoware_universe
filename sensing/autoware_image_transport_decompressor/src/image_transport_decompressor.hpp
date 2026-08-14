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

#ifndef IMAGE_TRANSPORT_DECOMPRESSOR_HPP_
#define IMAGE_TRANSPORT_DECOMPRESSOR_HPP_

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <optional>
#include <string>

namespace autoware::image_preprocessor::image_transport_decompressor
{

/// @brief Severity of DecompressResult::message.
enum class Severity { Ok, Warn, Error };

/// @brief Outcome of decompress(). The caller decides whether/how to report message; severity is Ok
/// exactly when message is empty. image is set independently of severity.
struct DecompressResult
{
  std::optional<sensor_msgs::msg::Image> image;
  Severity severity{Severity::Ok};
  std::string message;
};

/// @brief Decompress @p compressed_image, copying its header into the result's image. "rgb8" and
/// "bgr8" force that encoding; any other requested value keeps the encoding the format field names.
DecompressResult decompress(
  const sensor_msgs::msg::CompressedImage & compressed_image,
  const std::string & requested_encoding);

}  // namespace autoware::image_preprocessor::image_transport_decompressor

#endif  // IMAGE_TRANSPORT_DECOMPRESSOR_HPP_
