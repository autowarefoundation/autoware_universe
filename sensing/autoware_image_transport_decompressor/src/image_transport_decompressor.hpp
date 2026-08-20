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

#include <string>

namespace autoware::image_preprocessor::image_transport_decompressor
{

/// @brief Decompress @p compressed_image, copying its header into the returned image. "rgb8" and
/// "bgr8" force that encoding, any other requested one keeps the encoding the format field names.
/// @throws std::runtime_error when the payload cannot be decoded. A sensor sits at the far end of a
/// physical link, so that is an expected event rather than a defect of this function, and the
/// caller owns the policy for it.
sensor_msgs::msg::Image decompress(
  const sensor_msgs::msg::CompressedImage & compressed_image,
  const std::string & requested_encoding);

}  // namespace autoware::image_preprocessor::image_transport_decompressor

#endif  // IMAGE_TRANSPORT_DECOMPRESSOR_HPP_
