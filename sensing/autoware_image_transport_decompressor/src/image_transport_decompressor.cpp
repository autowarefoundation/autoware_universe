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

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "image_transport_decompressor.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/image_encodings.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::image_preprocessor::image_transport_decompressor
{

void revert_color_transformation(
  cv_bridge::CvImage & cv_image, const std::string & compressed_encoding);

sensor_msgs::msg::Image decompress(
  const sensor_msgs::msg::CompressedImage & compressed_image,
  const std::string & requested_encoding)
{
  cv_bridge::CvImage cv_image;
  // Copy message header
  cv_image.header = compressed_image.header;

  // Decode the image. cv::IMREAD_COLOR makes it 8-bit with three channels in BGR order, whatever
  // the depth and the channel count of the compressed stream are.
  try {
    cv_image.image = cv::imdecode(cv::Mat(compressed_image.data), cv::IMREAD_COLOR);
  } catch (const cv::Exception & e) {
    // Reported the same way as the empty image below, keeping the decoder's own message.
    throw std::runtime_error(e.what());
  }
  if (cv_image.image.empty()) {
    // cv::imdecode throws on an empty payload, but reports one it cannot make sense of by returning
    // an empty image, so raise that second case here.
    throw std::runtime_error("the compressed image could not be decoded");
  }

  // Assign image encoding string
  const size_t split_pos = compressed_image.format.find(';');
  if (split_pos == std::string::npos) {
    // Older version of compressed_image_transport does not signal image format
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
  } else {
    std::string image_encoding;
    if (requested_encoding == std::string("rgb8")) {
      image_encoding = "rgb8";
    } else if (requested_encoding == std::string("bgr8")) {
      image_encoding = "bgr8";
    } else {
      // default encoding
      image_encoding = compressed_image.format.substr(0, split_pos);
    }

    cv_image.encoding = image_encoding;

    revert_color_transformation(cv_image, compressed_image.format.substr(split_pos));
  }

  sensor_msgs::msg::Image output;
  cv_image.toImageMsg(output);
  return output;
}

// Conversion that undoes the color transformation the sender applied, per encoding to publish
// under. There are two tables because the conversion depends on the channel order the sender
// compressed in. An encoding absent from a table, "mono8" or "yuv422" for instance, needs no
// conversion.
using ColorConversions = std::array<std::pair<const char *, cv::ColorConversionCodes>, 6>;

constexpr ColorConversions conversions_from_bgr{
  {{sensor_msgs::image_encodings::RGB8, cv::COLOR_BGR2RGB},
   {sensor_msgs::image_encodings::RGB16, cv::COLOR_BGR2RGB},
   {sensor_msgs::image_encodings::RGBA8, cv::COLOR_BGR2RGBA},
   {sensor_msgs::image_encodings::RGBA16, cv::COLOR_BGR2RGBA},
   {sensor_msgs::image_encodings::BGRA8, cv::COLOR_BGR2BGRA},
   {sensor_msgs::image_encodings::BGRA16, cv::COLOR_BGR2BGRA}}};

constexpr ColorConversions conversions_from_rgb{
  {{sensor_msgs::image_encodings::BGR8, cv::COLOR_RGB2BGR},
   {sensor_msgs::image_encodings::BGR16, cv::COLOR_RGB2BGR},
   {sensor_msgs::image_encodings::BGRA8, cv::COLOR_RGB2BGRA},
   {sensor_msgs::image_encodings::BGRA16, cv::COLOR_RGB2BGRA},
   {sensor_msgs::image_encodings::RGBA8, cv::COLOR_RGB2RGBA},
   {sensor_msgs::image_encodings::RGBA16, cv::COLOR_RGB2RGBA}}};

// Undo the color transformation the sender applied before compressing, so that the channels end up
// in the order cv_image.encoding promises. @p compressed_encoding is the part of the format field
// from the ';' onwards, which names the order the sender compressed in.
void revert_color_transformation(
  cv_bridge::CvImage & cv_image, const std::string & compressed_encoding)
{
  const bool compressed_bgr_image = compressed_encoding.find("compressed bgr") != std::string::npos;
  const ColorConversions & conversions =
    compressed_bgr_image ? conversions_from_bgr : conversions_from_rgb;

  const auto conversion = std::find_if(
    conversions.begin(), conversions.end(),
    [&cv_image](const auto & entry) { return cv_image.encoding == entry.first; });

  if (conversion != conversions.end()) {
    cv::cvtColor(cv_image.image, cv_image.image, conversion->second);
  }
}

}  // namespace autoware::image_preprocessor::image_transport_decompressor
