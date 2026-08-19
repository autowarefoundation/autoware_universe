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

#include <string>
#include <utility>

namespace autoware::image_preprocessor::image_transport_decompressor
{

DecompressResult decompress(
  const sensor_msgs::msg::CompressedImage & compressed_image,
  const std::string & requested_encoding)
{
  DecompressResult result;

  cv_bridge::CvImage cv_image;
  // Copy message header
  cv_image.header = compressed_image.header;

  // Decode the image. cv::IMREAD_COLOR makes it 8-bit with three channels in BGR order, whatever
  // the depth and the channel count of the compressed stream are.
  try {
    cv_image.image = cv::imdecode(cv::Mat(compressed_image.data), cv::IMREAD_COLOR);

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

      if (sensor_msgs::image_encodings::isColor(image_encoding)) {
        std::string compressed_encoding = compressed_image.format.substr(split_pos);
        bool compressed_bgr_image =
          (compressed_encoding.find("compressed bgr") != std::string::npos);

        // Revert color transformation
        if (compressed_bgr_image) {
          // if necessary convert colors from bgr to rgb
          if (
            (image_encoding == sensor_msgs::image_encodings::RGB8) ||
            (image_encoding == sensor_msgs::image_encodings::RGB16)) {
            cv::cvtColor(cv_image.image, cv_image.image, cv::COLOR_BGR2RGB);
          }

          if (
            (image_encoding == sensor_msgs::image_encodings::RGBA8) ||
            (image_encoding == sensor_msgs::image_encodings::RGBA16)) {
            cv::cvtColor(cv_image.image, cv_image.image, cv::COLOR_BGR2RGBA);
          }

          if (
            (image_encoding == sensor_msgs::image_encodings::BGRA8) ||
            (image_encoding == sensor_msgs::image_encodings::BGRA16)) {
            cv::cvtColor(cv_image.image, cv_image.image, cv::COLOR_BGR2BGRA);
          }
        } else {
          // if necessary convert colors from rgb to bgr
          if (
            (image_encoding == sensor_msgs::image_encodings::BGR8) ||
            (image_encoding == sensor_msgs::image_encodings::BGR16)) {
            cv::cvtColor(cv_image.image, cv_image.image, cv::COLOR_RGB2BGR);
          }

          if (
            (image_encoding == sensor_msgs::image_encodings::BGRA8) ||
            (image_encoding == sensor_msgs::image_encodings::BGRA16)) {
            cv::cvtColor(cv_image.image, cv_image.image, cv::COLOR_RGB2BGRA);
          }

          if (
            (image_encoding == sensor_msgs::image_encodings::RGBA8) ||
            (image_encoding == sensor_msgs::image_encodings::RGBA16)) {
            cv::cvtColor(cv_image.image, cv_image.image, cv::COLOR_RGB2RGBA);
          }
        }
      }
    }
  } catch (const cv::Exception & e) {
    result.severity = Severity::Error;
    result.message = e.what();
  }

  if ((cv_image.image.rows == 0) || (cv_image.image.cols == 0)) {
    return result;
  }

  sensor_msgs::msg::Image image_msg;
  cv_image.toImageMsg(image_msg);
  result.image = std::move(image_msg);
  return result;
}

}  // namespace autoware::image_preprocessor::image_transport_decompressor
