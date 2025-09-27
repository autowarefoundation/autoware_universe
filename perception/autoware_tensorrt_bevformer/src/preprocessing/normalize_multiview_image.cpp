// Copyright 2025 The Autoware Contributors
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
//
/*
 * Copyright (c) 2025 Multicoreware, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "normalize_multiview_image.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace bevformer
{
namespace preprocessing
{

NormalizeMultiviewImage::NormalizeMultiviewImage(
  const std::vector<float> & mean, const std::vector<float> & std, bool to_rgb)
: to_rgb_(to_rgb)
{
  if (mean.size() != 3 || std.size() != 3) {
    throw std::invalid_argument("Mean and std must have size 3");
  }

  // Copy values to fixed-size array
  std::copy(mean.begin(), mean.end(), mean_.begin());
  std::copy(std.begin(), std.end(), std_.begin());
}

DataDict NormalizeMultiviewImage::operator()(DataDict results)
{
  if (!results.count("img") || !std::holds_alternative<std::vector<cv::Mat>>(results["img"])) {
    std::cerr << "No images found in results" << std::endl;
    return results;
  }

  std::vector<cv::Mat> images = std::get<std::vector<cv::Mat>>(results["img"]);
  std::vector<cv::Mat> normalized_images;
  normalized_images.reserve(images.size());

  for (const auto & img : images) {
    cv::Mat normalized;

    // Convert BGR to RGB if needed
    if (to_rgb_ && img.channels() == 3) {
      cv::cvtColor(img, normalized, cv::COLOR_BGR2RGB);
    } else {
      normalized = img.clone();
    }

    // Ensure float format
    if (normalized.type() != CV_32FC3) {
      normalized.convertTo(normalized, CV_32FC3);
    }

    // Apply normalization directly to each channel
    cv::Mat channels[3];
    cv::split(normalized, channels);

    for (int c = 0; c < 3; ++c) {
      channels[c] = (channels[c] - mean_[c]) / std_[c];
    }

    cv::merge(channels, 3, normalized);
    normalized_images.push_back(normalized);
  }

  results["img"] = normalized_images;
  results.setNestedDict("img_norm_cfg");

  auto & img_norm_cfg = results.getNestedDict("img_norm_cfg");
  img_norm_cfg["mean"] = std::vector<float>(mean_.begin(), mean_.end());
  img_norm_cfg["std"] = std::vector<float>(std_.begin(), std_.end());
  img_norm_cfg["to_rgb"] = to_rgb_;

  return results;
}

std::string NormalizeMultiviewImage::toString() const
{
  return "NormalizeMultiviewImage(mean=[" + std::to_string(mean_[0]) + ", " +
         std::to_string(mean_[1]) + ", " + std::to_string(mean_[2]) + "], std=[" +
         std::to_string(std_[0]) + ", " + std::to_string(std_[1]) + ", " + std::to_string(std_[2]) +
         "], to_rgb=" + std::string(to_rgb_ ? "true" : "false") + ")";
}

}  // namespace preprocessing
}  // namespace bevformer
