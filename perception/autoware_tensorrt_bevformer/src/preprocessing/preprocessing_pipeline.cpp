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

#include "preprocessing_pipeline.hpp"

#include <rclcpp/logging.hpp>

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

namespace bevformer
{
namespace preprocessing
{

BEVPreprocessingPipeline::BEVPreprocessingPipeline(
  const std::vector<float> & img_mean, const std::vector<float> & img_std, bool to_rgb,
  int pad_divisor, float scale_factor)
{
  try {
    normalize_multiview_images_ =
      std::make_shared<NormalizeMultiviewImage>(img_mean, img_std, to_rgb);

    std::vector<std::shared_ptr<Transform>> transforms;

    auto random_scale = std::make_shared<RandomScaleImageMultiViewImageTransform>(scale_factor);
    transforms.push_back(random_scale);

    auto pad_img = std::make_shared<PadMultiViewImageTransform>(nullptr, pad_divisor);
    transforms.push_back(pad_img);

    auto format_bundle = std::make_shared<DefaultFormatBundle3DTransform>();
    transforms.push_back(format_bundle);

    multi_scale_flip_aug_ = std::make_shared<MultiScaleFlipAug3D>(transforms);
  } catch (const std::bad_alloc & e) {
    std::cerr << "Memory allocation failed in BEVPreprocessingPipeline: " << e.what() << std::endl;
    throw;
  } catch (const std::exception & e) {
    std::cerr << "Exception in BEVPreprocessingPipeline: " << e.what() << std::endl;
    throw;
  }
}

DataDict BEVPreprocessingPipeline::processImages(
  const std::vector<cv::Mat> & images, const std::vector<cv::Mat> & lidar2img_matrices,
  const DataDict & metadata)
{
  DataDict results;

  if (images.size() != 6) {
    std::cerr << "Warning: Expected 6 images, received " << images.size() << std::endl;
  }

  std::vector<cv::Mat> image_copies;
  for (const auto & img : images) {
    cv::Mat float_img;
    if (img.type() != CV_32FC3) {
      img.convertTo(float_img, CV_32FC3);
    } else {
      float_img = img.clone();
    }
    image_copies.push_back(float_img);
  }
  results["img"] = image_copies;

  // Store lidar2img matrices directly from the parameters
  results["lidar2img"] = lidar2img_matrices;

  // Copy only metadata of normalization
  if (metadata.count("mean")) {
    results["mean"] = metadata.at("mean");
  }
  if (metadata.count("std")) {
    results["std"] = metadata.at("std");
  }
  if (metadata.count("to_rgb")) {
    results["to_rgb"] = metadata.at("to_rgb");
  }

  try {
    // Apply normalization and augmentation
    results = (*normalize_multiview_images_)(results);
    results = (*multi_scale_flip_aug_)(results);
  } catch (const std::exception & e) {
    std::cerr << "Error (Image Preprocessing pipeline) : " << e.what() << std::endl;
  }

  return results;
}

}  // namespace preprocessing
}  // namespace bevformer
