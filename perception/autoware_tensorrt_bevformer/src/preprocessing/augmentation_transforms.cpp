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

#include "augmentation_transforms.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace bevformer
{
namespace preprocessing
{

DataDict RandomScaleImageMultiViewImageTransform::operator()(DataDict results)
{
  std::vector<cv::Mat> images;
  if (!results.count("img") || !std::holds_alternative<std::vector<cv::Mat>>(results["img"])) {
    std::cerr << "Expected vector<cv::Mat> for 'img' key" << std::endl;
    return results;
  }
  images = std::get<std::vector<cv::Mat>>(results["img"]);

  // Get lidar2img matrices if present
  std::vector<cv::Mat> lidar2img;
  if (
    results.count("lidar2img") &&
    std::holds_alternative<std::vector<cv::Mat>>(results["lidar2img"])) {
    lidar2img = std::get<std::vector<cv::Mat>>(results["lidar2img"]);
  } else {
    lidar2img.resize(images.size(), cv::Mat::eye(4, 4, CV_32F));
  }

  // Add scale information
  results["scale_factor"] = scale_;
  results["scale"] = scale_;

  // Apply scaling
  PreprocessResult processed = RandomScaleImageMultiViewImage(images, lidar2img, scale_);

  results["img"] = processed.images;
  results["img_shape"] = processed.img_shape;
  results["lidar2img"] = processed.lidar2img;

  return results;
}

std::string RandomScaleImageMultiViewImageTransform::toString() const
{
  std::string str = "RandomScaleImageMultiViewImage(scale=";
  str += std::to_string(scale_);
  str += ")";
  return str;
}

PreprocessResult RandomScaleImageMultiViewImage(
  const std::vector<cv::Mat> & input_images, const std::vector<cv::Mat> & input_lidar2img,
  float rand_scale)
{
  PreprocessResult result;

  cv::Mat scale_factor = cv::Mat::eye(4, 4, CV_32F);

  scale_factor.at<float>(0, 0) *= rand_scale;
  scale_factor.at<float>(1, 1) *= rand_scale;

  for (size_t i = 0; i < input_images.size(); ++i) {
    const auto & img = input_images[i];

    int new_width = static_cast<int>(img.cols * rand_scale);
    int new_height = static_cast<int>(img.rows * rand_scale);

    cv::Mat resized_img;
    cv::resize(img, resized_img, cv::Size(new_width, new_height));

    result.images.push_back(resized_img);

    result.img_shape.push_back(resized_img.size());

    // Multiply the lidar-to-image matrix with the scale matrix if available
    if (i < input_lidar2img.size()) {
      cv::Mat transformed = scale_factor * input_lidar2img[i];
      result.lidar2img.push_back(transformed);
    }
  }

  return result;
}

DataDict PadMultiViewImageTransform::operator()(DataDict results)
{
  // Convert to the Results struct expected by PadMultiViewImages
  Results padResults;

  if (std::holds_alternative<std::vector<cv::Mat>>(results["img"])) {
    padResults.img = std::get<std::vector<cv::Mat>>(results["img"]);
  } else {
    std::cerr << "Expected vector<cv::Mat> for 'img' key" << std::endl;
    return results;
  }

  // Apply padding
  PadMultiViewImages(padResults, size_.get(), size_divisor_, pad_val_);

  results["img"] = padResults.img;
  results["ori_shape"] = padResults.ori_shape;
  results["img_shape"] = padResults.img_shape;

  // Store the width and height as separate integers
  if (padResults.pad_fixed_size.width > 0 || padResults.pad_fixed_size.height > 0) {
    results["pad_fixed_size_width"] = padResults.pad_fixed_size.width;
    results["pad_fixed_size_height"] = padResults.pad_fixed_size.height;
  }
  results["pad_size_divisor"] = padResults.pad_size_divisor;

  return results;
}

std::string PadMultiViewImageTransform::toString() const
{
  std::string str = "PadMultiViewImage(";
  if (size_) {
    str += "size=(" + std::to_string(size_->width) + ", " + std::to_string(size_->height) + "), ";
  } else {
    str += "size=None, ";
  }
  str += "size_divisor=" + std::to_string(size_divisor_) + ", ";
  str += "pad_val=" + std::to_string(pad_val_) + ")";
  return str;
}

void PadMultiViewImages(
  Results & results, const cv::Size * fixed_size, int size_divisor, uchar pad_val)
{
  // Ensure that only one padding method is selected (fixed or divisible)
  assert(
    (fixed_size != nullptr && size_divisor == 0) || (fixed_size == nullptr && size_divisor > 0));

  // Clear previous metadata in case function is reused
  results.ori_shape.clear();
  results.img_shape.clear();
  std::vector<cv::Mat> padded_images;

  for (const auto & img : results.img) {
    // Store original image size (width, height)
    results.ori_shape.push_back(cv::Size(img.cols, img.rows));

    cv::Mat padded;
    int bottom = 0, right = 0;

    if (fixed_size) {
      // Calculate how much padding is needed on the bottom and right
      bottom = std::max(0, fixed_size->height - img.rows);
      right = std::max(0, fixed_size->width - img.cols);

      // Add padding to bottom and right with constant value (pad_val)
      cv::copyMakeBorder(
        img, padded, 0, bottom, 0, right, cv::BORDER_CONSTANT,
        cv::Scalar(pad_val, pad_val, pad_val));

      results.pad_fixed_size = *fixed_size;
      results.pad_size_divisor = 0;
    } else {
      // Compute nearest size that is divisible by size_divisor
      int new_h = (img.rows + size_divisor - 1) / size_divisor * size_divisor;
      int new_w = (img.cols + size_divisor - 1) / size_divisor * size_divisor;

      // Calculate needed padding
      bottom = new_h - img.rows;
      right = new_w - img.cols;

      // Add padding
      cv::copyMakeBorder(
        img, padded, 0, bottom, 0, right, cv::BORDER_CONSTANT,
        cv::Scalar(pad_val, pad_val, pad_val));

      results.pad_fixed_size = cv::Size(0, 0);
      results.pad_size_divisor = size_divisor;
    }

    results.img_shape.push_back(cv::Size(padded.cols, padded.rows));

    // Add padded image to the output list
    padded_images.push_back(padded);
  }

  results.img = padded_images;
}

DataDict DefaultFormatBundle3DTransform::operator()(DataDict results)
{
  return DefaultFormatBundle3D(results);
}

std::string DefaultFormatBundle3DTransform::toString() const
{
  return "DefaultFormatBundle3D()";
}

DataDict DefaultFormatBundle3D(const DataDict & results)
{
  DataDict formatted_results = results;

  if (
    formatted_results.count("img") &&
    std::holds_alternative<std::vector<cv::Mat>>(formatted_results["img"])) {
    std::vector<cv::Mat> images = std::get<std::vector<cv::Mat>>(formatted_results["img"]);

    if (!images.empty()) {
      std::vector<cv::Mat> transposed_images;

      for (auto & img : images) {
        if (img.type() != CV_32FC3) {
          RCLCPP_ERROR(
            rclcpp::get_logger("bevformer_preprocessing"), "Image must be in CV_32FC3 format");
          return formatted_results;
        }

        int height = img.rows;
        int width = img.cols;
        int channels = img.channels();

        std::vector<cv::Mat> channels_vec;
        cv::split(img, channels_vec);

        // Create CHW matrix with proper dimensions
        cv::Mat transposed(channels, height * width, CV_32F);

        // Copy each channel's data into the correct position
        for (int c = 0; c < channels; c++) {
          const float * src = channels_vec[c].ptr<float>();
          float * dst = transposed.ptr<float>(c);
          std::memcpy(dst, src, height * width * sizeof(float));
        }

        transposed = transposed.reshape(1, {channels, height, width});

        // Ensure memory is contiguous
        transposed = transposed.clone();

        transposed_images.push_back(transposed);
      }

      formatted_results["img"] = transposed_images;
    }
  }
  return formatted_results;
}

}  // namespace preprocessing
}  // namespace bevformer
