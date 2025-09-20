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

// cspell:ignore BEVFORMER

#ifndef PREPROCESSING__AUGMENTATION_TRANSFORMS_HPP_
#define PREPROCESSING__AUGMENTATION_TRANSFORMS_HPP_

#include "data_types.hpp"
#include "transforms.hpp"

#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <vector>

namespace bevformer
{
namespace preprocessing
{

/**
 * @brief Results structure for padding operation
 */
struct Results
{
  std::vector<cv::Mat> img;
  std::vector<cv::Size> ori_shape;
  std::vector<cv::Size> img_shape;
  cv::Size pad_fixed_size{0, 0};
  int pad_size_divisor = 0;
};

/**
 * @brief Results structure for preprocessing operations
 */
struct PreprocessResult
{
  std::vector<cv::Mat> images;
  std::vector<cv::Mat> lidar2img;
  std::vector<cv::Size> img_shape;
};

/**
 * @brief Transform for scaling multi-view images
 */
class RandomScaleImageMultiViewImageTransform : public Transform
{
public:
  explicit RandomScaleImageMultiViewImageTransform(float scale) : scale_(scale) {}

  DataDict operator()(DataDict results) override;
  std::string toString() const override;

private:
  float scale_;  // Single scale factor
};

// Scaling operation helper
PreprocessResult RandomScaleImageMultiViewImage(
  const std::vector<cv::Mat> & input_images, const std::vector<cv::Mat> & input_lidar2img,
  float rand_scale);

/**
 * @brief Transform for padding images to specific dimensions
 */
class PadMultiViewImageTransform : public Transform
{
public:
  explicit PadMultiViewImageTransform(
    const cv::Size * size = nullptr, int size_divisor = 0, int pad_val = 0)
  : size_(size ? std::make_unique<cv::Size>(*size) : nullptr),
    size_divisor_(size_divisor),
    pad_val_(pad_val)
  {
  }

  // Destructor is not needed when using unique_ptr
  ~PadMultiViewImageTransform() override = default;

  DataDict operator()(DataDict results) override;
  std::string toString() const override;

private:
  std::unique_ptr<cv::Size> size_;  // Use smart pointer
  int size_divisor_;
  int pad_val_;
};

// Padding operation helper
void PadMultiViewImages(
  Results & results, const cv::Size * fixed_size, int size_divisor, uchar pad_val = 0);

/**
 * @brief Transform for converting image format from HWC to CHW
 */
class DefaultFormatBundle3DTransform : public Transform
{
public:
  DefaultFormatBundle3DTransform() = default;

  DataDict operator()(DataDict results) override;
  std::string toString() const override;
};

// Format conversion helper
DataDict DefaultFormatBundle3D(const DataDict & results);

}  // namespace preprocessing
}  // namespace bevformer

#endif  // PREPROCESSING__AUGMENTATION_TRANSFORMS_HPP_
