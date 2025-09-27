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

#ifndef PREPROCESSING__PREPROCESSING_PIPELINE_HPP_
#define PREPROCESSING__PREPROCESSING_PIPELINE_HPP_

#include "augmentation_transforms.hpp"
#include "multi_scale_flip_aug_3d.hpp"
#include "normalize_multiview_image.hpp"
#include "transforms.hpp"

#include <memory>
#include <vector>

namespace bevformer
{
namespace preprocessing
{

class BEVPreprocessingPipeline
{
public:
  /**
   * Constructor builds the preprocessing pipeline
   * @param img_mean Image mean values for normalization
   * @param img_std Image standard deviation values for normalization
   * @param to_rgb Whether to convert BGR to RGB
   */
  BEVPreprocessingPipeline(
    const std::vector<float> & img_mean, const std::vector<float> & img_std, bool to_rgb,
    int pad_divisor, float scale_factor);

  /**
   * Process images through the pipeline
   * @param images Input images (expected 6 cameras)
   * @param lidar2img_matrices Lidar to image transformation matrices
   * @param metadata Normalization parameters
   */
  DataDict processImages(
    const std::vector<cv::Mat> & images, const std::vector<cv::Mat> & lidar2img_matrices,
    const DataDict & metadata = {});

private:
  std::shared_ptr<NormalizeMultiviewImage> normalize_multiview_images_;
  std::shared_ptr<MultiScaleFlipAug3D> multi_scale_flip_aug_;
};

}  // namespace preprocessing
}  // namespace bevformer

#endif  // PREPROCESSING__PREPROCESSING_PIPELINE_HPP_
