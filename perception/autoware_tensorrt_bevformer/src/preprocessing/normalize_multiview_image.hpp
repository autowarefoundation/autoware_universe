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

#ifndef PREPROCESSING__NORMALIZE_MULTIVIEW_IMAGE_HPP_
#define PREPROCESSING__NORMALIZE_MULTIVIEW_IMAGE_HPP_

#include "transforms.hpp"

#include <array>
#include <string>
#include <vector>

namespace bevformer
{
namespace preprocessing
{

class NormalizeMultiviewImage : public Transform
{
public:
  /**
   * @brief Constructor with normalization parameters
   * @param mean Mean values for RGB channels
   * @param std Standard deviation values for RGB channels
   * @param to_rgb Whether to convert BGR to RGB
   * @throws std::invalid_argument if mean or std don't have exactly 3 elements
   */
  NormalizeMultiviewImage(
    const std::vector<float> & mean, const std::vector<float> & std, bool to_rgb = false);

  /**
   * @brief Normalizes images in the results dictionary
   * @param results Dictionary containing images to normalize
   * @return Updated dictionary with normalized images and normalization config
   */
  DataDict operator()(DataDict results) override;
  std::string toString() const override;

private:
  std::array<float, 3> mean_;  // RGB mean values
  std::array<float, 3> std_;   // RGB std values
  bool to_rgb_;
};

}  // namespace preprocessing
}  // namespace bevformer

#endif  // PREPROCESSING__NORMALIZE_MULTIVIEW_IMAGE_HPP_
