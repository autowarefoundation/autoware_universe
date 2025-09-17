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

#ifndef PREPROCESSING__MULTI_SCALE_FLIP_AUG_3D_HPP_
#define PREPROCESSING__MULTI_SCALE_FLIP_AUG_3D_HPP_

#include "transforms.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace bevformer
{
namespace preprocessing
{

class MultiScaleFlipAug3D
{
public:
  /**
   * @brief Constructor for image transformation pipeline
   * @param transforms Vector of transforms to be applied in sequence
   * @param img_scale Target image scale (width, height)
   */
  explicit MultiScaleFlipAug3D(const std::vector<std::shared_ptr<Transform>> & transforms);

  /**
   * @brief Apply the transformation pipeline to input data
   * @param results Input data dictionary containing images and metadata
   * @return Transformed data dictionary
   */
  DataDict operator()(DataDict results);

  /**
   * @brief Get string representation of the transform
   */
  std::string toString() const;

private:
  std::shared_ptr<Compose> compose_;
};

}  // namespace preprocessing
}  // namespace bevformer

#endif  // PREPROCESSING__MULTI_SCALE_FLIP_AUG_3D_HPP_
