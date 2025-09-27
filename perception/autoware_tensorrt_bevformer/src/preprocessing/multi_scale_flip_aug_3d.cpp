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

#include "multi_scale_flip_aug_3d.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace bevformer
{
namespace preprocessing
{

MultiScaleFlipAug3D::MultiScaleFlipAug3D(const std::vector<std::shared_ptr<Transform>> & transforms)
{
  compose_ = std::make_shared<Compose>(transforms);
}

DataDict MultiScaleFlipAug3D::operator()(DataDict results)
{
  // Create a copy to avoid modifying the original
  DataDict augmented_results = results;

  if (compose_) {
    augmented_results = (*compose_)(augmented_results);
  }

  return augmented_results;
}

std::string MultiScaleFlipAug3D::toString() const
{
  return "MultiScaleFlipAug3D(transforms=" + (compose_ ? compose_->toString() : "none") + ")";
}

}  // namespace preprocessing
}  // namespace bevformer
