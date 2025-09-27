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

#ifndef PREPROCESSING__TRANSFORMS_HPP_
#define PREPROCESSING__TRANSFORMS_HPP_

#include "data_types.hpp"

#include <memory>
#include <string>
#include <vector>

namespace bevformer
{
namespace preprocessing
{

class Transform
{
public:
  virtual DataDict operator()(DataDict results) = 0;
  virtual ~Transform() = default;
  virtual std::string toString() const = 0;
};

// Compose: Groups transforms together
class Compose
{
public:
  explicit Compose(const std::vector<std::shared_ptr<Transform>> & transforms);
  DataDict operator()(DataDict results) const;
  std::string toString() const;

private:
  std::vector<std::shared_ptr<Transform>> transforms_;
};

class LoadMultiViewImageFromFiles;
class NormalizeMultiviewImage;
class RandomScaleImageMultiViewImageTransform;
class PadMultiViewImageTransform;
class DefaultFormatBundle3DTransform;
class CustomCollect3DTransform;
class MultiScaleFlipAug3D;

}  // namespace preprocessing
}  // namespace bevformer

#endif  // PREPROCESSING__TRANSFORMS_HPP_
