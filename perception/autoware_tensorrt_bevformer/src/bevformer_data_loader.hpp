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

#ifndef BEVFORMER_DATA_LOADER_HPP_
#define BEVFORMER_DATA_LOADER_HPP_

#include <opencv2/core.hpp>

#include <vector>

namespace autoware
{
namespace tensorrt_bevformer
{

class BEVFormerDataLoader
{
public:
  BEVFormerDataLoader();
  ~BEVFormerDataLoader() = default;

  // Creates a flat tensor from 6 images in CHW format, shape [1, 6*3*H*W]
  cv::Mat createImageTensor(const std::vector<cv::Mat> & images);
};

}  // namespace tensorrt_bevformer
}  // namespace autoware

#endif  // BEVFORMER_DATA_LOADER_HPP_
