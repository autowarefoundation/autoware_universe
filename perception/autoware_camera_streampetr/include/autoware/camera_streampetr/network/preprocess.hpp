// Copyright 2025 TIER IV
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

#ifndef AUTOWARE__CAMERA_STREAMPETR__NETWORK__PREPROCESS_HPP_
#define AUTOWARE__CAMERA_STREAMPETR__NETWORK__PREPROCESS_HPP_

#include <cstdint>

namespace autoware::camera_streampetr
{
cudaError_t resizeAndExtractRoi_launch(
  const std::uint8_t * input_img, float * output_img,
  int camera_offset,         // Camera offset in the input image
  int H, int W,              // Original image dimensions
  int H2, int W2,            // Resized image dimensions
  int H3, int W3,            // ROI dimensions
  int y_start, int x_start,  // ROI top-left coordinates in resized image
  const float * channel_wise_mean, const float * channel_wise_std, cudaStream_t stream);
}  // namespace autoware::camera_streampetr

#endif  // AUTOWARE__CAMERA_STREAMPETR__NETWORK__PREPROCESS_HPP_
