// Copyright 2026 TIER IV
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

#ifndef AUTOWARE__BEVFUSION__CAMERA__CAMERA_PREPROCESS_HPP_
#define AUTOWARE__BEVFUSION__CAMERA__CAMERA_PREPROCESS_HPP_

#include "autoware/bevfusion/utils.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <cstdint>

namespace autoware::bevfusion
{

class CameraPreprocess
{
public:
  CameraPreprocess(cudaStream_t stream, const int camera_id);

  cudaError_t resizeAndExtractRoi_launch(
    const std::uint8_t * input_img, std::uint8_t * output_img, int H,
    int W,                    // Original image dimensions
    int H2, int W2,           // Resized image dimensions
    int H3, int W3,           // ROI dimensions
    int y_start, int x_start  // ROI top-left coordinates in resized image
  );
  cudaError_t remap_launch(
    const std::uint8_t * input_img, std::uint8_t * output_img, int output_height,
    int output_width,                   // Output (destination) image dimensions
    int input_height, int input_width,  // Input (source) image dimensions
    const float * map_x, const float * map_y);

private:
  cudaStream_t stream_;
  int camera_id_;
};
}  // namespace autoware::bevfusion
#endif  // AUTOWARE__BEVFUSION__CAMERA__CAMERA_PREPROCESS_HPP_
