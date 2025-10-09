// Copyright 2025 TIER IV, Inc.
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

#ifndef PERCEPTION__AUTOWARE_IMAGE_OBJECT_LOCATOR__LIB__SAMPLER__GRID_PIXEL_SAMPLER_HPP_
#define PERCEPTION__AUTOWARE_IMAGE_OBJECT_LOCATOR__LIB__SAMPLER__GRID_PIXEL_SAMPLER_HPP_

#include <opencv2/opencv.hpp>

#include <vector>

namespace autoware::image_object_locator::grid_pixel_sampler
{
class GridPixelSamplerBase
{
private:
public:
  int half_grid_size_;
  int grid_points_num_;

  GridPixelSamplerBase() : GridPixelSamplerBase(3) {}
  explicit GridPixelSamplerBase(int half_grid_size);

  virtual std::vector<cv::Point2f> samplePointsImpl(
    const cv::Point2f & center_px, [[maybe_unused]] const float roi_w,
    [[maybe_unused]] const float roi_h) = 0;
  std::vector<cv::Point2f> samplePoints(
    const std::vector<cv::Point2f> & sampling_base_points, const float roi_w,
    [[maybe_unused]] const float roi_h);
};

class FixedGridPixelSampler : public GridPixelSamplerBase
{
private:
  float grid_cell_size_;

public:
  FixedGridPixelSampler() : GridPixelSamplerBase(3), grid_cell_size_(10.0) {}
  explicit FixedGridPixelSampler(int half_grid_size, float grid_cell_size)
  : GridPixelSamplerBase(half_grid_size), grid_cell_size_(grid_cell_size)
  {
  }

  std::vector<cv::Point2f> samplePointsImpl(
    const cv::Point2f & center_px, [[maybe_unused]] const float roi_w,
    [[maybe_unused]] const float roi_h);
  std::vector<cv::Point2f> samplePoints(const std::vector<cv::Point2f> & sampling_base_points);
};

class AdaptiveGridPixelSampler : public GridPixelSamplerBase
{
private:
  float bbox_fraction_;

public:
  AdaptiveGridPixelSampler() : GridPixelSamplerBase(3), bbox_fraction_(0.01) {}
  explicit AdaptiveGridPixelSampler(int half_grid_size, float bbox_fraction)
  : GridPixelSamplerBase(half_grid_size), bbox_fraction_(bbox_fraction)
  {
  }

  std::vector<cv::Point2f> samplePointsImpl(
    const cv::Point2f & center_px, const float roi_w, const float roi_h);
  std::vector<cv::Point2f> samplePoints(
    const std::vector<cv::Point2f> & sampling_base_points, const float roi_w, const float roi_h);
};

}  // namespace autoware::image_object_locator::grid_pixel_sampler

#endif  // PERCEPTION__AUTOWARE_IMAGE_OBJECT_LOCATOR__LIB__SAMPLER__GRID_PIXEL_SAMPLER_HPP_
