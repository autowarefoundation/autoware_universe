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

#include "grid_pixel_sampler.hpp"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <vector>

namespace autoware::image_object_locator::grid_pixel_sampler
{
GridPixelSamplerBase::GridPixelSamplerBase(int half_grid_size) : half_grid_size_(half_grid_size)
{
  grid_points_num_ = (2 * half_grid_size_ + 1) * (2 * half_grid_size_ + 1);
}

std::vector<cv::Point2f> GridPixelSamplerBase::samplePoints(
  const std::vector<cv::Point2f> & sampling_base_points, const float roi_w, const float roi_h)
{
  std::vector<cv::Point2f> all_sampling_points;

  for (const cv::Point2f & grid_center_pixel : sampling_base_points) {
    std::vector<cv::Point2f> sampled_points = samplePointsImpl(grid_center_pixel, roi_w, roi_h);

    all_sampling_points.reserve(all_sampling_points.size() + sampled_points.size());
    all_sampling_points.insert(
      all_sampling_points.end(), sampled_points.begin(), sampled_points.end());
  }

  return all_sampling_points;
}

////////////////////////////////
/// FixedGridPixelSampler
////////////////////////////////

// sample points with pre-defined grid
std::vector<cv::Point2f> FixedGridPixelSampler::samplePointsImpl(
  const cv::Point2f & center_px, [[maybe_unused]] const float roi_w,
  [[maybe_unused]] const float roi_h)
{
  // Sampled points form a square grid:
  //   half_grid_size = 3
  //        /
  //   |-------|       |-------|
  //   3   2   1   0   1   2   3
  //   .   .   .   .   .   .   .
  //   .   .   .   .   .   .   .
  //   ...
  //   .   .   .   .   .   .   .
  //   .   .   .   .   .   .   .
  //   |---|---|---|---|---|---|
  //        /
  //   grid_cell_size

  std::vector<cv::Point2f> sampled_points;
  sampled_points.reserve(grid_points_num_);

  for (int j = -half_grid_size_; j <= half_grid_size_; ++j) {
    for (int i = -half_grid_size_; i <= half_grid_size_; ++i) {
      sampled_points.emplace_back(
        center_px.x + i * grid_cell_size_, center_px.y + j * grid_cell_size_);
    }
  }

  return sampled_points;
}

////////////////////////////////
/// AdaptiveGridPixelSampler
////////////////////////////////

// sample multiple points with adaptive grid sice based on ROI
std::vector<cv::Point2f> AdaptiveGridPixelSampler::samplePointsImpl(
  const cv::Point2f & center_px, const float roi_w, const float roi_h)
{
  // Sampled points form a rectangular grid:
  //   half_grid_size = 3
  //        /
  //   |-------|       |-------|
  //   3   2   1   0   1   2   3
  //   .   .   .   .   .   .   .
  //
  //   .   .   .   .   .   .   .
  //   ...
  //   .   .   .   .   .   .   . ---
  //                              |  -- grid_cell_size_h = roi_h * bbox_fraction
  //   .   .   .   .   .   .   . ---
  //   |---|---|---|---|---|---|
  //        /
  //   grid_cell_size_w = roi_w * bbox_fraction
  //

  constexpr float min_cell_size = 1.0f;
  const float grid_cell_size_w = std::max(min_cell_size, roi_w * bbox_fraction_);
  const float grid_cell_size_h = std::max(min_cell_size, roi_h * bbox_fraction_);

  std::vector<cv::Point2f> sampled_points;
  sampled_points.reserve(grid_points_num_);

  for (int j = -half_grid_size_; j <= half_grid_size_; ++j) {
    for (int i = -half_grid_size_; i <= half_grid_size_; ++i) {
      sampled_points.emplace_back(
        center_px.x + i * grid_cell_size_w, center_px.y + j * grid_cell_size_h);
    }
  }

  return sampled_points;
}

}  // namespace autoware::image_object_locator::grid_pixel_sampler
