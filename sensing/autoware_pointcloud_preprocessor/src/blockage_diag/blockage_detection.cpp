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

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_detection.hpp"

#include <opencv2/imgproc.hpp>

#include <string>
#include <utility>

namespace autoware::pointcloud_preprocessor
{

BlockageDetector::BlockageDetector(const BlockageDetectionConfig & config) : config_(config)
{
}

cv::Mat BlockageDetector::make_blockage_mask(const cv::Mat & no_return_mask) const
{
  assert(no_return_mask.type() == CV_8UC1);
  auto dimensions = no_return_mask.size();

  int kernel_size = 2 * config_.blockage_kernel + 1;
  int kernel_center = config_.blockage_kernel;
  cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(kernel_size, kernel_size), cv::Point(kernel_center, kernel_center));

  cv::Mat erosion_result(dimensions, CV_8UC1, cv::Scalar(0));
  cv::erode(no_return_mask, erosion_result, kernel);

  cv::Mat blockage_mask(dimensions, CV_8UC1, cv::Scalar(0));
  cv::dilate(erosion_result, blockage_mask, kernel);

  return blockage_mask;
}

float BlockageDetector::get_nonzero_ratio(const cv::Mat & mask)
{
  size_t area = mask.cols * mask.rows;
  if (area == 0) {
    return 0.F;
  }

  return static_cast<float>(cv::countNonZero(mask)) / static_cast<float>(area);
}

void BlockageDetector::update_blockage_info(
  const cv::Mat & blockage_mask, BlockageAreaResult & area_result)
{
  if (area_result.blockage_ratio <= config_.blockage_ratio_threshold) {
    area_result.blockage_count = 0;
    return;
  }

  cv::Rect blockage_bb = cv::boundingRect(blockage_mask);
  double blockage_start_deg =
    blockage_bb.x * config_.horizontal_resolution + config_.angle_range_min_deg;
  double blockage_end_deg = (blockage_bb.x + blockage_bb.width) * config_.horizontal_resolution +
                            config_.angle_range_min_deg;

  area_result.blockage_start_deg = static_cast<float>(blockage_start_deg);
  area_result.blockage_end_deg = static_cast<float>(blockage_end_deg);

  if (area_result.blockage_count <= 2 * config_.blockage_count_threshold) {
    area_result.blockage_count += 1;
  }
}

BlockageDetectionResult BlockageDetector::compute_blockage_diagnostics(
  const cv::Mat & depth_image_16u)
{
  cv::Mat depth_image_8u = quantize_to_8u(depth_image_16u);
  cv::Mat no_return_mask = make_no_return_mask(depth_image_8u);
  cv::Mat blockage_mask = make_blockage_mask(no_return_mask);

  auto [ground_blockage_mask, sky_blockage_mask] =
    segment_into_ground_and_sky(blockage_mask, config_.horizontal_ring_id);

  result_.ground.blockage_ratio = get_nonzero_ratio(ground_blockage_mask);
  result_.sky.blockage_ratio = get_nonzero_ratio(sky_blockage_mask);

  update_blockage_info(ground_blockage_mask, result_.ground);
  update_blockage_info(sky_blockage_mask, result_.sky);

  result_.blockage_mask = blockage_mask;

  return result_;
}

DiagnosticOutput BlockageDetector::get_blockage_diagnostics_output() const
{
  DiagnosticOutput output;

  output.additional_data.push_back(
    {"ground_blockage_ratio", std::to_string(result_.ground.blockage_ratio)});
  output.additional_data.push_back(
    {"ground_blockage_count", std::to_string(result_.ground.blockage_count)});
  output.additional_data.push_back(
    {"ground_blockage_range_deg", "[" + std::to_string(result_.ground.blockage_start_deg) + "," +
                                    std::to_string(result_.ground.blockage_end_deg) + "]"});
  output.additional_data.push_back(
    {"sky_blockage_ratio", std::to_string(result_.sky.blockage_ratio)});
  output.additional_data.push_back(
    {"sky_blockage_count", std::to_string(result_.sky.blockage_count)});
  output.additional_data.push_back(
    {"sky_blockage_range_deg", "[" + std::to_string(result_.sky.blockage_start_deg) + "," +
                                 std::to_string(result_.sky.blockage_end_deg) + "]"});

  output.level = DiagnosticLevel::OK;
  output.message = "OK";

  if (result_.ground.blockage_ratio < 0) {
    output.level = DiagnosticLevel::STALE;
    output.message = "STALE";
  } else if (
    (result_.ground.blockage_ratio > config_.blockage_ratio_threshold) &&
    (result_.ground.blockage_count > config_.blockage_count_threshold)) {
    output.level = DiagnosticLevel::ERROR;
    output.message = "ERROR";
  } else if (result_.ground.blockage_ratio > 0.0f) {
    output.level = DiagnosticLevel::WARN;
    output.message = "WARN";
  }

  if ((result_.ground.blockage_ratio > 0.0f) && (result_.sky.blockage_ratio > 0.0f)) {
    output.message = output.message + ": LIDAR both blockage";
  } else if (result_.ground.blockage_ratio > 0.0f) {
    output.message = output.message + ": LIDAR ground blockage";
  } else if (result_.sky.blockage_ratio > 0.0f) {
    output.message = output.message + ": LIDAR sky blockage";
  }

  return output;
}

}  // namespace autoware::pointcloud_preprocessor
