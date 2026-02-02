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

#include "autoware/pointcloud_preprocessor/blockage_diag/blockage_diag.hpp"

#include <opencv2/imgproc.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <string>
#include <vector>
#include <utility>

namespace autoware::pointcloud_preprocessor
{

MultiFrameDetectionAggregator::MultiFrameDetectionAggregator(
  const MultiFrameDetectionAggregatorConfig & config)
: frame_count_(0), buffering_interval_(config.buffering_interval)
{
  mask_buffer_.set_capacity(config.buffering_frames);
}

cv::Mat MultiFrameDetectionAggregator::update(const cv::Mat & mask)
{
  if (buffering_interval_ == 0) {
    return mask.clone();
  }

  assert(mask.type() == CV_8UC1);
  auto dimensions = mask.size();

  cv::Mat time_series_result(dimensions, CV_8UC1, cv::Scalar(0));
  cv::Mat time_series_mask(dimensions, CV_8UC1, cv::Scalar(0));
  cv::Mat binarized_mask(dimensions, CV_8UC1, cv::Scalar(0));

  binarized_mask = mask / 255;
  if (frame_count_ >= buffering_interval_) {
    mask_buffer_.push_back(binarized_mask);
    frame_count_ = 0;
  } else {
    frame_count_++;
  }

  for (const auto & binary_mask : mask_buffer_) {
    time_series_mask += binary_mask;
  }

  cv::inRange(time_series_mask, mask_buffer_.size() - 1, mask_buffer_.size(), time_series_result);

  return time_series_result;
}

DustDetector::DustDetector(const DustDetectionConfig & config) : config_(config)
{
}

cv::Mat make_no_return_mask(const cv::Mat & depth_image_16u)
{
  assert(depth_image_16u.type() == CV_16UC1);
  auto dimensions = depth_image_16u.size();

  cv::Mat no_return_mask_8u(dimensions, CV_8UC1, cv::Scalar(0));
  // 16-bit depth values 0-256 (equivalent to 8-bit 0-1) indicate no return
  cv::inRange(depth_image_16u, 0, 256, no_return_mask_8u);

  return no_return_mask_8u;
}

std::pair<cv::Mat, cv::Mat> segment_into_ground_and_sky(
  const cv::Mat & mask, int horizontal_ring_id)
{
  assert(mask.type() == CV_8UC1);
  auto dimensions = mask.size();

  cv::Mat sky_mask;
  mask(cv::Rect(0, 0, dimensions.width, horizontal_ring_id)).copyTo(sky_mask);

  cv::Mat ground_mask;
  mask(cv::Rect(0, horizontal_ring_id, dimensions.width, dimensions.height - horizontal_ring_id))
    .copyTo(ground_mask);

  return {ground_mask, sky_mask};
}

cv::Mat DustDetector::compute_dust_diagnostics(const cv::Mat & depth_image_16u)
{
  cv::Mat no_return_mask = make_no_return_mask(depth_image_16u);

  assert(no_return_mask.type() == CV_8UC1);
  auto dimensions = no_return_mask.size();

  auto [single_dust_ground_img, sky_blank] =
    segment_into_ground_and_sky(no_return_mask, config_.horizontal_ring_id);

  // It is normal for the sky region to be blank, therefore ignore it.
  sky_blank.setTo(cv::Scalar(0));

  int kernel_size = 2 * config_.dust_kernel_size + 1;
  int kernel_center = config_.dust_kernel_size;
  cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(kernel_size, kernel_size), cv::Point(kernel_center, kernel_center));

  cv::dilate(single_dust_ground_img, single_dust_ground_img, kernel);
  cv::erode(single_dust_ground_img, single_dust_ground_img, kernel);
  cv::inRange(single_dust_ground_img, 254, 255, single_dust_ground_img);

  // Re-assemble the processed ground dust image and the sky blank.
  cv::Mat single_dust_img(dimensions, CV_8UC1, cv::Scalar(0));
  cv::vconcat(sky_blank, single_dust_ground_img, single_dust_img);

  result_.ground_dust_ratio = static_cast<float>(cv::countNonZero(single_dust_ground_img)) /
                              (single_dust_ground_img.cols * single_dust_ground_img.rows);

  if (result_.ground_dust_ratio > config_.dust_ratio_threshold) {
    if (result_.dust_frame_count < 2 * config_.dust_count_threshold) {
      result_.dust_frame_count++;
    }
  } else {
    result_.dust_frame_count = 0;
  }

  return single_dust_img;
}

DiagnosticOutput DustDetector::get_dust_diagnostics_output() const
{
  DiagnosticOutput output;

  output.additional_data.push_back(
    {"ground_dust_ratio", std::to_string(result_.ground_dust_ratio)});

  output.level = DiagnosticLevel::OK;
  output.message = "OK";

  if (result_.ground_dust_ratio < 0.0f) {
    output.level = DiagnosticLevel::STALE;
    output.message = "STALE";
  } else if (
    (result_.ground_dust_ratio > config_.dust_ratio_threshold) &&
    (result_.dust_frame_count > config_.dust_count_threshold)) {
    output.level = DiagnosticLevel::ERROR;
    output.message = "ERROR";
  } else if (result_.ground_dust_ratio > 0.0f) {
    output.level = DiagnosticLevel::WARN;
    output.message = "WARN";
  }

  if (result_.ground_dust_ratio > 0.0f) {
    output.message = output.message + ": LIDAR ground dust";
  }
  return output;
}

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
  double blockage_start_deg = blockage_bb.x * config_.horizontal_resolution + config_.angle_range_min_deg;
  double blockage_end_deg =
    (blockage_bb.x + blockage_bb.width) * config_.horizontal_resolution + config_.angle_range_min_deg;

  area_result.blockage_start_deg = static_cast<float>(blockage_start_deg);
  area_result.blockage_end_deg = static_cast<float>(blockage_end_deg);

  if (area_result.blockage_count <= 2 * config_.blockage_count_threshold) {
    area_result.blockage_count += 1;
  }
}

cv::Mat BlockageDetector::compute_blockage_diagnostics(const cv::Mat & depth_image_16u)
{
  cv::Mat no_return_mask = make_no_return_mask(depth_image_16u);
  cv::Mat blockage_mask = make_blockage_mask(no_return_mask);

  auto [ground_blockage_mask, sky_blockage_mask] =
    segment_into_ground_and_sky(blockage_mask, config_.horizontal_ring_id);

  result_.ground.blockage_ratio = get_nonzero_ratio(ground_blockage_mask);
  result_.sky.blockage_ratio = get_nonzero_ratio(sky_blockage_mask);

  update_blockage_info(ground_blockage_mask, result_.ground);
  update_blockage_info(sky_blockage_mask, result_.sky);

  return blockage_mask;
}

DiagnosticOutput BlockageDetector::get_blockage_diagnostics_output() const
{
  DiagnosticOutput output;

  output.additional_data.push_back(
    {"ground_blockage_ratio", std::to_string(result_.ground.blockage_ratio)});
  output.additional_data.push_back(
    {"ground_blockage_count", std::to_string(result_.ground.blockage_count)});
  output.additional_data.push_back(
    {"ground_blockage_range_deg",
     "[" + std::to_string(result_.ground.blockage_start_deg) + "," +
       std::to_string(result_.ground.blockage_end_deg) + "]"});
  output.additional_data.push_back(
    {"sky_blockage_ratio", std::to_string(result_.sky.blockage_ratio)});
  output.additional_data.push_back(
    {"sky_blockage_count", std::to_string(result_.sky.blockage_count)});
  output.additional_data.push_back(
    {"sky_blockage_range_deg",
     "[" + std::to_string(result_.sky.blockage_start_deg) + "," +
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

void validate_pointcloud_fields(const sensor_msgs::msg::PointCloud2 & input)
{
  std::vector<std::string> required_fields = {"channel", "azimuth", "distance"};

  for (const auto & field : input.fields) {
    auto it = std::find(required_fields.begin(), required_fields.end(), field.name);
    if (it != required_fields.end()) {
      required_fields.erase(it);
    }
  }

  bool has_all_required_fields = required_fields.empty();
  if (has_all_required_fields) {
    return;
  }

  std::string error_msg = "PointCloud2 missing required fields:";
  for (const auto & missing_field : required_fields) {
    error_msg += " " + missing_field;
  }
  throw std::runtime_error(error_msg);
}

}  // namespace autoware::pointcloud_preprocessor
