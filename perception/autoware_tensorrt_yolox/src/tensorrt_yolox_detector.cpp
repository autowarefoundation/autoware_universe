// Copyright 2026 TIER IV, Inc.
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

#include "autoware/tensorrt_yolox/tensorrt_yolox_detector.hpp"

#include "autoware/tensorrt_yolox/label.hpp"
#include "perception_utils/run_length_encoder.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <algorithm>
#include <cstring>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// cspell: ignore semseg

namespace autoware::tensorrt_yolox
{
TrtYoloXDetector::TrtYoloXDetector(const TrtYoloXDetectorConfig & config) : config_(config)
{
  if (config_.is_publish_color_mask && config_.semseg_color_map.empty()) {
    std::stringstream error_msg;
    error_msg << "semantic_segmentation_color_map_path must be specified "
              << "when `is_publish_color_mask` is true.";
    throw std::runtime_error{error_msg.str()};
  }

  const bool semseg_remap_configured = std::any_of(
    config_.roi_labels.begin(), config_.roi_labels.end(),
    [](const RoiLabel & roi_label) { return roi_label.semseg_id != g_unmapped_label_id; });
  if (config_.is_roi_overlap_semseg && !semseg_remap_configured) {
    std::stringstream error_msg;
    error_msg << "roi_to_semantic_segmentation_remap_path must be specified "
              << "when `is_roi_overlap_segmentation` is true.";
    throw std::runtime_error{error_msg.str()};
  }

  TrtCommonConfig trt_config(
    config_.model_path, config_.precision, "", (1ULL << 30U), config_.dla_core_id,
    config_.profile_per_layer);

  CalibrationConfig calib_config(
    config_.calibration_algorithm, config_.quantize_first_layer, config_.quantize_last_layer,
    config_.clip_value);

  const double norm_factor = 1.0;

  trt_yolox_ = std::make_unique<tensorrt_yolox::TrtYoloX>(
    trt_config, config_.roi_labels.size(), config_.score_threshold, config_.nms_threshold,
    config_.gpu_id, config_.calibration_image_list_path, norm_factor, calib_config);

  if (!trt_yolox_->isGPUInitialized()) {
    std::stringstream error_msg;
    error_msg << "GPU " << static_cast<int>(config_.gpu_id)
              << " does not exist or is not suitable.";
    throw std::runtime_error{error_msg.str()};
  }
}

tl::expected<TrtYoloXDetectorResult, std::string> TrtYoloXDetector::detect(
  const sensor_msgs::msg::Image & image_msg)
{
  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    return tl::make_unexpected(std::string("cv_bridge exception: ") + e.what());
  }
  const auto width = in_image_ptr->image.cols;
  const auto height = in_image_ptr->image.rows;

  tensorrt_yolox::ObjectArrays objects;
  std::vector<cv::Mat> masks = {cv::Mat(cv::Size(height, width), CV_8UC1, cv::Scalar(0))};

  if (!trt_yolox_->doInference({in_image_ptr->image}, objects, masks)) {
    return tl::make_unexpected(std::string("failed to run inference"));
  }
  auto & mask = masks.at(0);

  tier4_perception_msgs::msg::DetectedObjectsWithFeature out_objects;
  for (const auto & yolox_object : objects.at(0)) {
    tier4_perception_msgs::msg::DetectedObjectWithFeature object;
    object.feature.roi.x_offset = yolox_object.x_offset;
    object.feature.roi.y_offset = yolox_object.y_offset;
    object.feature.roi.width = yolox_object.width;
    object.feature.roi.height = yolox_object.height;
    object.object.existence_probability = yolox_object.score;

    // direct mapping from YOLOX ID to class ID
    const int target_class_id = config_.roi_labels[yolox_object.type].class_id;

    // drop the object if it is marked as ignore
    if (target_class_id == g_unmapped_label_id) continue;

    const auto classification =
      autoware_perception_msgs::build<autoware_perception_msgs::msg::ObjectClassification>()
        .label(static_cast<uint8_t>(target_class_id))
        .probability(1.0f);

    object.object.classification.push_back(classification);

    out_objects.feature_objects.push_back(object);

    const auto left = std::max(0, static_cast<int>(object.feature.roi.x_offset));
    const auto top = std::max(0, static_cast<int>(object.feature.roi.y_offset));
    const auto right =
      std::min(static_cast<int>(object.feature.roi.x_offset + object.feature.roi.width), width);
    const auto bottom =
      std::min(static_cast<int>(object.feature.roi.y_offset + object.feature.roi.height), height);
    cv::rectangle(
      in_image_ptr->image, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255), 3,
      8, 0);
    // Refine mask: replacing segmentation mask by roi class
    // This should remove when the segmentation accuracy is high
    if (config_.is_roi_overlap_semseg && trt_yolox_->getMultitaskNum() > 0) {
      overlapSegmentByRoi(yolox_object, mask, width, height);
    }
  }

  TrtYoloXDetectorResult result;

  if (trt_yolox_->getMultitaskNum() > 0) {
    sensor_msgs::msg::Image::SharedPtr out_mask_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, mask)
        .toImageMsg();
    out_mask_msg->header = image_msg.header;

    std::vector<std::pair<uint8_t, int>> compressed_data = perception_utils::runLengthEncoder(mask);
    int step = sizeof(uint8_t) + sizeof(int);
    out_mask_msg->data.resize(static_cast<int>(compressed_data.size()) * step);
    for (size_t i = 0; i < compressed_data.size(); ++i) {
      std::memcpy(&out_mask_msg->data[i * step], &compressed_data.at(i).first, sizeof(uint8_t));
      std::memcpy(&out_mask_msg->data[i * step + 1], &compressed_data.at(i).second, sizeof(int));
    }

    result.mask = *out_mask_msg;
  }

  result.image = *in_image_ptr->toImageMsg();
  out_objects.header = image_msg.header;
  result.objects = out_objects;

  if (config_.is_publish_color_mask && trt_yolox_->getMultitaskNum() > 0) {
    cv::Mat color_mask = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC3);
    getColorizedMask(mask, color_mask);

    sensor_msgs::msg::Image::SharedPtr output_color_mask_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, color_mask)
        .toImageMsg();
    output_color_mask_msg->header = image_msg.header;
    result.color_mask = *output_color_mask_msg;
  }

  return result;
}

void TrtYoloXDetector::overlapSegmentByRoi(
  const tensorrt_yolox::Object & roi_object, cv::Mat & mask, const int orig_width,
  const int orig_height)
{
  if (roi_object.score < config_.overlap_roi_score_threshold) return;
  // A negative semseg ID marks a ROI class that should not overlay the segmentation mask.
  const int seg_class_index = config_.roi_labels[roi_object.type].semseg_id;
  if (seg_class_index < 0) return;

  const float scale_x = static_cast<float>(mask.cols) / static_cast<float>(orig_width);
  const float scale_y = static_cast<float>(mask.rows) / static_cast<float>(orig_height);
  const int roi_width = static_cast<int>(roi_object.width * scale_x);
  const int roi_height = static_cast<int>(roi_object.height * scale_y);
  const int roi_x_offset = static_cast<int>(roi_object.x_offset * scale_x);
  const int roi_y_offset = static_cast<int>(roi_object.y_offset * scale_y);

  cv::Mat replace_roi(
    cv::Size(roi_width, roi_height), mask.type(), static_cast<uint8_t>(seg_class_index));
  replace_roi.copyTo(mask.colRange(roi_x_offset, roi_x_offset + roi_width)
                       .rowRange(roi_y_offset, roi_y_offset + roi_height));
}

/**
 * @brief get colorized masks from index using specific colormap
 * @param[out] cmask colorized mask
 * @param[in] index multitask index
 * @param[in] colormap colormap for masks
 */
void TrtYoloXDetector::getColorizedMask(const cv::Mat & mask, cv::Mat & cmask)
{
  int width = mask.cols;
  int height = mask.rows;
  if ((cmask.cols != width) || (cmask.rows != height)) {
    throw std::runtime_error("input and output image have difference size.");
  }

  const auto & semseg_color_map = config_.semseg_color_map;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      unsigned char id = mask.at<unsigned char>(y, x);
      const bool unregistered = id >= semseg_color_map.size();
      if (unregistered) {
        const auto black = cv::Vec3b(0, 0, 0);
        cmask.at<cv::Vec3b>(y, x) = black;
        continue;
      }
      cmask.at<cv::Vec3b>(y, x)[0] = semseg_color_map[id].color[2];
      cmask.at<cv::Vec3b>(y, x)[1] = semseg_color_map[id].color[1];
      cmask.at<cv::Vec3b>(y, x)[2] = semseg_color_map[id].color[0];
    }
  }
}

}  // namespace autoware::tensorrt_yolox
