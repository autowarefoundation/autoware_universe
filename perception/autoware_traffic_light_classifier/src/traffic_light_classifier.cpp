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

#include "traffic_light_classifier.hpp"

#include "traffic_light_classifier_process.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <tier4_perception_msgs/msg/traffic_light.hpp>

// cppcheck-suppress preprocessorErrorDirective
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{
TrafficLightClassifier::TrafficLightClassifier(
  std::shared_ptr<ClassifierInterface> classifier, uint8_t classify_traffic_light_type,
  double over_exposure_threshold, double under_exposure_threshold)
: classifier_(std::move(classifier)),
  classify_traffic_light_type_(classify_traffic_light_type),
  over_exposure_threshold_(over_exposure_threshold),
  under_exposure_threshold_(under_exposure_threshold)
{
}

std::optional<TrafficLightClassifier::Result> TrafficLightClassifier::classify(
  const sensor_msgs::msg::Image & image_msg,
  const tier4_perception_msgs::msg::TrafficLightRoiArray & rois) const
{
  Result result;
  if (rois.rois.empty()) {
    result.signals.header = image_msg.header;
    return result;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (const cv_bridge::Exception &) {
    return std::nullopt;
  }
  const cv::Mat & image = cv_ptr->image;

  std::vector<cv::Mat> images;
  // Valid ROIs in classification order (parallel to `images`). The classifier returns elements
  // only, so we zip each ROI's traffic_light_id / type back onto the output afterwards.
  std::vector<const tier4_perception_msgs::msg::TrafficLightRoi *> valid_rois;
  std::vector<size_t> exposure_out_of_range_indices;
  for (const auto & input_roi : rois.rois) {
    // ignore if the roi is not the type to be classified
    if (input_roi.traffic_light_type != classify_traffic_light_type_) {
      continue;
    }
    // skip if the roi size is zero
    if (input_roi.roi.height == 0 || input_roi.roi.width == 0) {
      continue;
    }

    const sensor_msgs::msg::RegionOfInterest & roi = input_roi.roi;
    auto roi_img = image(cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height));
    const double brightness = utils::compute_brightness(roi_img);
    if (brightness >= over_exposure_threshold_) {
      exposure_out_of_range_indices.emplace_back(images.size());
      result.detected_over_exposure = true;
    } else if (brightness <= under_exposure_threshold_) {
      exposure_out_of_range_indices.emplace_back(images.size());
      result.detected_under_exposure = true;
    }
    valid_rois.emplace_back(&input_roi);
    images.emplace_back(roi_img);
  }

  // classify the images
  if (!images.empty()) {
    auto classified = classifier_->classify(images);
    if (!classified) {
      return std::nullopt;
    }
    result.signals = std::move(*classified);
  }

  // One signal per input image is the ClassifierInterface contract. Enforce it here, at the one
  // place that relies on it: a backend returning a different count would break the per-index
  // traffic_light_id / type association below (and, for a longer result, read out of bounds).
  if (result.signals.signals.size() != images.size()) {
    return std::nullopt;
  }

  // The classifier leaves traffic_light_id / type unset; associate them by position.
  for (size_t i = 0; i < result.signals.signals.size(); i++) {
    result.signals.signals[i].traffic_light_id = valid_rois[i]->traffic_light_id;
    result.signals.signals[i].traffic_light_type = valid_rois[i]->traffic_light_type;
  }

  // Hand the classified crops back to the caller so it can request a debug view later; the debug
  // image reflects the classifier's view, before the UNKNOWN / exposure post-processing below.
  result.roi_images = images;

  // append the undetected rois as unknown
  for (const auto & input_roi : rois.rois) {
    // if the type is the target type but the roi size is zero, the roi is undetected
    if (
      (input_roi.roi.height == 0 || input_roi.roi.width == 0) &&
      input_roi.traffic_light_type == classify_traffic_light_type_) {
      tier4_perception_msgs::msg::TrafficLight signal;
      signal.traffic_light_id = input_roi.traffic_light_id;
      signal.traffic_light_type = input_roi.traffic_light_type;
      traffic_light_utils::setSignalUnknown(signal, 0.0);
      result.signals.signals.push_back(signal);
    }
  }

  // overwrite the out-of-range exposure rois with unknown
  for (const auto & idx : exposure_out_of_range_indices) {
    auto & signal = result.signals.signals.at(idx);
    traffic_light_utils::setSignalUnknown(signal, 0.0);
  }

  result.signals.header = image_msg.header;
  return result;
}

sensor_msgs::msg::Image::ConstSharedPtr TrafficLightClassifier::make_debug_image(
  const Result & result) const
{
  const cv::Mat debug_image = classifier_->make_debug_image(result.roi_images);
  if (debug_image.empty()) {
    return nullptr;
  }
  return cv_bridge::CvImage(result.signals.header, sensor_msgs::image_encodings::RGB8, debug_image)
    .toImageMsg();
}

}  // namespace autoware::traffic_light
