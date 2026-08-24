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

#ifndef TRAFFIC_LIGHT_CLASSIFIER_HPP_
#define TRAFFIC_LIGHT_CLASSIFIER_HPP_

#include "classifier/classifier_interface.hpp"

#include <opencv2/core/core.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::traffic_light
{
// ROS-free classification orchestration extracted from TrafficLightClassifierNode.
// Owns the classifier backend and the per-ROI filtering / exposure / UNKNOWN-handling
// logic; the node remains a thin adapter that handles I/O (params, pub/sub, diagnostics).
class TrafficLightClassifier
{
public:
  struct Result
  {
    // Classification output, stamped with the input image's header.
    tier4_perception_msgs::msg::TrafficLightArray signals;
    bool detected_over_exposure = false;
    bool detected_under_exposure = false;
    // The cropped ROI images fed to the classifier, in classification order (cheap cv::Mat
    // views). The node passes them back to make_debug_image to render a debug view.
    std::vector<cv::Mat> roi_images;
  };

  TrafficLightClassifier(
    std::shared_ptr<ClassifierInterface> classifier, uint8_t classify_traffic_light_type,
    double over_exposure_threshold, double under_exposure_threshold);

  // Returns an empty, header-stamped Result without decoding the image when `rois` is empty.
  // Otherwise decodes the ROS image message, then filters ROIs by type, crops and classifies
  // the valid ones, appends undetected (zero-sized) ROIs as UNKNOWN, and overwrites
  // over/under-exposed slots with UNKNOWN. Returns std::nullopt when decoding or the classifier
  // backend fails, so the caller can skip publishing.
  std::optional<Result> classify(
    const sensor_msgs::msg::Image & image_msg,
    const tier4_perception_msgs::msg::TrafficLightRoiArray & rois) const;

  // Composite debug view for a classify() result, built from its roi_images and stamped with
  // its signals' header. Returns nullptr when there is nothing to render. Off the hot path: the
  // node calls this only when a debug consumer is attached.
  sensor_msgs::msg::Image::ConstSharedPtr make_debug_image(const Result & result) const;

private:
  std::shared_ptr<ClassifierInterface> classifier_;
  uint8_t classify_traffic_light_type_;
  double over_exposure_threshold_;
  double under_exposure_threshold_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_CLASSIFIER_HPP_
