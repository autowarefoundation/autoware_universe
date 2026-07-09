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

#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <cstdint>
#include <memory>
#include <optional>

namespace autoware::traffic_light
{
// ROS-free classification orchestration extracted from TrafficLightClassifierNodelet.
// Owns the classifier backend and the per-ROI filtering / exposure / UNKNOWN-handling
// logic; the node remains a thin adapter that handles I/O (params, pub/sub, diagnostics).
class TrafficLightClassifier
{
public:
  struct Result
  {
    // Classification output. The header is intentionally left unset: the node stamps
    // it from the input image before publishing.
    tier4_perception_msgs::msg::TrafficLightArray signals;
    bool detected_over_exposure = false;
    bool detected_under_exposure = false;
  };

  TrafficLightClassifier(
    std::shared_ptr<ClassifierInterface> classifier, uint8_t classify_traffic_light_type,
    double over_exposure_threshold, double under_exposure_threshold);

  // Pure orchestration over an already-decoded RGB image and its ROIs: filter ROIs by type,
  // crop and classify the valid ones, append undetected (zero-sized) ROIs as UNKNOWN, and
  // overwrite over/under-exposed slots with UNKNOWN. Returns std::nullopt when the classifier
  // backend fails, so the caller can skip publishing.
  std::optional<Result> classify(
    const cv::Mat & image, const tier4_perception_msgs::msg::TrafficLightRoiArray & rois) const;

private:
  std::shared_ptr<ClassifierInterface> classifier_;
  uint8_t classify_traffic_light_type_;
  double over_exposure_threshold_;
  double under_exposure_threshold_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_CLASSIFIER_HPP_
