// Copyright 2023 TIER IV, Inc.
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

#ifndef CLASSIFIER__CLASSIFIER_INTERFACE_HPP_
#define CLASSIFIER__CLASSIFIER_INTERFACE_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tier4_perception_msgs/msg/traffic_light_array.hpp>

#include <optional>
#include <vector>

namespace autoware::traffic_light
{
class ClassifierInterface
{
public:
  virtual ~ClassifierInterface() = default;
  // Classify each ROI image and return one signal per image, carrying the classified elements.
  // traffic_light_id / type are left unset -- the caller associates them. Returns std::nullopt on
  // inference failure.
  virtual std::optional<tier4_perception_msgs::msg::TrafficLightArray> classify(
    const std::vector<cv::Mat> & images) = 0;

  // One composite RGB debug view for the batch, rendered from the most recent classify() call.
  // Returns an empty Mat when there is nothing to show. The caller (the node) invokes it only when
  // a debug consumer is attached, so this stays off the hot path.
  virtual cv::Mat make_debug_image(const std::vector<cv::Mat> & images) const = 0;
};
}  // namespace autoware::traffic_light

#endif  // CLASSIFIER__CLASSIFIER_INTERFACE_HPP_
