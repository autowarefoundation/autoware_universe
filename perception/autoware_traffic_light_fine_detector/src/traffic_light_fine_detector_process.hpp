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

#ifndef TRAFFIC_LIGHT_FINE_DETECTOR_PROCESS_HPP_
#define TRAFFIC_LIGHT_FINE_DETECTOR_PROCESS_HPP_

#include <autoware/tensorrt_yolox/tensorrt_yolox.hpp>
#include <opencv2/core/types.hpp>

#include <sensor_msgs/msg/region_of_interest.hpp>

namespace autoware::traffic_light
{

namespace utils
{

float calWeightedIou(
  const sensor_msgs::msg::RegionOfInterest & bbox1, const autoware::tensorrt_yolox::Object & bbox2);

bool fitInFrame(cv::Point & lt, cv::Point & rb, const cv::Size & size);

}  // namespace utils

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_FINE_DETECTOR_PROCESS_HPP_
