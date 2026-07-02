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

#ifndef TRAFFIC_LIGHT_SELECTOR_HPP_
#define TRAFFIC_LIGHT_SELECTOR_HPP_

#include "traffic_light_selector_utils.hpp"

#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <map>
#include <vector>

namespace autoware::traffic_light
{
using sensor_msgs::msg::RegionOfInterest;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

/**
 * @brief For each expected traffic light ROI, select the detected ROI that best matches it.
 *
 * The expected ROIs are shifted onto each candidate detection (restricted to detections whose
 * center falls inside the corresponding rough ROI), and the shift that maximizes the total IoU
 * across all expected ROIs is chosen. Expected ROIs left unmatched fall back to a default
 * (empty) ROI in the output.
 *
 * @param detected_traffic_light_msg traffic light detections from the detector
 * @param rough_rois_msg rough ROIs used to gate candidate detections per traffic light
 * @param expected_rois_msg expected ROIs projected from the map, one per traffic light
 * @param camera_info_msg camera info used to clamp shifted ROIs to the image bounds
 * @return selected ROI (or a default one when unmatched) for each expected traffic light
 */
TrafficLightRoiArray select(
  const DetectedObjectsWithFeature & detected_traffic_light_msg,
  const TrafficLightRoiArray & rough_rois_msg, const TrafficLightRoiArray & expected_rois_msg,
  const sensor_msgs::msg::CameraInfo & camera_info_msg);

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_SELECTOR_HPP_
