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

#include "traffic_light_selector.hpp"

#include <sensor_msgs/msg/region_of_interest.hpp>

#include <map>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{

namespace
{

void evaluateWholeRois(
  const std::vector<RegionOfInterest> & detected_rois,
  const std::map<int64_t, RegionOfInterest> & expect_rois_shifted_map, double & total_max_iou,
  std::map<int64_t, RegionOfInterest> & total_max_iou_rois_map)
{
  for (const auto & expect_roi_shifted : expect_rois_shifted_map) {
    double max_iou = 0.0;
    RegionOfInterest max_iou_roi;
    for (const auto & detected_roi : detected_rois) {
      const double iou = utils::getGenIoU(expect_roi_shifted.second, detected_roi);
      if (iou > max_iou) {
        max_iou = iou;
        max_iou_roi = detected_roi;
      }
    }
    total_max_iou += max_iou;
    total_max_iou_rois_map[expect_roi_shifted.first] = max_iou_roi;
  }
}

}  // namespace

TrafficLightRoiArray select(
  const DetectedObjectsWithFeature & detected_traffic_light_msg,
  const TrafficLightRoiArray & rough_rois_msg, const TrafficLightRoiArray & expected_rois_msg,
  const sensor_msgs::msg::CameraInfo & camera_info_msg)
{
  const auto image_width = camera_info_msg.width;
  const auto image_height = camera_info_msg.height;
  std::map<int64_t, RegionOfInterest> rough_rois_map;
  for (const auto & roi : rough_rois_msg.rois) {
    rough_rois_map[roi.traffic_light_id] = roi.roi;
  }

  std::vector<RegionOfInterest> detected_rois;
  for (const auto & detected_object : detected_traffic_light_msg.feature_objects) {
    detected_rois.push_back(detected_object.feature.roi);
  }

  double final_iou = 0.0;
  std::map<int64_t, RegionOfInterest> final_rois_map;
  for (const auto & expected_rois_msg_rois : expected_rois_msg.rois) {
    const auto traffic_light_id = expected_rois_msg_rois.traffic_light_id;
    const auto & rough_roi = rough_rois_map[traffic_light_id];
    const auto & expect_roi = expected_rois_msg_rois.roi;

    for (const auto & detected_roi : detected_rois) {
      if (!utils::isCenterInsideRoughRoi(detected_roi, rough_roi)) {
        continue;
      }

      // shift expect roi because the location of detected roi is better that it
      int32_t shift_x, shift_y;
      utils::computeCenterOffset(detected_roi, expect_roi, shift_x, shift_y);

      std::map<int64_t, RegionOfInterest> expect_rois_shifted_map;
      for (const auto & rois : expected_rois_msg.rois) {
        const auto expect_roi_shifted =
          utils::getShiftedRoi(rois.roi, image_width, image_height, shift_x, shift_y);
        expect_rois_shifted_map[rois.traffic_light_id] = expect_roi_shifted;
      }

      // check total IoU after all expect roi shift
      double total_max_iou = 0.0;
      std::map<int64_t, RegionOfInterest> total_max_iou_rois_map;
      evaluateWholeRois(
        detected_rois, expect_rois_shifted_map, total_max_iou, total_max_iou_rois_map);

      if (total_max_iou > final_iou) {
        final_iou = total_max_iou;
        final_rois_map = std::move(total_max_iou_rois_map);
      }
    }
  }

  // assign max iou roi to output
  using tier4_perception_msgs::msg::TrafficLightRoi;
  TrafficLightRoiArray output;
  output.header = detected_traffic_light_msg.header;
  for (const auto & expected_rois_msg_rois : expected_rois_msg.rois) {
    const auto traffic_light_id = expected_rois_msg_rois.traffic_light_id;
    const auto traffic_light_type = expected_rois_msg_rois.traffic_light_type;
    TrafficLightRoi traffic_light_roi;

    if (final_rois_map.find(traffic_light_id) != final_rois_map.end()) {
      traffic_light_roi.traffic_light_id = traffic_light_id;
      traffic_light_roi.traffic_light_type = traffic_light_type;
      traffic_light_roi.roi = final_rois_map[traffic_light_id];
      output.rois.push_back(traffic_light_roi);
    } else {
      traffic_light_roi.traffic_light_id = traffic_light_id;
      traffic_light_roi.traffic_light_type = traffic_light_type;
      output.rois.push_back(traffic_light_roi);
    }
  }

  return output;
}

}  // namespace autoware::traffic_light
