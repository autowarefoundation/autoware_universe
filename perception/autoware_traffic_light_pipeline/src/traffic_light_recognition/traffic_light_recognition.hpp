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

#ifndef TRAFFIC_LIGHT_RECOGNITION__TRAFFIC_LIGHT_RECOGNITION_HPP_
#define TRAFFIC_LIGHT_RECOGNITION__TRAFFIC_LIGHT_RECOGNITION_HPP_

#include <autoware/tensorrt_yolox/tensorrt_yolox_detector.hpp>
#include <autoware/traffic_light_classifier/traffic_light_classifier.hpp>
#include <autoware/traffic_light_map_based_detector/traffic_light_map_based_detector.hpp>
#include <tl_expected/expected.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <tf2/buffer_core.h>

#include <optional>
#include <string>

namespace autoware::traffic_light
{

struct TrafficLightRecognitionConfig
{
  std::string whole_image_detector_model_path;
  std::string whole_image_detector_label_path;
  std::string whole_image_detector_roi_remap_path;
  float whole_image_detector_score_threshold;
  float whole_image_detector_nms_threshold;

  double min_timestamp_offset;
  double max_timestamp_offset;

  std::string car_classifier_model_path;
  std::string car_classifier_label_path;

  std::string pedestrian_classifier_model_path;
  std::string pedestrian_classifier_label_path;

  double over_exposure_threshold;
  double under_exposure_threshold;
};

// Builds (and discards) the detector's TensorRT engines.
void build_engines(const TrafficLightRecognitionConfig & config);

struct TrafficLightRecognitionResult
{
  tier4_perception_msgs::msg::TrafficLightArray merged_signals;
  tier4_perception_msgs::msg::TrafficLightRoiArray selected_rois;
};

class TrafficLightRecognition
{
public:
  TrafficLightRecognition(
    const TrafficLightRecognitionConfig & config,
    const autoware_map_msgs::msg::LaneletMapBin & map_msg, const tf2::BufferCore & tf_buffer);

  std::optional<SetRouteError> set_route(
    const autoware_planning_msgs::msg::LaneletRoute & route_msg);

  tl::expected<TrafficLightRecognitionResult, std::string> run(
    const sensor_msgs::msg::Image & image, const sensor_msgs::msg::CameraInfo & camera_info);

private:
  autoware::tensorrt_yolox::TrtYoloXDetector whole_image_detector_;
  TrafficLightMapBasedDetector map_based_detector_;
  TrafficLightClassifier car_classifier_;
  TrafficLightClassifier pedestrian_classifier_;
  const tf2::BufferCore & tf_buffer_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_RECOGNITION__TRAFFIC_LIGHT_RECOGNITION_HPP_
