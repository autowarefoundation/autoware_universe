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
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <tf2/buffer_core.h>

#include <optional>
#include <string>
#include <vector>

namespace autoware::traffic_light
{

struct ClassifierModelConfig
{
  std::string model_path;
  std::string label_path;
  // TensorRT engine precision: "fp32" / "fp16" / "int8".
  std::string precision;
  // Input normalization; must match the preprocessing the model at model_path was trained with.
  std::vector<float> mean;
  std::vector<float> std;
};

struct TrafficLightRecognitionConfig
{
  std::string whole_image_detector_model_path;
  std::string whole_image_detector_label_path;
  std::string whole_image_detector_roi_remap_path;
  float whole_image_detector_score_threshold;
  float whole_image_detector_nms_threshold;
  std::string whole_image_detector_precision;

  double min_timestamp_offset;
  double max_timestamp_offset;

  ClassifierModelConfig car_classifier;
  ClassifierModelConfig pedestrian_classifier;

  double over_exposure_threshold;
  double under_exposure_threshold;

  std::string diagnostics_node_name;
};

// Builds (and discards) the detector's TensorRT engines.
void build_engines(const TrafficLightRecognitionConfig & config);

struct TrafficLightRecognitionResult
{
  tier4_perception_msgs::msg::TrafficLightArray merged_signals;
  tier4_perception_msgs::msg::TrafficLightRoiArray selected_rois;
  diagnostic_msgs::msg::DiagnosticArray diagnostics;
};

class TrafficLightRecognition
{
public:
  TrafficLightRecognition(
    const TrafficLightRecognitionConfig & config, const tf2::BufferCore & tf_buffer);

  void set_map(const autoware_map_msgs::msg::LaneletMapBin & map_msg);

  tl::expected<void, std::string> set_route(
    const autoware_planning_msgs::msg::LaneletRoute & route_msg);

  tl::expected<TrafficLightRecognitionResult, std::string> run(
    const sensor_msgs::msg::Image & image, const sensor_msgs::msg::CameraInfo & camera_info);

private:
  TrafficLightMapBasedDetectorConfig map_based_detector_config_;

  autoware::tensorrt_yolox::TrtYoloXDetector whole_image_detector_;
  std::optional<TrafficLightMapBasedDetector> map_based_detector_;
  TrafficLightClassifier car_classifier_;
  TrafficLightClassifier pedestrian_classifier_;
  std::string diagnostics_node_name_;
  const tf2::BufferCore & tf_buffer_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_RECOGNITION__TRAFFIC_LIGHT_RECOGNITION_HPP_
