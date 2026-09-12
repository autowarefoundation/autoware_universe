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

#include "traffic_light_recognition.hpp"

#include <autoware/tensorrt_yolox/label.hpp>
#include <autoware/traffic_light_category_merger/traffic_light_category_merger.hpp>
#include <autoware/traffic_light_classifier/classifier/cnn_classifier.hpp>
#include <autoware/traffic_light_selector/traffic_light_selector.hpp>

#include <tier4_perception_msgs/msg/traffic_light.hpp>

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{
namespace
{
std::vector<std::string> read_label_file(const std::string & filepath)
{
  std::ifstream labels_file(filepath);
  if (!labels_file.is_open()) {
    throw std::runtime_error("Could not open label file: " + filepath);
  }
  std::vector<std::string> labels;
  std::string label;
  while (std::getline(labels_file, label)) {
    labels.push_back(label);
  }
  return labels;
}

autoware::tensorrt_yolox::TrtYoloXDetectorConfig make_whole_image_detector_config(
  const TrafficLightRecognitionConfig & config)
{
  autoware::tensorrt_yolox::TrtYoloXDetectorConfig detector_config;
  detector_config.model_path = config.whole_image_detector_model_path;
  detector_config.score_threshold = config.whole_image_detector_score_threshold;
  detector_config.nms_threshold = config.whole_image_detector_nms_threshold;
  // Only fp16 models are used in this package, so precision and the int8-only knobs
  // (calibration_algorithm / dla_core_id / quantize_first_layer / quantize_last_layer /
  // clip_value / calibration_image_list_path) are fixed rather than exposed as parameters.
  // profile_per_layer is dev-only (may affect execution speed) and likewise fixed. gpu_id is
  // fixed to the default CUDA device: this package does not need per-node GPU selection.
  detector_config.precision = "fp16";
  detector_config.calibration_algorithm = "Entropy";
  detector_config.dla_core_id = -1;
  detector_config.quantize_first_layer = false;
  detector_config.quantize_last_layer = false;
  detector_config.profile_per_layer = false;
  detector_config.clip_value = 6.0;
  detector_config.calibration_image_list_path = "";
  detector_config.gpu_id = 0;

  detector_config.roi_labels = autoware::tensorrt_yolox::load_label_maps(
    config.whole_image_detector_label_path, config.whole_image_detector_roi_remap_path, "");
  // The traffic-light yolox model has no segmentation head: these are fixed rather than exposed
  // as parameters.
  detector_config.semseg_color_map = autoware::tensorrt_yolox::load_segmentation_colormap("");
  detector_config.is_roi_overlap_semseg = false;
  detector_config.is_publish_color_mask = false;
  detector_config.overlap_roi_score_threshold = 0.0f;
  return detector_config;
}

TrafficLightMapBasedDetectorConfig make_map_based_detector_config(
  const TrafficLightRecognitionConfig & config)
{
  TrafficLightMapBasedDetectorConfig detector_config;
  detector_config.max_vibration_pitch = 0.01745329251;  // 1 deg
  detector_config.max_vibration_yaw = 0.01745329251;    // 1 deg
  detector_config.max_vibration_height = 0.5;
  detector_config.max_vibration_width = 0.5;
  detector_config.max_vibration_depth = 0.5;
  detector_config.max_detection_range = 200.0;
  detector_config.car_traffic_light_max_angle_range = 40.0;
  detector_config.pedestrian_traffic_light_max_angle_range = 80.0;
  detector_config.min_timestamp_offset = config.min_timestamp_offset;
  detector_config.max_timestamp_offset = config.max_timestamp_offset;
  return detector_config;
}

CNNConfig make_cnn_config(const std::string & model_path, const std::string & label_path)
{
  CNNConfig cnn_config;
  cnn_config.model_path = model_path;
  cnn_config.precision = "fp16";
  cnn_config.labels = read_label_file(label_path);
  cnn_config.mean = {123.675f, 116.28f, 103.53f};
  cnn_config.std = {58.395f, 57.12f, 57.375f};
  return cnn_config;
}

TrafficLightClassifier make_car_classifier(const TrafficLightRecognitionConfig & config)
{
  auto backend = std::make_shared<CNNClassifier>(
    make_cnn_config(config.car_classifier_model_path, config.car_classifier_label_path));
  return TrafficLightClassifier(
    std::move(backend), tier4_perception_msgs::msg::TrafficLight::CAR_TRAFFIC_LIGHT,
    config.over_exposure_threshold, config.under_exposure_threshold);
}

TrafficLightClassifier make_pedestrian_classifier(const TrafficLightRecognitionConfig & config)
{
  auto backend = std::make_shared<CNNClassifier>(make_cnn_config(
    config.pedestrian_classifier_model_path, config.pedestrian_classifier_label_path));
  return TrafficLightClassifier(
    std::move(backend), tier4_perception_msgs::msg::TrafficLight::PEDESTRIAN_TRAFFIC_LIGHT,
    config.over_exposure_threshold, config.under_exposure_threshold);
}
}  // namespace

void build_engines(const TrafficLightRecognitionConfig & config)
{
  [[maybe_unused]] autoware::tensorrt_yolox::TrtYoloXDetector whole_image_detector(
    make_whole_image_detector_config(config));
  [[maybe_unused]] auto car_classifier = make_car_classifier(config);
  [[maybe_unused]] auto pedestrian_classifier = make_pedestrian_classifier(config);
}

TrafficLightRecognition::TrafficLightRecognition(
  const TrafficLightRecognitionConfig & config,
  const autoware_map_msgs::msg::LaneletMapBin & map_msg, const tf2::BufferCore & tf_buffer)
: whole_image_detector_(make_whole_image_detector_config(config)),
  map_based_detector_(make_map_based_detector_config(config), map_msg),
  car_classifier_(make_car_classifier(config)),
  pedestrian_classifier_(make_pedestrian_classifier(config)),
  tf_buffer_(tf_buffer)
{
}

std::optional<SetRouteError> TrafficLightRecognition::set_route(
  const autoware_planning_msgs::msg::LaneletRoute & route_msg)
{
  return map_based_detector_.set_route(route_msg);
}

tl::expected<TrafficLightRecognitionResult, std::string> TrafficLightRecognition::run(
  const sensor_msgs::msg::Image & image, const sensor_msgs::msg::CameraInfo & camera_info)
{
  const auto detected = whole_image_detector_.detect(image);
  if (!detected) {
    return tl::make_unexpected("whole_image_detector failed: " + detected.error());
  }

  const auto map_based_result = map_based_detector_.detect(tf_buffer_, camera_info);

  const auto selected_rois = select(
    detected->objects, map_based_result.rough_rois, map_based_result.expect_rois, camera_info);

  const auto car_result = car_classifier_.classify(image, selected_rois);
  if (!car_result) {
    return tl::make_unexpected("car classifier failed");
  }

  const auto pedestrian_result = pedestrian_classifier_.classify(image, selected_rois);
  if (!pedestrian_result) {
    return tl::make_unexpected("pedestrian classifier failed");
  }

  TrafficLightRecognitionResult result;
  result.merged_signals =
    TrafficLightCategoryMerger::merge(car_result->signals, pedestrian_result->signals);
  result.selected_rois = selected_rois;
  return result;
}

}  // namespace autoware::traffic_light
