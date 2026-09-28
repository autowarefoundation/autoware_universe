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

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/traffic_light.hpp>

#include <cstdint>
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

diagnostic_msgs::msg::KeyValue make_key_value(const std::string & key, const bool value)
{
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = key;
  key_value.value = value ? "True" : "False";
  return key_value;
}

diagnostic_msgs::msg::DiagnosticArray make_exposure_diagnostics(
  const std::string & node_name, const builtin_interfaces::msg::Time & stamp,
  const bool detected_over_exposure, const bool detected_under_exposure)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = node_name + ": traffic_light_classifier";
  status.hardware_id = node_name;
  status.values.push_back(
    make_key_value("detect_traffic_light_over_exposure", detected_over_exposure));
  status.values.push_back(
    make_key_value("detect_traffic_light_under_exposure", detected_under_exposure));

  if (detected_over_exposure || detected_under_exposure) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message =
      "Detected out-of-range exposure in ROI. Corresponding ROI was overwritten "
      "with UNKNOWN.";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";
  }

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = stamp;
  diagnostics.status.push_back(status);
  return diagnostics;
}

TrafficLightRecognitionResult make_empty_result(
  const std::string & node_name, const std_msgs::msg::Header & image_header)
{
  TrafficLightRecognitionResult result;
  result.merged_signals.header = image_header;
  result.selected_rois.header = image_header;
  result.diagnostics = make_exposure_diagnostics(node_name, image_header.stamp, false, false);
  return result;
}

autoware::tensorrt_yolox::TrtYoloXDetectorConfig make_whole_image_detector_config(
  const TrafficLightRecognitionConfig & config)
{
  autoware::tensorrt_yolox::TrtYoloXDetectorConfig detector_config;
  detector_config.model_path = config.whole_image_detector_model_path;
  detector_config.score_threshold = config.whole_image_detector_score_threshold;
  detector_config.nms_threshold = config.whole_image_detector_nms_threshold;
  detector_config.precision = config.whole_image_detector_precision;
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

CNNConfig make_cnn_config(const ClassifierModelConfig & classifier_config)
{
  CNNConfig cnn_config;
  cnn_config.model_path = classifier_config.model_path;
  cnn_config.precision = classifier_config.precision;
  cnn_config.labels = read_label_file(classifier_config.label_path);
  cnn_config.mean = classifier_config.mean;
  cnn_config.std = classifier_config.std;
  return cnn_config;
}

TrafficLightClassifier make_classifier(
  const TrafficLightRecognitionConfig & config, const ClassifierModelConfig & classifier_config,
  const uint8_t traffic_light_type)
{
  auto backend = std::make_shared<CNNClassifier>(make_cnn_config(classifier_config));
  return TrafficLightClassifier(
    std::move(backend), traffic_light_type, config.over_exposure_threshold,
    config.under_exposure_threshold);
}
}  // namespace

void build_engines(const TrafficLightRecognitionConfig & config)
{
  [[maybe_unused]] autoware::tensorrt_yolox::TrtYoloXDetector whole_image_detector(
    make_whole_image_detector_config(config));
  [[maybe_unused]] auto car_classifier = make_classifier(
    config, config.car_classifier, tier4_perception_msgs::msg::TrafficLight::CAR_TRAFFIC_LIGHT);
  [[maybe_unused]] auto pedestrian_classifier = make_classifier(
    config, config.pedestrian_classifier,
    tier4_perception_msgs::msg::TrafficLight::PEDESTRIAN_TRAFFIC_LIGHT);
}

TrafficLightRecognition::TrafficLightRecognition(
  const TrafficLightRecognitionConfig & config, const tf2::BufferCore & tf_buffer)
: map_based_detector_config_(make_map_based_detector_config(config)),
  whole_image_detector_(make_whole_image_detector_config(config)),
  car_classifier_(make_classifier(
    config, config.car_classifier, tier4_perception_msgs::msg::TrafficLight::CAR_TRAFFIC_LIGHT)),
  pedestrian_classifier_(make_classifier(
    config, config.pedestrian_classifier,
    tier4_perception_msgs::msg::TrafficLight::PEDESTRIAN_TRAFFIC_LIGHT)),
  diagnostics_node_name_(config.diagnostics_node_name),
  tf_buffer_(tf_buffer)
{
}

void TrafficLightRecognition::set_map(const autoware_map_msgs::msg::LaneletMapBin & map_msg)
{
  map_based_detector_.emplace(map_based_detector_config_, map_msg);
}

tl::expected<void, std::string> TrafficLightRecognition::set_route(
  const autoware_planning_msgs::msg::LaneletRoute & route_msg)
{
  if (!map_based_detector_) {
    return tl::make_unexpected(std::string("vector map is not set yet"));
  }

  const auto error = map_based_detector_->set_route(route_msg);
  if (error) {
    return tl::make_unexpected(error->message);
  }
  return {};
}

tl::expected<TrafficLightRecognitionResult, std::string> TrafficLightRecognition::run(
  const sensor_msgs::msg::Image & image, const sensor_msgs::msg::CameraInfo & camera_info)
{
  if (!map_based_detector_) {
    return tl::make_unexpected(std::string("vector map is not set yet"));
  }

  const auto map_based_result = map_based_detector_->detect(tf_buffer_, camera_info);
  if (map_based_result.expect_rois.rois.empty()) {
    return make_empty_result(diagnostics_node_name_, image.header);
  }

  const auto detected = whole_image_detector_.detect(image);
  if (!detected) {
    return tl::make_unexpected("whole_image_detector failed: " + detected.error());
  }

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
  result.diagnostics = make_exposure_diagnostics(
    diagnostics_node_name_, image.header.stamp,
    car_result->detected_over_exposure || pedestrian_result->detected_over_exposure,
    car_result->detected_under_exposure || pedestrian_result->detected_under_exposure);
  return result;
}

}  // namespace autoware::traffic_light
