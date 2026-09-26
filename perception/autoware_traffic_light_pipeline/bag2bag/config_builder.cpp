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

#include "config_builder.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using autoware::traffic_light::Bag2BagConfig;
using autoware::traffic_light::CameraConfig;
using autoware::traffic_light::ClassifierModelConfig;
using autoware::traffic_light::CommandLineArgs;
using autoware::traffic_light::TrafficLightRecognitionConfig;

YAML::Node require(const YAML::Node & node, const std::string & key)
{
  const auto child = node[key];
  if (!child) {
    throw std::runtime_error("recognition config: missing required key '" + key + "'");
  }
  return child;
}

std::string default_recognition_config_path()
{
  return ament_index_cpp::get_package_share_directory("autoware_traffic_light_pipeline") +
         "/config/traffic_light_recognition.param.yaml";
}

YAML::Node recognition_parameters(const std::string & config_path)
{
  if (!std::filesystem::exists(config_path)) {
    throw std::runtime_error("recognition config does not exist: " + config_path);
  }
  const auto root = YAML::LoadFile(config_path);
  return require(require(root, "/**"), "ros__parameters");
}

std::string default_roi_remap_path()
{
  return ament_index_cpp::get_package_share_directory("autoware_tensorrt_yolox") +
         "/config/traffic_light_roi_label_remap.csv";
}

std::string resolve_artifact(
  const YAML::Node & parameters, const std::string & ml_model_path, const std::string & section,
  const std::string & key)
{
  const auto relative_path = require(require(parameters, section), key).as<std::string>();
  return ml_model_path.empty() ? relative_path : ml_model_path + "/" + relative_path;
}

std::string default_ml_model_path()
{
  const char * home = std::getenv("HOME");
  if (!home) {
    throw std::runtime_error(
      "--ml-model-path was not given and $HOME is not set, so its default $HOME/autoware_data "
      "cannot be resolved");
  }
  return std::string(home) + "/autoware_data";
}

ClassifierModelConfig read_classifier_config(
  const YAML::Node & parameters, const std::string & ml_model_path, const std::string & section)
{
  const auto classifier = require(parameters, section);

  ClassifierModelConfig classifier_config;
  classifier_config.model_path = resolve_artifact(parameters, ml_model_path, section, "model_path");
  classifier_config.label_path = resolve_artifact(parameters, ml_model_path, section, "label_path");
  classifier_config.precision = require(classifier, "precision").as<std::string>();
  classifier_config.mean = require(classifier, "mean").as<std::vector<float>>();
  classifier_config.std = require(classifier, "std").as<std::vector<float>>();
  return classifier_config;
}

TrafficLightRecognitionConfig build_recognition_config(
  const std::string & ml_model_path, const std::string & config_path)
{
  const auto parameters = recognition_parameters(config_path);
  const auto detector = require(parameters, "whole_image_detector");
  const auto map_based_detector = require(parameters, "map_based_detector");
  const auto classifier = require(parameters, "classifier");

  TrafficLightRecognitionConfig config;

  config.whole_image_detector_model_path =
    resolve_artifact(parameters, ml_model_path, "whole_image_detector", "model_path");
  config.whole_image_detector_label_path =
    resolve_artifact(parameters, ml_model_path, "whole_image_detector", "label_path");
  config.whole_image_detector_roi_remap_path = default_roi_remap_path();
  config.whole_image_detector_score_threshold =
    static_cast<float>(require(detector, "score_threshold").as<double>());
  config.whole_image_detector_nms_threshold =
    static_cast<float>(require(detector, "nms_threshold").as<double>());
  config.whole_image_detector_precision = require(detector, "precision").as<std::string>();

  config.min_timestamp_offset = require(map_based_detector, "min_timestamp_offset").as<double>();
  config.max_timestamp_offset = require(map_based_detector, "max_timestamp_offset").as<double>();

  config.car_classifier = read_classifier_config(parameters, ml_model_path, "car_classifier");
  config.pedestrian_classifier =
    read_classifier_config(parameters, ml_model_path, "pedestrian_classifier");

  config.over_exposure_threshold = require(classifier, "over_exposure_threshold").as<double>();
  config.under_exposure_threshold = require(classifier, "under_exposure_threshold").as<double>();

  // The Node passes its own name; this tool has no Node, so it reuses the production node name so
  // that the recorded diagnostics carry the same hardware_id as a live run.
  config.diagnostics_node_name = "traffic_light_recognition";

  return config;
}

CameraConfig build_camera_config(int camera_index)
{
  const auto camera_namespace = "camera" + std::to_string(camera_index);

  CameraConfig camera;
  camera.camera_namespace = camera_namespace;
  camera.camera_info_topic = "/sensing/camera/" + camera_namespace + "/camera_info";
  camera.compressed_image_topic = "/sensing/camera/" + camera_namespace + "/image_raw/compressed";
  camera.traffic_signals_topic =
    "/perception/traffic_light_recognition/" + camera_namespace + "/classification/traffic_signals";
  camera.rois_topic =
    "/perception/traffic_light_recognition/" + camera_namespace + "/detection/rois";
  return camera;
}

std::string find_lanelet2_map(const std::string & map_path)
{
  std::vector<std::string> found_paths;
  for (const auto & entry : std::filesystem::directory_iterator(map_path)) {
    if (entry.is_regular_file() && entry.path().extension() == ".osm") {
      found_paths.push_back(entry.path().string());
    }
  }
  if (found_paths.size() != 1) {
    throw std::runtime_error(
      "expected exactly one .osm file in " + map_path + ", but found " +
      std::to_string(found_paths.size()));
  }
  return found_paths.front();
}

std::string find_map_projector_info(const std::string & map_path)
{
  const auto path = map_path + "/map_projector_info.yaml";
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("no map_projector_info.yaml in " + map_path);
  }
  return path;
}

}  // namespace

namespace autoware::traffic_light
{

Bag2BagConfig build_bag2bag_config(const CommandLineArgs & args)
{
  Bag2BagConfig config;
  config.input_bag_path = args.input_bag_path;
  config.output_bag_path = args.output_bag_path;
  config.lanelet2_map_path = find_lanelet2_map(args.map_path);
  config.map_projector_info_path = find_map_projector_info(args.map_path);

  config.recognition = build_recognition_config(
    args.ml_model_path.empty() ? default_ml_model_path() : args.ml_model_path,
    args.config_path.empty() ? default_recognition_config_path() : args.config_path);

  for (const auto camera_index : args.camera_indices) {
    config.cameras.push_back(build_camera_config(camera_index));
  }
  return config;
}

}  // namespace autoware::traffic_light
