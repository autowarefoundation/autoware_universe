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

#include "traffic_light_recognition/traffic_light_recognition.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/image_transport_decompressor/image_transport_decompressor.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/map_projection_loader/map_projection_loader.hpp>
#include <autoware/traffic_light_multi_camera_fusion/traffic_light_multi_camera_fusion.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_filter.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2/buffer_core.h>
#include <tf2/time.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
using autoware::traffic_light::ClassifierModelConfig;
using autoware::traffic_light::MultiCameraFusion;
using autoware::traffic_light::MultiCameraFusionConfig;
using autoware::traffic_light::TrafficLightRecognition;
using autoware::traffic_light::TrafficLightRecognitionConfig;
using autoware::traffic_light::TrafficLightRecognitionResult;

// --- run config ---------------------------------------------------------------------------------

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

struct CameraConfig
{
  std::string camera_namespace;
  std::string camera_info_topic;
  std::string compressed_image_topic;
  std::string traffic_signals_topic;
  std::string rois_topic;
};

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

// The whole run: N cameras sharing one set of models/thresholds, over one input bag.
struct Bag2BagConfig
{
  std::string input_bag_path;
  std::string lanelet2_map_path;
  std::string map_projector_info_path;

  std::vector<CameraConfig> cameras;
  TrafficLightRecognitionConfig recognition;
};

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

Bag2BagConfig build_bag2bag_config(
  const std::string & input_bag_path, const std::string & map_path,
  const std::vector<int> & camera_indices, const std::string & ml_model_path,
  const std::string & config_path)
{
  Bag2BagConfig config;
  config.input_bag_path = input_bag_path;
  config.lanelet2_map_path = find_lanelet2_map(map_path);
  config.map_projector_info_path = find_map_projector_info(map_path);

  config.recognition = build_recognition_config(
    ml_model_path.empty() ? default_ml_model_path() : ml_model_path,
    config_path.empty() ? default_recognition_config_path() : config_path);

  for (const auto camera_index : camera_indices) {
    config.cameras.push_back(build_camera_config(camera_index));
  }
  return config;
}

// --- rosbag / map input
// ------------------------------------------------------------------------------

template <typename MessageT>
MessageT deserialize(const rosbag2_storage::SerializedBagMessageSharedPtr & bag_message)
{
  rclcpp::SerializedMessage serialized_message(*bag_message->serialized_data);
  rclcpp::Serialization<MessageT> serialization;
  MessageT message;
  serialization.deserialize_message(&serialized_message, &message);
  return message;
}

int64_t stamp_nanoseconds(const std_msgs::msg::Header & header)
{
  return rclcpp::Time(header.stamp).nanoseconds();
}

// One exact-stamp matched (image, camera_info) pair, tagged with the camera it came from.
struct Frame
{
  std::size_t camera_index;
  sensor_msgs::msg::CompressedImage image;
  sensor_msgs::msg::CameraInfo camera_info;
};

// One camera's images/camera_infos keyed by header stamp while the bag is being read, so pairing
// does not depend on how the two topics happened to interleave on disk. Images are kept
// compressed, as read.
struct CameraBuffers
{
  std::map<int64_t, sensor_msgs::msg::CompressedImage> images_by_stamp;
  std::map<int64_t, sensor_msgs::msg::CameraInfo> camera_infos_by_stamp;
};

std::vector<Frame> load_frames_for_camera(const Bag2BagConfig & config, std::size_t camera_index)
{
  const auto & camera = config.cameras.at(camera_index);

  rosbag2_cpp::Reader reader;
  reader.open(config.input_bag_path);
  reader.set_filter(
    rosbag2_storage::StorageFilter{{camera.compressed_image_topic, camera.camera_info_topic}});

  CameraBuffers buffers;
  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    if (bag_message->topic_name == camera.camera_info_topic) {
      auto camera_info = deserialize<sensor_msgs::msg::CameraInfo>(bag_message);
      buffers.camera_infos_by_stamp.emplace(
        stamp_nanoseconds(camera_info.header), std::move(camera_info));
    } else {
      auto compressed_image = deserialize<sensor_msgs::msg::CompressedImage>(bag_message);
      buffers.images_by_stamp.emplace(
        stamp_nanoseconds(compressed_image.header), std::move(compressed_image));
    }
  }

  // buffers.images_by_stamp is already stamp-sorted (std::map), so this preserves ascending order.
  std::vector<Frame> frames;
  frames.reserve(buffers.images_by_stamp.size());
  for (auto & [stamp, image] : buffers.images_by_stamp) {
    auto camera_info_iter = buffers.camera_infos_by_stamp.find(stamp);
    if (camera_info_iter == buffers.camera_infos_by_stamp.end()) {
      continue;
    }
    frames.push_back(Frame{camera_index, std::move(image), std::move(camera_info_iter->second)});
  }
  return frames;
}

// The Node gets its map->camera transforms from a tf2_ros::TransformListener; here they all come
// from the input bag instead.
std::unique_ptr<tf2::BufferCore> load_transform_buffer(const std::string & bag_path)
{
  auto buffer = std::make_unique<tf2::BufferCore>(tf2::durationFromSec(24 * 60 * 60));

  rosbag2_cpp::Reader reader;
  reader.open(bag_path);
  reader.set_filter(rosbag2_storage::StorageFilter{{"/tf", "/tf_static"}});
  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    const bool is_static = bag_message->topic_name == "/tf_static";
    const auto message = deserialize<tf2_msgs::msg::TFMessage>(bag_message);
    for (const auto & transform : message.transforms) {
      buffer->setTransform(transform, "traffic_light_pipeline_bag2bag", is_static);
    }
  }
  return buffer;
}

// Loads `config.lanelet2_map_path` (must be an MGRS-projected map, per
// `config.map_projector_info_path`) as a LaneletMapBin.
autoware_map_msgs::msg::LaneletMapBin load_map(const Bag2BagConfig & config)
{
  const auto projector_info =
    autoware::map_projection_loader::load_info_from_yaml(config.map_projector_info_path);
  if (projector_info.projector_type != autoware_map_msgs::msg::MapProjectorInfo::MGRS) {
    throw std::runtime_error(config.map_projector_info_path + " is not an MGRS projector");
  }
  const auto map =
    autoware::experimental::lanelet2_utils::load_mgrs_coordinate_map(config.lanelet2_map_path);
  return autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
}

// --- back-end config ----------------------------------------------------------------------------

// Where the fused result is written: production's own topic name for multi_camera_fusion's output.
constexpr char kFusionOutputTopic[] =
  "/perception/traffic_light_recognition/internal/traffic_signals";

MultiCameraFusionConfig fusion_config(const autoware_map_msgs::msg::LaneletMapBin & map_msg)
{
  MultiCameraFusionConfig config;
  config.message_lifespan = 0.12;
  config.prior_log_odds = 0.0;
  config.use_signal_consistency_check = false;
  config.publish_partial_matched_signal = false;
  config.use_map_based_signal_filter = false;

  // The same round trip MultiCameraFusionNode's map_callback() does.
  config.lanelet_map_ptr = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(map_msg));

  return config;
}

// --- pass A: front-end --------------------------------------------------------------------------

// One front-end run() result, kept together with the camera it came from and the camera_info it
// was produced from -- the back-end pass needs the latter (fuse()'s first argument) without
// re-reading the bag.
struct RecordedFrameResult
{
  std::size_t camera_index;
  sensor_msgs::msg::CameraInfo camera_info;
  TrafficLightRecognitionResult result;
};

void sort_by_stamp_and_camera(std::vector<RecordedFrameResult> & recorded_results)
{
  std::stable_sort(
    recorded_results.begin(), recorded_results.end(),
    [](const RecordedFrameResult & lhs, const RecordedFrameResult & rhs) {
      const auto lhs_stamp = rclcpp::Time(lhs.camera_info.header.stamp).nanoseconds();
      const auto rhs_stamp = rclcpp::Time(rhs.camera_info.header.stamp).nanoseconds();
      return std::tie(lhs_stamp, lhs.camera_index) < std::tie(rhs_stamp, rhs.camera_index);
    });
}

std::vector<RecordedFrameResult> run_recognition_for_camera(
  const Bag2BagConfig & config, const autoware_map_msgs::msg::LaneletMapBin & map_msg,
  tf2::BufferCore & tf_buffer, std::size_t camera_index)
{
  const auto & camera = config.cameras[camera_index];
  TrafficLightRecognition recognition(config.recognition, tf_buffer);
  recognition.set_map(map_msg);

  const auto frames = load_frames_for_camera(config, camera_index);
  std::cerr << "loaded " << frames.size() << " frames for camera " << camera.camera_namespace
            << " from " << config.input_bag_path << std::endl;

  std::vector<RecordedFrameResult> recorded_results;
  recorded_results.reserve(frames.size());
  for (const auto & frame : frames) {
    const auto image = autoware::image_preprocessor::image_transport_decompressor::decompress(
      frame.image, "default");
    const auto result = recognition.run(image, frame.camera_info);
    if (!result) {
      std::cerr << "camera " << camera.camera_namespace << " frame at "
                << rclcpp::Time(image.header.stamp).nanoseconds() << " failed: " << result.error()
                << std::endl;
      continue;
    }
    recorded_results.push_back({camera_index, frame.camera_info, *result});
  }
  return recorded_results;
}

std::vector<RecordedFrameResult> run_recognition(
  const Bag2BagConfig & config, const autoware_map_msgs::msg::LaneletMapBin & map_msg)
{
  const auto tf_buffer = load_transform_buffer(config.input_bag_path);

  std::vector<RecordedFrameResult> recorded_results;
  for (std::size_t camera_index = 0; camera_index < config.cameras.size(); ++camera_index) {
    const auto camera_results =
      run_recognition_for_camera(config, map_msg, *tf_buffer, camera_index);
    recorded_results.insert(recorded_results.end(), camera_results.begin(), camera_results.end());
  }

  sort_by_stamp_and_camera(recorded_results);
  return recorded_results;
}

// --- pass B: back-end ---------------------------------------------------------------------------

// One MultiCameraFusion, fed pass A's results in ascending (stamp, camera_index) order -- matching
// production's per-trigger arrival order -- required because MultiCameraFusion is stateful (its
// record_arr_set_ keeps every record within message_lifespan of the newest one). fuse() reports no
// errors; the ids it could not find in the map are logged, as the Node warns about them.
std::vector<autoware_perception_msgs::msg::TrafficLightGroupArray> run_fusion(
  const MultiCameraFusionConfig & config,
  const std::vector<RecordedFrameResult> & recorded_frame_results)
{
  MultiCameraFusion multi_camera_fusion(config);

  std::vector<autoware_perception_msgs::msg::TrafficLightGroupArray> recorded_fusion_results;
  recorded_fusion_results.reserve(recorded_frame_results.size());
  for (const auto & recorded : recorded_frame_results) {
    const auto result = multi_camera_fusion.fuse(
      recorded.camera_info, recorded.result.selected_rois, recorded.result.merged_signals);
    recorded_fusion_results.push_back(result.traffic_light_groups);
  }
  return recorded_fusion_results;
}

// --- pass C: rosbag output ----------------------------------------------------------------------

// The output bag is always written in the same storage format as the input bag: open the input bag
// with rosbag2_cpp's auto-detecting Reader and read back whichever storage plugin id it detected
// from the bag's own metadata.yaml.
std::string detect_input_bag_storage_id(const std::string & bag_path)
{
  rosbag2_cpp::Reader reader;
  reader.open(bag_path);
  return reader.get_metadata().storage_identifier;
}

// Guards remove_output_bag_if_exists() against deleting anything that is not actually a rosbag2
// bag directory. Throws if a stray entry is found, so a mistyped --output-bag path never silently
// wipes out an unrelated directory.
void check_only_contains_rosbag_files(const std::string & bag_path)
{
  for (const auto & entry : std::filesystem::directory_iterator(bag_path)) {
    const auto & extension = entry.path().extension();
    if (
      entry.is_directory() ||
      (extension != ".db3" && extension != ".mcap" && extension != ".yaml")) {
      throw std::runtime_error(
        "refusing to overwrite " + bag_path + ": unexpected entry " + entry.path().string() +
        " (expected only .db3 / .mcap / .yaml files -- is --output-bag pointing at the right "
        "directory?)");
    }
  }
}

// Removes `output_bag_path` if it already exists, so a Writer can always create it fresh.
void remove_output_bag_if_exists(const std::string & output_bag_path)
{
  if (!std::filesystem::exists(output_bag_path)) {
    return;
  }
  if (!std::filesystem::is_directory(output_bag_path)) {
    throw std::runtime_error(
      "refusing to overwrite " + output_bag_path + ": it is not a directory");
  }
  check_only_contains_rosbag_files(output_bag_path);
  std::filesystem::remove_all(output_bag_path);
}

void write_to_rosbag(
  const Bag2BagConfig & config, const std::string & output_bag_path,
  const std::vector<RecordedFrameResult> & recorded_frame_results,
  const std::vector<autoware_perception_msgs::msg::TrafficLightGroupArray> &
    recorded_fusion_results)
{
  remove_output_bag_if_exists(output_bag_path);

  rosbag2_cpp::Writer writer;
  writer.open({output_bag_path, detect_input_bag_storage_id(config.input_bag_path)});

  for (const auto & recorded : recorded_frame_results) {
    const auto & camera = config.cameras[recorded.camera_index];
    const rclcpp::Time stamp(recorded.result.merged_signals.header.stamp);
    writer.write(recorded.result.merged_signals, camera.traffic_signals_topic, stamp);
    writer.write(recorded.result.selected_rois, camera.rois_topic, stamp);
  }
  for (const auto & fusion_result : recorded_fusion_results) {
    writer.write(fusion_result, kFusionOutputTopic, rclcpp::Time(fusion_result.stamp));
  }
}

// --- entry point --------------------------------------------------------------------------------

struct CommandLineArgs
{
  std::string input_bag_path;
  std::string map_path;
  std::string output_bag_path;
  std::vector<int> camera_indices;
  std::string ml_model_path;
  std::string config_path;
};

// Parses `--camera 4,5` (or `--camera 4 --camera 5`) into {4, 5}: the numeric part of each
// camera's namespace, which is all the topic names need (see build_camera_config()).
std::vector<int> parse_camera_indices(const std::string & value)
{
  std::vector<int> camera_indices;
  std::stringstream stream(value);
  std::string token;
  while (std::getline(stream, token, ',')) {
    if (token.empty() || token.find_first_not_of("0123456789") != std::string::npos) {
      throw std::runtime_error(
        "--camera takes comma-separated camera numbers (e.g. 4,5), but got '" + token + "'");
    }
    camera_indices.push_back(std::stoi(token));
  }
  return camera_indices;
}

CommandLineArgs parse_args(int argc, char ** argv)
{
  // `--flag value` and `--flag=value` are both accepted, so neither habit trips over the other.
  const auto split_flag = [](const std::string & arg) {
    const auto separator = arg.find('=');
    return separator == std::string::npos ? std::pair<std::string, std::string>{arg, {}}
                                          : std::pair<std::string, std::string>{
                                              arg.substr(0, separator), arg.substr(separator + 1)};
  };

  CommandLineArgs args;
  for (int i = 1; i < argc; ++i) {
    auto [flag, value] = split_flag(argv[i]);
    if (value.empty()) {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + flag);
      }
      value = argv[++i];
    }

    if (flag == "--input-bag") {
      args.input_bag_path = value;
    } else if (flag == "--map") {
      args.map_path = value;
    } else if (flag == "--output-bag") {
      args.output_bag_path = value;
    } else if (flag == "--camera") {
      const auto camera_indices = parse_camera_indices(value);
      args.camera_indices.insert(
        args.camera_indices.end(), camera_indices.begin(), camera_indices.end());
    } else if (flag == "--ml-model-path") {
      args.ml_model_path = value;
    } else if (flag == "--config") {
      args.config_path = value;
    } else {
      throw std::runtime_error("unknown argument " + flag);
    }
  }
  if (
    args.input_bag_path.empty() || args.map_path.empty() || args.output_bag_path.empty() ||
    args.camera_indices.empty()) {
    throw std::runtime_error(
      "usage: traffic_light_pipeline_bag2bag_runner --input-bag <path> --map <dir> "
      "--output-bag <path> --camera <numbers, e.g. 4,5> [--ml-model-path <dir>] "
      "[--config <param.yaml>]");
  }
  return args;
}

void run_bag2bag(const CommandLineArgs & args)
{
  const auto config = build_bag2bag_config(
    args.input_bag_path, args.map_path, args.camera_indices, args.ml_model_path, args.config_path);
  const auto map_msg = load_map(config);

  const auto recorded_frame_results = run_recognition(config, map_msg);
  const auto recorded_fusion_results = run_fusion(fusion_config(map_msg), recorded_frame_results);

  write_to_rosbag(config, args.output_bag_path, recorded_frame_results, recorded_fusion_results);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    run_bag2bag(parse_args(argc, argv));
  } catch (const std::exception & e) {
    std::cerr << "traffic_light_pipeline_bag2bag_runner failed: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
