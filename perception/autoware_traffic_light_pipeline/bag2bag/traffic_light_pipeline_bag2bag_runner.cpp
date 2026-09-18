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

// Bag-to-bag runner for the traffic light pipeline: a recorded rosbag in, a rosbag of the
// pipeline's own results out, as fast as the machine can go. It drives this package's ROS-free
// front-end core (TrafficLightRecognition) followed by autoware_traffic_light_multi_camera_fusion's
// MultiCameraFusion, with no rclcpp::init, no executor and no DDS anywhere -- so the run is not
// bound to the bag's recorded rate and always produces the same output for the same input bag.
//
// The back-end here is MultiCameraFusion alone. Production continues with arbiter ->
// crosswalk_traffic_light_estimator, but this package composes no fusion Node yet, so
// `multi_camera_fusion.fuse()` is where the chain stops: its output is production's
// /perception/traffic_light_recognition/internal/traffic_signals.
//
// Three passes:
//   A. front-end, one camera at a time. For each camera it loads only that camera's
//      (image, camera_info) pairs out of the input bag (load_frames_for_camera()), drives a
//      fresh TrafficLightRecognition over them, then discards those frames before moving to the
//      next camera -- so memory stays proportional to one camera's frames rather than every
//      camera's combined. The front-end has no cross-camera state, so this changes nothing about
//      any individual result; the per-camera results are then sorted back into ascending
//      (stamp, camera_index) order.
//   B. back-end. One MultiCameraFusion instance, fed pass A's results in that order. Splitting the
//      two passes rather than interleaving them frame by frame (the way the Node graph does) is
//      safe because MultiCameraFusion is stateful but never reads the clock: every timestamp it
//      acts on comes from the trigger's own camera_info/roi header, so replaying the same input
//      sequence in the same order through a fresh instance produces identical output.
//   C. rosbag output, under production topic names.
//
// Usage:
//   traffic_light_pipeline_bag2bag_runner
//     --input-bag <input bag dir>
//     --map <map dir, holding one .osm map and a map_projector_info.yaml>
//     --output-bag <output bag dir>
//     --camera 4,5                     (the cameras the input bag recorded)
//     [--ml-model-path <dir>]          (default $HOME/autoware_data)
//     [--config <param.yaml>]          (default this package's installed
//                                       config/traffic_light_recognition.param.yaml)
//
// The runner has no config file of its own: the front-end's tuning comes from the very file the
// Node is launched with, this package's config/traffic_light_recognition.param.yaml, so the
// default run reproduces the deployed configuration (see build_bag2bag_config()). --config swaps
// in another copy of that same file, for sweeping a threshold without editing the installed one.
// The topic names follow from the camera namespaces, and the back-end's settings are the deployed
// constants below.

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
#include <optional>
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

// Returns `node[key]`, throwing if it is missing -- the recognition config is expected to carry
// every value the Node declares, so a missing one is a broken config file rather than a default.
YAML::Node require(const YAML::Node & node, const std::string & key)
{
  const auto child = node[key];
  if (!child) {
    throw std::runtime_error("recognition config: missing required key '" + key + "'");
  }
  return child;
}

// This package's installed config/traffic_light_recognition.param.yaml -- the very file
// launch/traffic_light_recognition.launch.xml passes the Node as `<param from="..."/>`. Used
// whenever --config is not given, so that the default run measures the deployed configuration.
std::string default_recognition_config_path()
{
  return ament_index_cpp::get_package_share_directory("autoware_traffic_light_pipeline") +
         "/config/traffic_light_recognition.param.yaml";
}

// The `/**: ros__parameters:` block of the file above, or of the --config file, which has to have
// the same layout because it is meant to be a copy of it. Read with yaml-cpp rather than rcl's
// yaml parser because nothing here is a Node: every value the Node declares is a plain scalar
// under that one block, so the two parsers see the same thing.
YAML::Node recognition_parameters(const std::string & config_path)
{
  // yaml-cpp reports a missing file as a bare "bad file", which says nothing about a path the
  // caller may have mistyped on the command line.
  if (!std::filesystem::exists(config_path)) {
    throw std::runtime_error("recognition config does not exist: " + config_path);
  }
  const auto root = YAML::LoadFile(config_path);
  return require(require(root, "/**"), "ros__parameters");
}

// The whole-image detector ships this remap csv as installed package data, so unlike the model /
// label files (which live under the user's ML artifact directory) it is not configurable at all:
// it is resolved from autoware_tensorrt_yolox's own share directory, the same file the production
// launch file defaults to via $(find-pkg-share autoware_tensorrt_yolox). The Node declares the
// parameter with no default for a reason -- an empty remap leaves every detector label unmapped,
// which makes TrtYoloXDetector discard every detection.
std::string default_roi_remap_path()
{
  return ament_index_cpp::get_package_share_directory("autoware_tensorrt_yolox") +
         "/config/traffic_light_roi_label_remap.csv";
}

// `<ml_model_path>/<the relative name in the package config>`, mirroring the Node's
// resolve_artifact(): the package config names the artifacts relative to the directory the launch
// file injects as `ml_model_path`, so the same join has to happen here.
std::string resolve_artifact(
  const YAML::Node & parameters, const std::string & ml_model_path, const std::string & section,
  const std::string & key)
{
  const auto relative_path = require(require(parameters, section), key).as<std::string>();
  return ml_model_path.empty() ? relative_path : ml_model_path + "/" + relative_path;
}

// $HOME/autoware_data, the launch file's `data_path` default.
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

// One classifier's model/label/precision/normalization, mirroring the Node's
// declare_classifier_config(). `section` is "car_classifier" or "pedestrian_classifier".
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

// Every tuned value of the front-end, read from `config_path`; only the artifact directory comes
// from the command line. See build_bag2bag_config().
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

// One camera's topics, all four derived from its namespace the same way the production topic graph
// derives them, so nothing has to be configured:
//   - the two inputs are launch/traffic_light_recognition.launch.xml's `input/image` and
//     `input/camera_info` defaults (`/sensing/camera/<ns>/...`), with the compressed variant of
//     the image the input bag records rather than the raw one the Node subscribes to;
//   - the two outputs are its `output/traffic_signals` and `output/rois` defaults, which are in
//     turn the names traffic_light_multi_camera_fusion derives from its `camera_namespaces`
//     parameter.
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
  // The two files found in the map directory; see find_lanelet2_map() /
  // find_map_projector_info().
  std::string lanelet2_map_path;
  std::string map_projector_info_path;

  std::vector<CameraConfig> cameras;
  TrafficLightRecognitionConfig recognition;
};

// The one .osm file directly inside `map_path`: a map directory holds a single lanelet2 map, but
// its name varies, so it is found by extension rather than assumed. Anything else -- none, or
// several -- is reported rather than resolved by picking one, which would silently run against a
// map the caller did not mean.
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

// The projector info, by name rather than by extension: `map_projector_info.yaml` is the name
// autoware_map_projection_loader itself expects, and a map directory holds other yaml files too
// (pointcloud_map_metadata.yaml, ...), so the extension does not identify it.
std::string find_map_projector_info(const std::string & map_path)
{
  const auto path = map_path + "/map_projector_info.yaml";
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("no map_projector_info.yaml in " + map_path);
  }
  return path;
}

// Builds the whole run's configuration from the four things that vary per run -- the input bag, the
// map directory, the cameras the bag recorded (`camera_indices`, e.g. {4, 5} for camera4/camera5)
// and the directory the ML artifacts live in -- and this package's own
// config/traffic_light_recognition.param.yaml for everything else.
//
// There is no config file of the runner's own on purpose. Every tuned value comes from a config
// in the Node's own format -- by default the very file launch/traffic_light_recognition.launch.xml
// feeds the Node -- so a bag-to-bag run reproduces the deployed configuration and never states a
// threshold of its own. An empty `config_path` selects that default; anything else has to be a
// copy of it, which is how a threshold is swept without editing the installed file. What is left
// is derivable: the topic names follow from each camera's namespace exactly as the launch file's
// own defaults do (see build_camera_config()).
//
// `ml_model_path` is the launch file's `data_path` argument -- a property of the machine the run
// happens on rather than of the pipeline's tuning, which is why the package config names the
// artifacts relative to it instead of carrying it. An empty string means $HOME/autoware_data, the
// launch file's default.
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

// One exact-stamp matched (image, camera_info) pair, tagged with the camera it came from -- the
// same input unit the Node's message_filters::ExactTime sync hands to run(). `image` is kept
// compressed, as it was read from the bag: decoding every frame up front, for the whole bag, is
// what makes frame loading the dominant memory cost of a run. Call decode_frame_image() instead,
// right before handing the frame to the pipeline.
struct Frame
{
  std::size_t camera_index;
  sensor_msgs::msg::CompressedImage image;
  sensor_msgs::msg::CameraInfo camera_info;
};

// One camera's images/camera_infos keyed by header stamp while the bag is being read, so pairing
// does not depend on how the two topics happened to interleave on disk. Images are kept
// compressed, as read -- see Frame.
struct CameraBuffers
{
  std::map<int64_t, sensor_msgs::msg::CompressedImage> images_by_stamp;
  std::map<int64_t, sensor_msgs::msg::CameraInfo> camera_infos_by_stamp;
};

// Reads only `config.cameras[camera_index]`'s image/camera_info topics out of
// `config.input_bag_path` and returns that camera's exact-stamp matched frames, in ascending stamp
// order. Messages with no same-stamp partner on the other topic are dropped -- the same policy
// message_filters::ExactTime enforces in production. Reading one camera at a time -- call this,
// process that camera's frames, then let them go out of scope before moving to the next camera --
// keeps memory proportional to one camera's frame count rather than the whole bag's.
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

// Decodes `frame.image`, returning the plain image the pipeline consumes. Meant to be called right
// before that -- one frame at a time, as it is about to be processed -- rather than while frames
// are being buffered, so at most one decoded image is ever held in memory.
std::optional<sensor_msgs::msg::Image> decode_frame_image(const Frame & frame)
{
  // decompress() reports an undecodable payload by throwing, leaving the policy to its caller
  // (see its own doc comment). Here that policy is the Node's: log the frame and drop it.
  try {
    return autoware::image_preprocessor::image_transport_decompressor::decompress(
      frame.image, "default");
  } catch (const std::exception & e) {
    std::cerr << "failed to decompress image at " << stamp_nanoseconds(frame.image.header) << ": "
              << e.what() << std::endl;
    return std::nullopt;
  }
}

// The Node gets its map->camera transforms from a tf2_ros::TransformListener; here they all come
// from the input bag instead. The cache time is deliberately far longer than tf2::BufferCore's
// 10 s default: the whole bag's transforms must stay resolvable for the whole run, since frames
// are processed after the bag has been fully read.
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

// The back-end's settings, as constants rather than as configuration. Unlike the front-end's, none
// of them can come from a package config file: this package composes no fusion Node, so it has no
// config/*.param.yaml that a launch file would feed one. Nor can they be read from
// autoware_traffic_light_multi_camera_fusion's own package config, whose message_lifespan (0.09) is
// not the deployed value -- autoware_launch overrides it to 0.12. So the deployed values are stated
// here, where the reason they are what they are can be stated with them.
MultiCameraFusionConfig fusion_config(const autoware_map_msgs::msg::LaneletMapBin & map_msg)
{
  MultiCameraFusionConfig config;

  // The deployed value
  // (autoware_launch/config/perception/traffic_light_recognition/traffic_light_multi_camera_fusion/traffic_light_multi_camera_fusion.param.yaml).
  // It must stay strictly greater than the camera period (0.100005 s on x2): fuse() drops every
  // record older than message_lifespan relative to the newest one, so at the package default of
  // 0.09 the previous cycle's other-camera record is always already stale and each fusion trigger
  // sees only its own camera -- i.e. half the output becomes monocular, losing whichever light only
  // the other camera can see.
  config.message_lifespan = 0.12;
  config.prior_log_odds = 0.0;

  // MultiCameraFusionNode declares these three, but no x2 param file sets any of them, so all
  // three keep the package default (disabled).
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

// Loads the tf buffer, then drives one TrafficLightRecognition per camera, one camera at a time
// (see pass A in this file's header comment). Results come back grouped by camera rather than
// interleaved by stamp, so they are sorted into ascending (stamp, camera_index) order before being
// returned: pass B (run_fusion(), stateful) needs the order production feeds it results in. Frames
// that fail are logged to stderr and skipped, exactly as the Node drops them.
std::vector<RecordedFrameResult> run_recognition(
  const Bag2BagConfig & config, const autoware_map_msgs::msg::LaneletMapBin & map_msg)
{
  const auto tf_buffer = load_transform_buffer(config.input_bag_path);

  std::vector<RecordedFrameResult> recorded_results;
  for (std::size_t camera_index = 0; camera_index < config.cameras.size(); ++camera_index) {
    const auto & camera = config.cameras[camera_index];
    TrafficLightRecognition recognition(config.recognition, *tf_buffer);
    recognition.set_map(map_msg);

    const auto frames = load_frames_for_camera(config, camera_index);
    std::cerr << "loaded " << frames.size() << " frames for camera " << camera.camera_namespace
              << " from " << config.input_bag_path << std::endl;

    for (const auto & frame : frames) {
      const auto image = decode_frame_image(frame);
      if (!image) {
        continue;
      }
      const auto result = recognition.run(*image, frame.camera_info);
      if (!result) {
        std::cerr << "camera " << camera.camera_namespace << " frame at "
                  << rclcpp::Time(image->header.stamp).nanoseconds()
                  << " failed: " << result.error() << std::endl;
        continue;
      }
      recorded_results.push_back({camera_index, frame.camera_info, *result});
    }
  }

  std::stable_sort(
    recorded_results.begin(), recorded_results.end(),
    [](const RecordedFrameResult & lhs, const RecordedFrameResult & rhs) {
      const auto lhs_stamp = rclcpp::Time(lhs.camera_info.header.stamp).nanoseconds();
      const auto rhs_stamp = rclcpp::Time(rhs.camera_info.header.stamp).nanoseconds();
      return std::tie(lhs_stamp, lhs.camera_index) < std::tie(rhs_stamp, rhs.camera_index);
    });
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

// Writes every front-end and back-end result to `output_bag_path` under production topic names.
// The input image/camera_info topics are not copied over, and neither are the cores' intermediate
// stages. Each message is written at its own header stamp (never wall-clock time), so the same
// input always produces the same bag.
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

// Runs the front-end then the back-end over every camera and writes the results to
// `args.output_bag_path`. This is the whole of main()'s work, factored out so it can be driven
// without going through argv (e.g. from tests).
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
