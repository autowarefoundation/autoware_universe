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

#include "runner.hpp"

#include "traffic_light_recognition/traffic_light_recognition.hpp"

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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
using autoware::traffic_light::Bag2BagConfig;
using autoware::traffic_light::MultiCameraFusion;
using autoware::traffic_light::MultiCameraFusionConfig;
using autoware::traffic_light::TrafficLightRecognition;
using autoware::traffic_light::TrafficLightRecognitionResult;

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
  const Bag2BagConfig & config, const std::vector<RecordedFrameResult> & recorded_frame_results,
  const std::vector<autoware_perception_msgs::msg::TrafficLightGroupArray> &
    recorded_fusion_results)
{
  remove_output_bag_if_exists(config.output_bag_path);

  rosbag2_cpp::Writer writer;
  writer.open({config.output_bag_path, detect_input_bag_storage_id(config.input_bag_path)});

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

}  // namespace

namespace autoware::traffic_light
{

void run_bag2bag(const Bag2BagConfig & config)
{
  const auto map_msg = load_map(config);

  const auto recorded_frame_results = run_recognition(config, map_msg);
  const auto recorded_fusion_results = run_fusion(fusion_config(map_msg), recorded_frame_results);

  write_to_rosbag(config, recorded_frame_results, recorded_fusion_results);
}

}  // namespace autoware::traffic_light
