// Copyright 2025 TIER IV
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

#include "autoware/camera_streampetr/node.hpp"

#include "autoware/camera_streampetr/network/camera_ego_mask.hpp"

#include <Eigen/Dense>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/convert.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace autoware::camera_streampetr
{

namespace
{

// DiagnosticStatusWrapper's double overload prints large values in scientific notation,
// so format them as fixed-point strings instead.
std::string format_timestamp(const double seconds)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.9f", seconds);
  return std::string(buffer);
}

std::string format_milliseconds(const double milliseconds)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f", milliseconds);
  return std::string(buffer);
}

constexpr std::size_t MAX_CAMERA_MASK_ID = 10;

// Per-camera `cameraN/state` values published in the camera-status diagnostics; external
// monitors match on these strings, so they are part of the node's interface.
constexpr const char * CAMERA_STATE_REJECTED = "rejected";
constexpr const char * CAMERA_STATE_WAITING_CAMERA_INFO = "waiting_camera_info";
constexpr const char * CAMERA_STATE_WAITING_IMAGE = "waiting_image";
constexpr const char * CAMERA_STATE_STALE = "stale";
constexpr const char * CAMERA_STATE_ACTIVE = "active";

std::vector<int64_t> make_default_camera_mask_ids(const std::size_t rois_number)
{
  std::vector<int64_t> camera_ids;
  camera_ids.reserve(rois_number);
  for (std::size_t i = 0; i < rois_number; ++i) {
    camera_ids.push_back(static_cast<int64_t>(i));
  }
  return camera_ids;
}

void validate_mask_points(const std::vector<double> & mask, const std::string & parameter_name)
{
  if (mask.empty()) {
    return;
  }
  if (mask.size() < 6 || (mask.size() % 2) != 0) {
    throw std::runtime_error(
      parameter_name + " must contain at least 3 points to construct a mask.");
  }
}

std::vector<std::optional<EgoMaskRoiConfig>> declare_camera_mask_params(
  rclcpp::Node & node, const std::size_t rois_number, const std::array<std::uint8_t, 3> & fill_rgb,
  const std::vector<int64_t> & camera_mask_ids)
{
  std::vector<std::optional<EgoMaskRoiConfig>> camera_mask_configs(MAX_CAMERA_MASK_ID + 1);
  for (std::size_t camera_id = 0; camera_id <= MAX_CAMERA_MASK_ID; ++camera_id) {
    const std::string parameter_prefix = "camera_" + std::to_string(camera_id) + "_mask";
    const bool enabled = node.declare_parameter<bool>(parameter_prefix + ".enable", false);
    const auto mask = node.declare_parameter<std::vector<double>>(
      parameter_prefix + ".mask", std::vector<double>());
    const bool normalized = node.declare_parameter<bool>(parameter_prefix + ".normalized", false);

    if (!enabled || mask.empty()) {
      continue;
    }
    validate_mask_points(mask, parameter_prefix + ".mask");

    EgoMaskRoiConfig config;
    config.polygons.push_back(EgoMaskPolygon{mask, normalized});
    config.fill_rgb = fill_rgb;
    camera_mask_configs[camera_id] = std::move(config);
  }

  std::vector<std::optional<EgoMaskRoiConfig>> roi_mask_configs(rois_number, std::nullopt);
  for (std::size_t roi_i = 0; roi_i < rois_number; ++roi_i) {
    if (roi_i >= camera_mask_ids.size()) {
      continue;
    }
    const int64_t camera_id = camera_mask_ids[roi_i];
    if (camera_id < 0 || static_cast<std::size_t>(camera_id) >= camera_mask_configs.size()) {
      throw std::runtime_error("camera_mask.camera_ids contains an unsupported camera id.");
    }
    roi_mask_configs[roi_i] = camera_mask_configs[static_cast<std::size_t>(camera_id)];
  }

  return roi_mask_configs;
}

}  // namespace

std::vector<float> cast_to_float(const std::vector<double> & double_vector)
{
  std::vector<float> float_vector(double_vector.size());
  std::transform(
    double_vector.begin(), double_vector.end(), float_vector.begin(),
    [](double value) { return static_cast<float>(value); });
  return float_vector;
}

StreamPetrNode::StreamPetrNode(const rclcpp::NodeOptions & node_options)
: Node("autoware_camera_streampetr", node_options),
  logger_name_("camera_streampetr"),
  multithreading_(declare_parameter<bool>("multithreading")),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  rois_number_(static_cast<size_t>(declare_parameter<int>("model_params.rois_number"))),
  max_camera_time_diff_(declare_parameter<float>("max_camera_time_diff")),
  anchor_camera_id_(declare_parameter<int>("anchor_camera_id")),
  debug_mode_(declare_parameter<bool>("debug_mode"))
{
  RCLCPP_INFO(
    rclcpp::get_logger(logger_name_.c_str()), "nvinfer: %d.%d.%d\n", NV_TENSORRT_MAJOR,
    NV_TENSORRT_MINOR, NV_TENSORRT_PATCH);

  // Initialize network
  const int roi_width = declare_parameter<int>("model_params.input_image_width");
  const int roi_height = declare_parameter<int>("model_params.input_image_height");

  const std::string backbone_path = declare_parameter<std::string>("model_params.backbone_path");
  const std::string head_path = declare_parameter<std::string>("model_params.head_path");
  const std::string position_embedding_path =
    declare_parameter<std::string>("model_params.position_embedding_path");

  const std::string backbone_engine_path =
    declare_parameter<std::string>("model_params.backbone_engine_path");
  const std::string head_engine_path =
    declare_parameter<std::string>("model_params.head_engine_path");
  const std::string position_embedding_engine_path =
    declare_parameter<std::string>("model_params.position_embedding_engine_path");

  const std::string trt_precision = declare_parameter<std::string>("model_params.trt_precision");
  const uint64_t workspace_size =
    1ULL << declare_parameter<int>("model_params.workspace_size");  // Default 4GB

  const bool use_temporal = declare_parameter<bool>("model_params.use_temporal");
  const double search_distance_2d =
    declare_parameter<double>("post_process_params.iou_nms_search_distance_2d");
  const double circle_nms_dist_threshold =
    declare_parameter<double>("post_process_params.circle_nms_dist_threshold");
  const double iou_threshold = declare_parameter<double>("post_process_params.iou_nms_threshold");
  const std::vector<double> confidence_thresholds =
    declare_parameter<std::vector<double>>("post_process_params.confidence_threshold");
  const std::vector<std::string> class_names =
    declare_parameter<std::vector<std::string>>("model_params.class_names");
  const int32_t num_proposals = declare_parameter<int32_t>("model_params.num_proposals");
  const std::vector<double> yaw_norm_thresholds =
    declare_parameter<std::vector<double>>("post_process_params.yaw_norm_thresholds");
  const std::vector<float> detection_range =
    cast_to_float(declare_parameter<std::vector<double>>("model_params.detection_range"));
  const int pre_memory_length = declare_parameter<int>("model_params.pre_memory_length");
  const int post_memory_length = declare_parameter<int>("model_params.post_memory_length");

  NetworkConfig network_config{
    logger_name_,
    use_temporal,
    search_distance_2d,
    circle_nms_dist_threshold,
    iou_threshold,
    confidence_thresholds,
    class_names,
    num_proposals,
    yaw_norm_thresholds,
    detection_range,
    pre_memory_length,
    post_memory_length,
    roi_height,
    roi_width,
    static_cast<int>(rois_number_),
    workspace_size,
    trt_precision,
    backbone_path,
    head_path,
    position_embedding_path,
    backbone_engine_path,
    head_engine_path,
    position_embedding_engine_path};

  network_ = std::make_unique<StreamPetrNetwork>(network_config);

  if (declare_parameter<bool>("build_only", false)) {
    RCLCPP_INFO(
      rclcpp::get_logger(logger_name_.c_str()),
      "TensorRT engine files built successfully. Shutting Down...");
    rclcpp::shutdown();
    return;
  }

  // Setup subscriptions
  camera_info_subs_.resize(rois_number_);

  if (multithreading_) {
    RCLCPP_INFO(
      rclcpp::get_logger(logger_name_.c_str()),
      "Will be using multithreading for image callbacks.");
    camera_callback_groups_.resize(rois_number_);
  }
  const bool is_compressed_image = declare_parameter<bool>("is_compressed_image");
  const std::string image_transport_type = is_compressed_image ? "compressed" : "raw";
  camera_image_subs_.resize(rois_number_);
  for (size_t roi_i = 0; roi_i < rois_number_; ++roi_i) {
    auto sub_options = rclcpp::SubscriptionOptions();
    if (multithreading_) {
      camera_callback_groups_.at(roi_i) =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      sub_options.callback_group = camera_callback_groups_.at(roi_i);
    }

    camera_info_subs_.at(roi_i) = this->create_subscription<CameraInfo>(
      "~/input/camera" + std::to_string(roi_i) + "/camera_info",
      rclcpp::SensorDataQoS{}.keep_last(1), [this, roi_i](const CameraInfo::ConstSharedPtr msg) {
        this->camera_info_callback(msg, static_cast<int>(roi_i));
      });

    // Explicitly resolve the topic name so remapping applies to the base topic before
    // image_transport appends the transport suffix, please check
    // https://github.com/ros-perception/image_transport_plugins/issues/155
    const std::string base_topic = this->get_node_topics_interface()->resolve_topic_name(
      "~/input/camera" + std::to_string(roi_i) + "/image");
    camera_image_topics_.push_back(base_topic);
    camera_image_subs_.at(roi_i) = image_transport::create_subscription(
      this, base_topic,
      [this, roi_i](const Image::ConstSharedPtr & msg) {
        this->camera_image_callback(msg, static_cast<int>(roi_i));
      },
      image_transport_type, rmw_qos_profile_sensor_data, sub_options);
  }

  // Publishers
  pub_objects_ = this->create_publisher<DetectedObjects>("~/output/objects", rclcpp::QoS{1});

  EgoMaskParams ego_mask_params;
  ego_mask_params.enabled = declare_parameter<bool>("ego_mask.enabled", false);
  // RGB order, matching the model's input channel order and the online cameras' rgb8 encoding.
  const auto fill_value_rgb =
    declare_parameter<std::vector<double>>("ego_mask.fill_value_rgb", {0.0, 0.0, 0.0});
  for (std::size_t i = 0; i < 3; ++i) {
    const double v = i < fill_value_rgb.size() ? fill_value_rgb[i] : 0.0;
    ego_mask_params.fill_rgb[i] = static_cast<std::uint8_t>(std::clamp(v, 0.0, 255.0));
  }
  const auto camera_mask_ids = declare_parameter<std::vector<int64_t>>(
    "camera_mask.camera_ids", make_default_camera_mask_ids(rois_number_));
  ego_mask_params.roi_mask_configs =
    declare_camera_mask_params(*this, rois_number_, ego_mask_params.fill_rgb, camera_mask_ids);
  ego_mask_params.roi_polygons_yaml = declare_parameter<std::vector<std::string>>(
    "ego_mask.roi_polygons_yaml", std::vector<std::string>());

  // log the id of the masked camera
  std::string masked_camera_ids = "[";
  for (std::size_t roi_i = 0; roi_i < ego_mask_params.roi_mask_configs.size(); ++roi_i) {
    if (!ego_mask_params.roi_mask_configs[roi_i].has_value()) {
      continue;
    }
    const int64_t camera_id =
      roi_i < camera_mask_ids.size() ? camera_mask_ids[roi_i] : static_cast<int64_t>(roi_i);
    masked_camera_ids += (masked_camera_ids.size() == 1 ? "" : ", ");
    masked_camera_ids += std::to_string(camera_id);
  }
  masked_camera_ids += "]";
  if (ego_mask_params.enabled || masked_camera_ids != "[]") {
    RCLCPP_INFO(
      rclcpp::get_logger(logger_name_.c_str()),
      "mask enabled for StreamPETR preprocess (masked camera ids: %s)", masked_camera_ids.c_str());
  }

  // Data store
  data_store_ = std::make_unique<CameraDataStore>(
    this, rois_number_, roi_height, roi_width, anchor_camera_id_,
    declare_parameter<bool>("is_distorted_image"), ego_mask_params);

  stop_watch_.tic("latency/cycle_time_ms");
  if (debug_mode_) {
    debug_publisher_ptr_ = std::make_unique<autoware_utils::DebugPublisher>(this, this->get_name());
  }

  max_allowed_processing_time_ms_ =
    declare_parameter<double>("diagnostics.max_allowed_processing_time_ms");
  max_acceptable_consecutive_delay_ms_ =
    declare_parameter<double>("diagnostics.max_acceptable_consecutive_delay_ms");
  max_image_age_ms_ = declare_parameter<double>("diagnostics.max_image_age_ms");
  const double validation_callback_interval_ms =
    declare_parameter<double>("diagnostics.validation_callback_interval_ms");

  diagnostic_updater_.setHardwareID(this->get_name());
  diagnostic_updater_.add("camera_status", this, &StreamPetrNode::diagnose_camera_status);
  diagnostic_updater_.add(
    "processing_time_status", this, &StreamPetrNode::diagnose_processing_time);
  diagnostic_updater_.setPeriod(validation_callback_interval_ms / 1000.0);
}

void StreamPetrNode::camera_info_callback(
  CameraInfo::ConstSharedPtr input_camera_info_msg, const int camera_id)
{
  data_store_->update_camera_info(camera_id, input_camera_info_msg);
}

void StreamPetrNode::camera_image_callback(
  Image::ConstSharedPtr input_camera_image_msg, const int camera_id)
{
  if (anchor_camera_id_ == camera_id) {
    stop_watch_.tic("latency/total");
  }

  const auto objects_sub_count =
    pub_objects_->get_subscription_count() + pub_objects_->get_intra_process_subscription_count();
  if (objects_sub_count < 1) {
    return;  // No subscribers, skip processing
  }
  if (!data_store_->check_if_all_camera_info_received()) {
    return;
  }

  data_store_->update_camera_image(camera_id, input_camera_image_msg);

  if (camera_id == anchor_camera_id_) {
    step(input_camera_image_msg->header.stamp);
  }
}

void StreamPetrNode::step(const rclcpp::Time & stamp)
{
  if (!validate_camera_sync()) {
    return;
  }

  if (!prepare_inference_data(stamp)) {
    return;
  }

  auto inference_result = perform_inference();
  if (!inference_result.has_value()) {
    return;
  }

  const auto & result = inference_result.value();
  publish_detection_results(stamp, result.objects);
  publish_debug_metrics(
    result.subnetwork_timings, result.inference_time_ms, result.postprocess_time_ms);
}

bool StreamPetrNode::validate_camera_sync()
{
  const float tdiff = data_store_->check_if_all_images_synced();
  const float prediction_timestamp = data_store_->get_timestamp();

  if (tdiff < 0) {
    RCLCPP_WARN(rclcpp::get_logger(logger_name_.c_str()), "Not all camera info or image received");
    return false;
  }

  if (tdiff > max_camera_time_diff_ || prediction_timestamp < 0.0) {
    RCLCPP_WARN(
      rclcpp::get_logger(logger_name_.c_str()),
      "Couldn't sync cameras. Sync difference: %.2f seconds, time elapsed from start: %.2f seconds",
      tdiff, prediction_timestamp);
    reset_system_state();
    return false;
  }

  return true;
}

void StreamPetrNode::reset_system_state()
{
  network_->wipe_memory();
  initial_transform_set_ = false;
  data_store_->restart();
}

bool StreamPetrNode::prepare_inference_data(const rclcpp::Time & stamp)
{
  if (multithreading_) {
    data_store_->freeze_updates();
  }

  const auto ego_pose_result = get_ego_pose_vector(stamp);
  if (!ego_pose_result.has_value()) {
    cleanup_on_failure();
    return false;
  }

  const auto extrinsic_vectors = get_camera_extrinsics_vector();
  if (!extrinsic_vectors.has_value()) {
    cleanup_on_failure();
    return false;
  }

  // Store the results for inference
  current_ego_pose_ = ego_pose_result.value();
  current_extrinsics_ = extrinsic_vectors.value();
  current_prediction_timestamp_ = data_store_->get_timestamp();

  return true;
}

void StreamPetrNode::cleanup_on_failure()
{
  if (multithreading_) {
    data_store_->unfreeze_updates();
  }
}

std::optional<StreamPetrNode::InferenceResult> StreamPetrNode::perform_inference()
{
  InferenceResult result;

  stop_watch_.tic("latency/inference");
  InferenceInputs inputs = create_inference_inputs();
  network_->inference_detector(inputs, result.subnetwork_timings);
  result.inference_time_ms = stop_watch_.toc("latency/inference", true);

  // The decode only reads the head's output bindings, so the cameras can resume before it.
  if (multithreading_) {
    data_store_->unfreeze_updates();
  }

  stop_watch_.tic("latency/postprocess");
  network_->postprocess(result.objects);
  result.postprocess_time_ms = stop_watch_.toc("latency/postprocess", true);

  return result;
}

InferenceInputs StreamPetrNode::create_inference_inputs()
{
  InferenceInputs inputs;
  inputs.imgs = data_store_->get_image_input();
  inputs.ego_pose = current_ego_pose_.first;
  inputs.ego_pose_inv = current_ego_pose_.second;
  inputs.img_metas_pad = data_store_->get_image_shape();
  inputs.intrinsics = data_store_->get_camera_info_vector();
  inputs.img2lidar = current_extrinsics_;
  inputs.stamp = current_prediction_timestamp_;
  return inputs;
}

void StreamPetrNode::publish_detection_results(
  const rclcpp::Time & stamp,
  const std::vector<autoware_perception_msgs::msg::DetectedObject> & output_objects)
{
  DetectedObjects output_msg;
  output_msg.objects = output_objects;
  output_msg.header.frame_id = "base_link";
  output_msg.header.stamp = stamp;
  pub_objects_->publish(output_msg);
  last_output_frame_stamp_ = stamp;
  last_publish_stamp_ = this->get_clock()->now();
}

void StreamPetrNode::publish_debug_metrics(
  const SubNetworkTimings & subnetwork_timings, double inference_time_ms,
  double postprocess_time_ms)
{
  // Latched before the debug_mode_ gate below: the watchdog is active even without debug topics.
  const double processing_time_ms = stop_watch_.toc("latency/total", true);
  last_processing_time_ms_ = processing_time_ms;
  last_preprocess_time_ms_ = data_store_->get_preprocess_time_ms();
  last_inference_time_ms_ = inference_time_ms;
  last_postprocess_time_ms_ = postprocess_time_ms;

  if (!debug_publisher_ptr_) {
    return;
  }

  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/total", processing_time_ms);
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/preprocess", data_store_->get_preprocess_time_ms());
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/inference", inference_time_ms);
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/inference/backbone", subnetwork_timings.backbone_ms);
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/inference/ptshead", subnetwork_timings.ptshead_ms);
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/inference/pos_embed", subnetwork_timings.pos_embed_ms);
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/postprocess", postprocess_time_ms);
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/cycle_time_ms", stop_watch_.toc("latency/cycle_time_ms", true));
  stop_watch_.tic("latency/cycle_time_ms");
}

void StreamPetrNode::diagnose_camera_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const rclcpp::Time now = this->get_clock()->now();
  const auto camera_status = data_store_->get_camera_status();

  std::size_t num_waiting = 0;
  std::size_t num_stale = 0;
  std::size_t num_rejected = 0;
  int oldest_image_camera_id = -1;
  double oldest_image_age_ms = 0.0;

  for (std::size_t camera_id = 0; camera_id < camera_status.size(); ++camera_id) {
    const auto & status = camera_status[camera_id];
    const std::string prefix = "camera" + std::to_string(camera_id) + "/";

    double age_ms = 0.0;
    if (status.image_received) {
      // Clamp at zero: header stamps run ahead of the node clock right after a rosbag loop.
      age_ms = std::max(0.0, (now.seconds() - status.last_image_timestamp) * 1000.0);
      if (age_ms > oldest_image_age_ms) {
        oldest_image_age_ms = age_ms;
        oldest_image_camera_id = static_cast<int>(camera_id);
      }
    }

    std::string state;
    if (status.input_rejected) {
      state = CAMERA_STATE_REJECTED;
      ++num_rejected;
    } else if (!status.camera_info_received) {
      state = CAMERA_STATE_WAITING_CAMERA_INFO;
      ++num_waiting;
    } else if (!status.image_received) {
      state = CAMERA_STATE_WAITING_IMAGE;
      ++num_waiting;
    } else if (age_ms > max_image_age_ms_) {
      state = CAMERA_STATE_STALE;
      ++num_stale;
    } else {
      state = CAMERA_STATE_ACTIVE;
    }

    stat.add(prefix + "topic", camera_image_topics_[camera_id]);
    stat.add(prefix + "state", state);
    stat.add(prefix + "image_age_ms", status.image_received ? format_milliseconds(age_ms) : "n/a");
  }

  stat.add("num_cameras", static_cast<int>(camera_status.size()));
  stat.add("num_waiting", static_cast<int>(num_waiting));
  stat.add("num_stale", static_cast<int>(num_stale));
  stat.add("num_rejected", static_cast<int>(num_rejected));
  stat.add("oldest_image_camera_id", oldest_image_camera_id);
  stat.add(
    "oldest_image_age_ms",
    oldest_image_camera_id >= 0 ? format_milliseconds(oldest_image_age_ms) : "n/a");

  const float inter_camera_diff_s = data_store_->check_if_all_images_synced();
  stat.add(
    "max_inter_camera_time_diff_ms",
    inter_camera_diff_s >= 0.0f ? format_milliseconds(inter_camera_diff_s * 1000.0) : "n/a");

  auto level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::stringstream message;

  if (num_rejected > 0) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    message << num_rejected
            << " camera(s) publish an encoding or buffer geometry the preprocessing cannot "
               "consume.";
  }
  if (num_stale > 0) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    message << (message.tellp() > 0 ? " " : "") << num_stale << " camera(s) stale; camera"
            << oldest_image_camera_id << " stopped publishing "
            << format_milliseconds(oldest_image_age_ms) << " ms ago.";
  }
  if (level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
    if (num_waiting > 0) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message << num_waiting << " camera(s) waiting for camera_info or a first image.";
    } else if (inter_camera_diff_s > max_camera_time_diff_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      message << "Inter-camera timestamp spread "
              << format_milliseconds(inter_camera_diff_s * 1000.0)
              << " ms exceeds max_camera_time_diff; prediction cycles are being skipped.";
    }
  }

  const std::string summary = message.str();
  stat.summary(level, summary.empty() ? "OK" : summary);
}

void StreamPetrNode::add_no_inference_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat, std::stringstream & message)
{
  stat.add("is_processing_time_ms_in_expected_range", true);
  stat.add("processing_time_ms", "n/a");
  stat.add("preprocess_time_ms", "n/a");
  stat.add("inference_time_ms", "n/a");
  stat.add("postprocess_time_ms", "n/a");
  stat.add("is_consecutive_processing_delay_in_range", true);
  stat.add("consecutive_processing_delay_ms", "n/a");
  message << "Waiting for the node to perform inference.";
}

diagnostic_msgs::msg::DiagnosticStatus::_level_type StreamPetrNode::check_processing_time_status(
  diagnostic_updater::DiagnosticStatusWrapper & stat, std::stringstream & message,
  const rclcpp::Time & timestamp_now)
{
  const double processing_time_ms = last_processing_time_ms_.value();

  if (processing_time_ms > max_allowed_processing_time_ms_) {
    stat.add("is_processing_time_ms_in_expected_range", false);

    message << "Processing time exceeds the acceptable limit of " << max_allowed_processing_time_ms_
            << " ms by " << (processing_time_ms - max_allowed_processing_time_ms_) << " ms.";

    if (!last_in_time_processing_timestamp_) {
      last_in_time_processing_timestamp_ = timestamp_now;
    }

    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  stat.add("is_processing_time_ms_in_expected_range", true);
  last_in_time_processing_timestamp_ = timestamp_now;
  return diagnostic_msgs::msg::DiagnosticStatus::OK;
}

diagnostic_msgs::msg::DiagnosticStatus::_level_type StreamPetrNode::check_consecutive_delays(
  diagnostic_updater::DiagnosticStatusWrapper & stat, std::stringstream & message,
  const rclcpp::Time & timestamp_now,
  diagnostic_msgs::msg::DiagnosticStatus::_level_type current_level)
{
  // check_processing_time_status() ran first and set the timestamp on both of its paths.
  const double delayed_state_duration_ms =
    std::chrono::duration<double, std::milli>(
      std::chrono::nanoseconds(
        (timestamp_now - last_in_time_processing_timestamp_.value()).nanoseconds()))
      .count();

  if (delayed_state_duration_ms > max_acceptable_consecutive_delay_ms_) {
    stat.add("is_consecutive_processing_delay_in_range", false);
    stat.add("consecutive_processing_delay_ms", format_milliseconds(delayed_state_duration_ms));
    message << " Processing delay has consecutively exceeded the acceptable limit continuously.";
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }

  stat.add("is_consecutive_processing_delay_in_range", true);
  stat.add("consecutive_processing_delay_ms", format_milliseconds(delayed_state_duration_ms));
  return current_level;
}

void StreamPetrNode::diagnose_processing_time(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const rclcpp::Time timestamp_now = this->get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus::_level_type diag_level =
    diagnostic_msgs::msg::DiagnosticStatus::OK;
  // Left empty rather than seeded with "OK": the first << would overwrite a seeded string.
  std::stringstream message;

  if (!last_processing_time_ms_) {
    add_no_inference_diagnostics(stat, message);
  } else {
    diag_level = check_processing_time_status(stat, message, timestamp_now);
    stat.add("processing_time_ms", format_milliseconds(last_processing_time_ms_.value()));
    stat.add("preprocess_time_ms", format_milliseconds(last_preprocess_time_ms_));
    stat.add("inference_time_ms", format_milliseconds(last_inference_time_ms_));
    stat.add("postprocess_time_ms", format_milliseconds(last_postprocess_time_ms_));
    diag_level = check_consecutive_delays(stat, message, timestamp_now, diag_level);
  }
  // Clamp at zero: a rosbag loop rewinds the clock behind the latched stamps.
  if (last_output_frame_stamp_ && last_publish_stamp_) {
    const double output_latency_ms = std::max(
      0.0, (last_publish_stamp_->seconds() - last_output_frame_stamp_->seconds()) * 1000.0);
    const double time_since_last_publish_ms =
      std::max(0.0, (timestamp_now.seconds() - last_publish_stamp_->seconds()) * 1000.0);
    stat.add("last_frame_timestamp", format_timestamp(last_output_frame_stamp_->seconds()));
    stat.add("last_published_timestamp", format_timestamp(last_publish_stamp_->seconds()));
    stat.add("output_latency_ms", format_milliseconds(output_latency_ms));
    stat.add("time_since_last_publish_ms", format_milliseconds(time_since_last_publish_ms));
  } else {
    stat.add("last_frame_timestamp", "never");
    stat.add("last_published_timestamp", "never");
    stat.add("output_latency_ms", "n/a");
    stat.add("time_since_last_publish_ms", "n/a");
  }

  const std::string summary = message.str();
  stat.summary(diag_level, summary.empty() ? "OK" : summary);
}

std::optional<std::vector<float>> StreamPetrNode::get_camera_extrinsics_vector()
{
  constexpr size_t num_row = 4;
  constexpr size_t num_col = 4;

  std::vector<std::string> camera_links = data_store_->get_camera_link_names();
  std::vector<float> intrinsics_all = data_store_->get_camera_info_vector();

  std::vector<float> res;
  res.reserve(camera_links.size() * num_row * num_col);

  for (size_t i = 0; i < camera_links.size(); ++i) {
    auto camera_transform_result = compute_camera_transform(i, camera_links, intrinsics_all);
    if (!camera_transform_result.has_value()) {
      return std::nullopt;
    }

    const auto transform_matrix = camera_transform_result.value();
    append_transform_to_result(transform_matrix, res);
  }

  return res;
}

std::optional<Eigen::Matrix4f> StreamPetrNode::compute_camera_transform(
  size_t camera_index, const std::vector<std::string> & camera_links,
  const std::vector<float> & intrinsics_all)
{
  const Eigen::Matrix4f K_4x4 = extract_intrinsic_matrix(camera_index, intrinsics_all);

  auto transform_stamped_result = get_camera_transform(camera_links[camera_index]);
  if (!transform_stamped_result.has_value()) {
    return std::nullopt;
  }

  const Eigen::Matrix4f T_lidar2cam =
    create_lidar_to_camera_transform(transform_stamped_result.value());
  const Eigen::Matrix4f T_lidar2img = K_4x4 * T_lidar2cam;
  return T_lidar2img.inverse();
}

Eigen::Matrix4f StreamPetrNode::extract_intrinsic_matrix(
  size_t camera_index, const std::vector<float> & intrinsics_all)
{
  constexpr size_t num_row = 4;
  constexpr size_t num_col = 4;

  Eigen::Matrix4f K_4x4 = Eigen::Matrix4f::Identity();
  size_t offset = camera_index * num_row * num_col;

  for (size_t row = 0; row < num_row; ++row) {
    for (size_t col = 0; col < num_col; ++col) {
      K_4x4(row, col) = intrinsics_all[offset + row * num_col + col];
    }
  }

  return K_4x4;
}

std::optional<geometry_msgs::msg::TransformStamped> StreamPetrNode::get_camera_transform(
  const std::string & camera_link)
{
  try {
    return tf_buffer_.lookupTransform(camera_link, "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger(logger_name_.c_str()), "Could not transform from base_link to %s: %s",
      camera_link.c_str(), ex.what());
    return std::nullopt;
  }
}

Eigen::Matrix4f StreamPetrNode::create_lidar_to_camera_transform(
  const geometry_msgs::msg::TransformStamped & transform_stamped)
{
  Eigen::Matrix4f T_lidar2cam = Eigen::Matrix4f::Identity();

  const auto rotation_matrix = extract_rotation_matrix(transform_stamped);
  const auto translation_vector = extract_translation_vector(transform_stamped);

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      T_lidar2cam(r, c) = rotation_matrix(r, c);
    }
    T_lidar2cam(r, 3) = translation_vector(r);
  }

  return T_lidar2cam;
}

Eigen::Matrix3f StreamPetrNode::extract_rotation_matrix(
  const geometry_msgs::msg::TransformStamped & transform_stamped)
{
  tf2::Quaternion tf2_q(
    transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y,
    transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w);
  tf2::Matrix3x3 tf2_R(tf2_q);

  Eigen::Matrix3f R;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R(r, c) = static_cast<float>(tf2_R[r][c]);
    }
  }

  return R;
}

Eigen::Vector3f StreamPetrNode::extract_translation_vector(
  const geometry_msgs::msg::TransformStamped & transform_stamped)
{
  Eigen::Vector3f t;
  t << static_cast<float>(transform_stamped.transform.translation.x),
    static_cast<float>(transform_stamped.transform.translation.y),
    static_cast<float>(transform_stamped.transform.translation.z);
  return t;
}

void StreamPetrNode::append_transform_to_result(
  const Eigen::Matrix4f & transform_matrix, std::vector<float> & result)
{
  constexpr size_t num_row = 4;
  constexpr size_t num_col = 4;

  for (size_t row = 0; row < num_row; ++row) {
    for (size_t col = 0; col < num_col; ++col) {
      result.push_back(transform_matrix(row, col));
    }
  }
}

std::optional<std::pair<std::vector<float>, std::vector<float>>>
StreamPetrNode::get_ego_pose_vector(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped current_transform;
  try {
    // Get the current transform from map to base_link at the specific timestamp
    current_transform = tf_buffer_.lookupTransform("map", "base_link", stamp);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger(logger_name_.c_str()),
      "Could not get transform from map to base_link at timestamp: %s", ex.what());
    return std::nullopt;
  }

  // Set initial transform if not set yet
  if (!initial_transform_set_) {
    initial_transform_ = current_transform;
    initial_transform_set_ = true;
  }

  // Get initial position
  tf2::Vector3 initial_translation(
    initial_transform_.transform.translation.x, initial_transform_.transform.translation.y,
    initial_transform_.transform.translation.z);

  // Get current transform
  tf2::Quaternion current_quat(
    current_transform.transform.rotation.x, current_transform.transform.rotation.y,
    current_transform.transform.rotation.z, current_transform.transform.rotation.w);
  tf2::Vector3 current_translation(
    current_transform.transform.translation.x, current_transform.transform.translation.y,
    current_transform.transform.translation.z);

  // Calculate relative position (current - initial)
  tf2::Vector3 relative_translation = current_translation - initial_translation;

  // Use absolute rotation (not relative to initial rotation)
  tf2::Matrix3x3 relative_rot(current_quat);

  std::vector<float> egopose = {
    static_cast<float>(relative_rot[0][0]),
    static_cast<float>(relative_rot[0][1]),
    static_cast<float>(relative_rot[0][2]),
    static_cast<float>(relative_translation.x()),
    static_cast<float>(relative_rot[1][0]),
    static_cast<float>(relative_rot[1][1]),
    static_cast<float>(relative_rot[1][2]),
    static_cast<float>(relative_translation.y()),
    static_cast<float>(relative_rot[2][0]),
    static_cast<float>(relative_rot[2][1]),
    static_cast<float>(relative_rot[2][2]),
    static_cast<float>(relative_translation.z()),
    0.0f,
    0.0f,
    0.0f,
    1.0f};

  // Compute inverse transform
  tf2::Matrix3x3 inverse_rot = relative_rot.transpose();
  tf2::Vector3 inverse_translation = -(inverse_rot * relative_translation);

  std::vector<float> inverse_egopose = {
    static_cast<float>(inverse_rot[0][0]),
    static_cast<float>(inverse_rot[0][1]),
    static_cast<float>(inverse_rot[0][2]),
    static_cast<float>(inverse_translation.x()),
    static_cast<float>(inverse_rot[1][0]),
    static_cast<float>(inverse_rot[1][1]),
    static_cast<float>(inverse_rot[1][2]),
    static_cast<float>(inverse_translation.y()),
    static_cast<float>(inverse_rot[2][0]),
    static_cast<float>(inverse_rot[2][1]),
    static_cast<float>(inverse_rot[2][2]),
    static_cast<float>(inverse_translation.z()),
    0.0f,
    0.0f,
    0.0f,
    1.0f};
  return std::make_pair(egopose, inverse_egopose);
}

}  // namespace autoware::camera_streampetr

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::camera_streampetr::StreamPetrNode)
