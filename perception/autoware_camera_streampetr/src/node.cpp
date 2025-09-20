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

#include "autoware/camera_streampetr/postprocess/non_maximum_suppression.hpp"

#include <Eigen/Dense>
#include <image_transport/image_transport.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace autoware::camera_streampetr
{

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
  logger_name_(declare_parameter<std::string>("logger_name", "camera_streampetr")),
  multithreading_(declare_parameter<bool>("multithreading", false)),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  rois_number_(static_cast<size_t>(declare_parameter<int>("model_params.rois_number", 6))),
  max_camera_time_diff_(declare_parameter<float>("max_camera_time_diff", 0.15f)),
  anchor_camera_id_(declare_parameter<int>("anchor_camera_id", 0)),
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
    declare_parameter<std::string>("model_params.backbone_engine_path", "");
  const std::string head_engine_path =
    declare_parameter<std::string>("model_params.head_engine_path", "");
  const std::string position_embedding_engine_path =
    declare_parameter<std::string>("model_params.position_embedding_engine_path", "");

  const std::string trt_precision = declare_parameter<std::string>("model_params.trt_precision");
  const uint64_t workspace_size =
    1ULL << declare_parameter<int>("model_params.workspace_size", 32);  // Default 4GB

  const bool use_temporal = declare_parameter<bool>("model_params.use_temporal");
  const double search_distance_2d =
    declare_parameter<double>("post_process_params.iou_nms_search_distance_2d");
  const double circle_nms_dist_threshold =
    declare_parameter<double>("post_process_params.circle_nms_dist_threshold");
  const double iou_threshold = declare_parameter<double>("post_process_params.iou_nms_threshold");
  const double confidence_threshold =
    declare_parameter<double>("post_process_params.confidence_threshold");
  const std::vector<std::string> class_names =
    declare_parameter<std::vector<std::string>>("model_params.class_names");
  const int32_t num_proposals = declare_parameter<int32_t>("model_params.num_proposals");
  const std::vector<double> yaw_norm_thresholds =
    declare_parameter<std::vector<double>>("post_process_params.yaw_norm_thresholds");
  const std::vector<float> detection_range =
    cast_to_float(declare_parameter<std::vector<double>>("model_params.detection_range"));
  const int pre_memory_length = declare_parameter<int>("model_params.pre_memory_length", 1024);
  const int post_memory_length = declare_parameter<int>("model_params.post_memory_length", 1280);

  NetworkConfig network_config{
    logger_name_,
    use_temporal,
    search_distance_2d,
    circle_nms_dist_threshold,
    iou_threshold,
    confidence_threshold,
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
    camera_image_subs_.at(roi_i) = image_transport::create_subscription(
      this, "~/input/camera" + std::to_string(roi_i) + "/image",
      std::bind(
        &StreamPetrNode::camera_image_callback, this, std::placeholders::_1,
        static_cast<int>(roi_i)),
      is_compressed_image ? "compressed" : "raw", rmw_qos_profile_sensor_data, sub_options);
  }

  // Publishers
  pub_objects_ = this->create_publisher<DetectedObjects>("~/output/objects", rclcpp::QoS{1});

  // Data store
  data_store_ = std::make_unique<CameraDataStore>(
    this, rois_number_, roi_height, roi_width, anchor_camera_id_,
    declare_parameter<bool>("is_distorted_image"),
    declare_parameter<double>("downsample_factor", 1.0));

  if (debug_mode_) {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("latency/cycle_time_ms");
  }
}

void StreamPetrNode::camera_info_callback(
  CameraInfo::ConstSharedPtr input_camera_info_msg, const int camera_id)
{
  data_store_->update_camera_info(camera_id, input_camera_info_msg);
}

void StreamPetrNode::camera_image_callback(
  Image::ConstSharedPtr input_camera_image_msg, const int camera_id)
{
  if (stop_watch_ptr_ && anchor_camera_id_ == camera_id) {
    stop_watch_ptr_->tic("latency/total");
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

  const auto [output_objects, forward_time_ms, inference_time_ms] = inference_result.value();
  publish_detection_results(stamp, output_objects);
  publish_debug_metrics(forward_time_ms, inference_time_ms);
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

std::optional<std::tuple<
  std::vector<autoware_perception_msgs::msg::DetectedObject>, std::vector<float>, double>>
StreamPetrNode::perform_inference()
{
  if (stop_watch_ptr_) {
    stop_watch_ptr_->tic("latency/inference");
  }

  std::vector<float> forward_time_ms;
  std::vector<autoware_perception_msgs::msg::DetectedObject> output_objects;

  InferenceInputs inputs = create_inference_inputs();
  network_->inference_detector(inputs, output_objects, forward_time_ms);

  if (multithreading_) {
    data_store_->unfreeze_updates();
  }

  double inference_time_ms = -1.0;
  if (stop_watch_ptr_) {
    inference_time_ms = stop_watch_ptr_->toc("latency/inference", true);
  }

  return std::make_tuple(output_objects, forward_time_ms, inference_time_ms);
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
}

void StreamPetrNode::publish_debug_metrics(
  const std::vector<float> & forward_time_ms, double inference_time_ms)
{
  if (!debug_publisher_ptr_ || !stop_watch_ptr_) {
    return;
  }

  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/total", stop_watch_ptr_->toc("latency/total", true));
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/preprocess", data_store_->get_preprocess_time_ms());
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/inference", inference_time_ms);
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/inference/backbone", forward_time_ms[0]);
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/inference/ptshead", forward_time_ms[1]);
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/inference/pos_embed", forward_time_ms[2]);
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/inference/postprocess", forward_time_ms[3]);
  debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "latency/cycle_time_ms", stop_watch_ptr_->toc("latency/cycle_time_ms", true));
  stop_watch_ptr_->tic("latency/cycle_time_ms");
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
