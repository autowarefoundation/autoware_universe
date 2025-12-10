// Copyright 2025 TIER IV.
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

#include "vad_node.hpp"

#include "utils/transform_utils.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::tensorrt_vad
{
template <typename MsgType>
bool VadNode::process_callback(
  const typename MsgType::ConstSharedPtr msg, const std::string & callback_name,
  std::function<void(const typename MsgType::ConstSharedPtr)> setter)
{
  if (!msg) {
    auto clock = this->get_clock();
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock, 5000, "Received null %s message", callback_name.c_str());
    return false;
  }

  try {
    std::lock_guard<std::mutex> lock(data_mutex_);
    setter(msg);
  } catch (const std::exception & e) {
    auto clock = this->get_clock();
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *clock, 5000, "Exception in %s: %s", callback_name.c_str(), e.what());
    return false;
  }

  auto clock = this->get_clock();
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *clock, 5000, "Received %s data", callback_name.c_str());
  return true;
}

bool VadNode::validate_camera_id(std::size_t camera_id, const std::string & context)
{
  if (static_cast<int32_t>(camera_id) < num_cameras_) {
    return true;
  }

  auto clock = this->get_clock();
  RCLCPP_ERROR_THROTTLE(
    this->get_logger(), *clock, 5000, "Invalid camera_id: %zu in %s. Expected range 0-%d",
    camera_id, context.c_str(), num_cameras_ - 1);
  return false;
}

VadNode::VadNode(const rclcpp::NodeOptions & options)
: Node("vad_node", options),
  tf_buffer_(this->get_clock()),
  num_cameras_(declare_parameter<int32_t>("node_params.num_cameras")),
  front_camera_id_(declare_parameter<int32_t>("sync_params.front_camera_id")),
  trajectory_timestep_(declare_parameter<double>("interface_params.trajectory_timestep")),
  vad_input_topic_data_current_frame_(num_cameras_)
{
  // Load model parameters from JSON first
  std::string model_param_path = this->declare_parameter<std::string>("model_param_path");
  RCLCPP_INFO(this->get_logger(), "Loading model parameters from: %s", model_param_path.c_str());
  utils::check_model_version(model_param_path);
  auto model_params = utils::load_model_params(model_param_path);
  RCLCPP_INFO(
    this->get_logger(), "Loaded model: %s (v%d.%d)", model_params.model_name.c_str(),
    model_params.major_version, model_params.minor_version);

  // Create VadInterfaceConfig using model parameters from JSON
  vad_interface_config_ = std::make_unique<VadInterfaceConfig>(
    model_params.target_image_width, model_params.target_image_height,
    declare_parameter<std::vector<double>>("interface_params.detection_range"),
    declare_parameter<int32_t>("model_params.default_command"),
    model_params.map_classes,  // From JSON
    declare_parameter<std::vector<double>>("interface_params.map_colors"),
    declare_parameter<std::vector<std::string>>("class_mapping"),
    model_params.object_classes  // From JSON
  );

  // Declare additional parameters that will be used in load_vad_config
  declare_parameter<std::vector<double>>("model_params.map_confidence_thresholds");
  declare_parameter<std::vector<double>>("model_params.object_confidence_thresholds");

  // Publishers
  trajectory_publisher_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/output/trajectory", rclcpp::QoS(1));

  candidate_trajectories_publisher_ =
    this->create_publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>(
      "~/output/trajectories", rclcpp::QoS(1));

  predicted_objects_publisher_ =
    this->create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
      "~/output/objects", rclcpp::QoS(1));

  map_points_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("~/output/map", rclcpp::QoS(1));

  // Create QoS profiles for sensor data (best effort for compatibility with typical sensor topics)
  auto sensor_qos = rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::BestEffort);
  auto camera_info_qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort);
  auto reliable_qos = rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::Reliable);

  // Subscribers for each camera
  create_camera_image_subscribers(sensor_qos);

  // Subscribers for camera info
  create_camera_info_subscribers(camera_info_qos);

  // Odometry subscriber (kinematic state is usually reliable)
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "~/input/kinematic_state", reliable_qos,
    std::bind(&VadNode::odometry_callback, this, std::placeholders::_1));

  // Acceleration subscriber (sensor data typically uses best effort)
  acceleration_sub_ = this->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "~/input/acceleration", sensor_qos,
    std::bind(&VadNode::acceleration_callback, this, std::placeholders::_1));

  // TF static subscriber (transient local for persistence)
  auto tf_static_qos = rclcpp::QoS(1)
                         .reliability(rclcpp::ReliabilityPolicy::Reliable)
                         .durability(rclcpp::DurabilityPolicy::TransientLocal);
  tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_static", tf_static_qos,
    std::bind(&VadNode::tf_static_callback, this, std::placeholders::_1));

  // Initialize synchronization strategy
  double sync_tolerance_ms = declare_parameter<double>("sync_params.sync_tolerance_ms");
  sync_strategy_ =
    std::make_unique<FrontCriticalSynchronizationStrategy>(front_camera_id_, sync_tolerance_ms);

  // Initialize VAD model on first complete frame
  initialize_vad_model();

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "VAD Node has been initialized - VAD model will be initialized after first callback");
}

void VadNode::image_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg, std::size_t camera_id)
{
  if (!validate_camera_id(camera_id, "image_callback")) {
    return;
  }

  const std::string callback_name = "image (camera " + std::to_string(camera_id) + ")";
  const bool processed = process_callback<sensor_msgs::msg::Image>(
    msg, callback_name,
    [this, camera_id](const sensor_msgs::msg::Image::ConstSharedPtr & image_msg) {
      vad_input_topic_data_current_frame_.set_image(camera_id, image_msg);
    });

  if (processed && static_cast<int32_t>(camera_id) == front_camera_id_) {
    anchor_callback();
  }
}

void VadNode::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg, std::size_t camera_id)
{
  if (!validate_camera_id(camera_id, "camera_info_callback")) {
    return;
  }

  const std::string callback_name = "camera_info (camera " + std::to_string(camera_id) + ")";
  process_callback<sensor_msgs::msg::CameraInfo>(
    msg, callback_name,
    [this, camera_id](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg) {
      vad_input_topic_data_current_frame_.set_camera_info(camera_id, info_msg);
    });
}

void VadNode::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  process_callback<nav_msgs::msg::Odometry>(
    msg, "odometry", [this](const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg) {
      vad_input_topic_data_current_frame_.set_kinematic_state(odometry_msg);
    });
}

void VadNode::acceleration_callback(
  const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr msg)
{
  process_callback<geometry_msgs::msg::AccelWithCovarianceStamped>(
    msg, "acceleration",
    [this](const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr & accel_msg) {
      vad_input_topic_data_current_frame_.set_acceleration(accel_msg);
    });
}

void VadNode::tf_static_callback(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg)
{
  // Register transforms in tf_buffer
  for (const auto & transform : msg->transforms) {
    tf_buffer_.setTransform(transform, "default_authority", true);
  }

  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received TF static data");
}

void VadNode::anchor_callback()
{
  try {
    std::optional<VadInputTopicData> frame_data;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!sync_strategy_) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000, "Sync strategy not initialized");
        return;
      }

      if (!sync_strategy_->is_ready(vad_input_topic_data_current_frame_)) {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Synchronization strategy indicates data is not ready for inference");
        return;
      }

      frame_data.emplace(vad_input_topic_data_current_frame_);
      vad_input_topic_data_current_frame_.reset();
    }

    if (!frame_data.has_value()) {
      return;
    }

    auto vad_output_topic_data = trigger_inference(std::move(*frame_data));
    if (vad_output_topic_data.has_value()) {
      publish(vad_output_topic_data.value());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Exception in anchor_callback: %s", e.what());
  }
}

std::optional<VadOutputTopicData> VadNode::trigger_inference(
  VadInputTopicData vad_input_topic_data_current_frame)
{
  if (sync_strategy_->is_dropped(vad_input_topic_data_current_frame)) {
    auto filled_data_opt = sync_strategy_->fill_dropped_data(vad_input_topic_data_current_frame);
    if (!filled_data_opt) {
      // Cannot fill dropped data (front camera unavailable)
      return std::nullopt;
    }
    vad_input_topic_data_current_frame = std::move(filled_data_opt.value());
  }

  if (vad_input_topic_data_current_frame.is_complete()) {
    // Execute inference
    auto vad_output_topic_data = execute_inference(vad_input_topic_data_current_frame);
    return vad_output_topic_data;
  } else {
    return std::nullopt;
  }
}

void VadNode::initialize_vad_model()
{
  // load configs
  VadConfig vad_config = load_vad_config();
  auto [backbone_trt_config, head_trt_config, head_no_prev_trt_config] = load_trt_common_configs();

  // Initialize VAD interface and model
  auto tf_buffer_shared = std::shared_ptr<tf2_ros::Buffer>(&tf_buffer_, [](tf2_ros::Buffer *) {});
  vad_interface_ptr_ = std::make_unique<VadInterface>(*vad_interface_config_, tf_buffer_shared);

  // Create RosVadLogger using the logger
  auto ros_logger = std::make_shared<RosVadLogger>(this->get_logger());
  vad_model_ptr_ = std::make_unique<VadModel<RosVadLogger>>(
    vad_config, backbone_trt_config, head_trt_config, head_no_prev_trt_config, ros_logger);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "VAD model and interface initialized successfully");
}

VadConfig VadNode::load_vad_config()
{
  VadConfig vad_config;

  // Load model parameters from JSON
  std::string model_param_path = this->get_parameter("model_param_path").as_string();
  RCLCPP_INFO(this->get_logger(), "Loading model parameters from: %s", model_param_path.c_str());
  utils::check_model_version(model_param_path);
  auto model_params = utils::load_model_params(model_param_path);

  RCLCPP_INFO(
    this->get_logger(), "Loaded model: %s (v%d.%d)", model_params.model_name.c_str(),
    model_params.major_version, model_params.minor_version);

  // Deployment-specific parameters (from YAML)
  vad_config.num_cameras = this->get_parameter("node_params.num_cameras").as_int();

  // Model-specific parameters (from param.json)
  vad_config.bev_h = model_params.bev_height;
  vad_config.bev_w = model_params.bev_width;
  vad_config.bev_feature_dim = model_params.bev_feature_dim;
  vad_config.downsample_factor = model_params.downsample_factor;
  vad_config.num_decoder_layers = model_params.num_decoder_layers;
  vad_config.prediction_num_queries = model_params.prediction_num_queries;
  vad_config.prediction_num_classes = model_params.prediction_num_classes;
  vad_config.prediction_bbox_pred_dim = model_params.prediction_bbox_pred_dim;
  vad_config.prediction_trajectory_modes = model_params.prediction_trajectory_modes;
  vad_config.prediction_timesteps = model_params.prediction_timesteps;
  vad_config.planning_ego_commands = model_params.planning_ego_commands;
  vad_config.planning_timesteps = model_params.planning_timesteps;
  vad_config.map_num_queries = model_params.map_num_queries;
  vad_config.map_num_class = model_params.map_num_classes;
  vad_config.map_points_per_polylines = model_params.map_points_per_polyline;
  vad_config.can_bus_dim = model_params.can_bus_dim;
  vad_config.target_image_width = model_params.target_image_width;
  vad_config.target_image_height = model_params.target_image_height;

  // Image normalization from param.json
  vad_config.image_normalization_param_mean = model_params.image_normalization_mean;
  vad_config.image_normalization_param_std = model_params.image_normalization_std;

  // Deployment-specific parameters (from YAML)
  load_detection_range(vad_config);

  // Map configuration: use class names from param.json but thresholds from YAML
  load_map_configuration_with_model_params(vad_config, model_params);

  // Object configuration: use class names from param.json but thresholds from YAML
  load_object_configuration_with_model_params(vad_config, model_params);

  vad_config.plugins_path = this->declare_parameter<std::string>("model_params.plugins_path");
  vad_config.input_image_width =
    this->declare_parameter<int32_t>("interface_params.input_image_width");
  vad_config.input_image_height =
    this->declare_parameter<int32_t>("interface_params.input_image_height");

  load_network_configurations(vad_config);

  return vad_config;
}

void VadNode::load_detection_range(VadConfig & config)
{
  const auto detection_range =
    this->get_parameter("interface_params.detection_range").as_double_array();
  const std::size_t entries =
    std::min<std::size_t>(config.detection_range.size(), detection_range.size());
  for (std::size_t i = 0; i < entries; ++i) {
    config.detection_range[i] = static_cast<float>(detection_range[i]);
  }
}

void VadNode::load_classification_config(const ClassificationConfig & params)
{
  if (params.class_names.size() != params.thresholds.size()) {
    RCLCPP_ERROR(
      this->get_logger(), "%s: class_names (%zu) and thresholds (%zu) size mismatch",
      params.validation_context.c_str(), params.class_names.size(), params.thresholds.size());
    throw std::runtime_error(params.validation_context + ": Parameter array length mismatch");
  }

  params.target_class_names->assign(params.class_names.begin(), params.class_names.end());
  if (params.num_classes != nullptr) {
    *params.num_classes = static_cast<int32_t>(params.class_names.size());
  }

  params.target_thresholds->clear();
  for (std::size_t i = 0; i < params.class_names.size(); ++i) {
    (*params.target_thresholds)[params.class_names[i]] = static_cast<float>(params.thresholds[i]);
  }
}

void VadNode::load_map_configuration(VadConfig & config)
{
  load_classification_config(
    {this->get_parameter("model_params.map_class_names").as_string_array(),
     this->get_parameter("model_params.map_confidence_thresholds").as_double_array(),
     &config.map_class_names, &config.map_confidence_thresholds, &config.map_num_classes,
     "load_map_configuration"});
}

void VadNode::load_map_configuration_with_model_params(
  VadConfig & config, const utils::ModelParams & model_params)
{
  load_classification_config(
    {model_params.map_classes,
     this->get_parameter("model_params.map_confidence_thresholds").as_double_array(),
     &config.map_class_names, &config.map_confidence_thresholds, &config.map_num_classes,
     "load_map_configuration_with_model_params"});
}

void VadNode::load_object_configuration(VadConfig & config)
{
  load_classification_config(
    {this->get_parameter("model_params.object_class_names").as_string_array(),
     this->get_parameter("model_params.object_confidence_thresholds").as_double_array(),
     &config.bbox_class_names, &config.object_confidence_thresholds, nullptr,
     "load_object_configuration"});
}

void VadNode::load_object_configuration_with_model_params(
  VadConfig & config, const utils::ModelParams & model_params)
{
  load_classification_config(
    {model_params.object_classes,
     this->get_parameter("model_params.object_confidence_thresholds").as_double_array(),
     &config.bbox_class_names, &config.object_confidence_thresholds, nullptr,
     "load_object_configuration_with_model_params"});
}

void VadNode::load_image_normalization(VadConfig & config)
{
  const auto image_mean =
    this->declare_parameter<std::vector<double>>("model_params.image_normalization_param_mean");
  const auto image_std =
    this->declare_parameter<std::vector<double>>("model_params.image_normalization_param_std");

  const std::size_t mean_entries =
    std::min<std::size_t>(config.image_normalization_param_mean.size(), image_mean.size());
  const std::size_t std_entries =
    std::min<std::size_t>(config.image_normalization_param_std.size(), image_std.size());

  for (std::size_t i = 0; i < mean_entries; ++i) {
    config.image_normalization_param_mean[i] = static_cast<float>(image_mean[i]);
  }

  for (std::size_t i = 0; i < std_entries; ++i) {
    config.image_normalization_param_std[i] = static_cast<float>(image_std[i]);
  }
}

void VadNode::load_network_configurations(VadConfig & config)
{
  config.nets_config.clear();

  NetConfig backbone_config;
  backbone_config.name = this->declare_parameter<std::string>("model_params.nets.backbone.name");

  NetConfig head_config;
  head_config.name = this->declare_parameter<std::string>("model_params.nets.head.name");
  const std::string head_input_feature =
    this->declare_parameter<std::string>("model_params.nets.head.inputs.input_feature");
  const std::string head_input_net =
    this->declare_parameter<std::string>("model_params.nets.head.inputs.net");
  const std::string head_input_name =
    this->declare_parameter<std::string>("model_params.nets.head.inputs.name");
  head_config.inputs[head_input_feature]["net"] = head_input_net;
  head_config.inputs[head_input_feature]["name"] = head_input_name;

  NetConfig head_no_prev_config;
  head_no_prev_config.name =
    this->declare_parameter<std::string>("model_params.nets.head_no_prev.name");
  const std::string head_no_prev_input_feature =
    this->declare_parameter<std::string>("model_params.nets.head_no_prev.inputs.input_feature");
  const std::string head_no_prev_input_net =
    this->declare_parameter<std::string>("model_params.nets.head_no_prev.inputs.net");
  const std::string head_no_prev_input_name =
    this->declare_parameter<std::string>("model_params.nets.head_no_prev.inputs.name");
  head_no_prev_config.inputs[head_no_prev_input_feature]["net"] = head_no_prev_input_net;
  head_no_prev_config.inputs[head_no_prev_input_feature]["name"] = head_no_prev_input_name;

  config.nets_config.push_back(backbone_config);
  config.nets_config.push_back(head_config);
  config.nets_config.push_back(head_no_prev_config);
}

std::tuple<
  autoware::tensorrt_common::TrtCommonConfig, autoware::tensorrt_common::TrtCommonConfig,
  autoware::tensorrt_common::TrtCommonConfig>
VadNode::load_trt_common_configs()
{
  std::string backbone_onnx_path =
    this->declare_parameter<std::string>("model_params.nets.backbone.onnx_path");
  std::string backbone_precision =
    this->declare_parameter<std::string>("model_params.nets.backbone.precision");
  std::string backbone_engine_path =
    this->declare_parameter<std::string>("model_params.nets.backbone.engine_path");
  autoware::tensorrt_common::TrtCommonConfig backbone_trt_config(
    backbone_onnx_path, backbone_precision, backbone_engine_path, 5ULL << 30U);

  std::string head_onnx_path =
    this->declare_parameter<std::string>("model_params.nets.head.onnx_path");
  std::string head_precision =
    this->declare_parameter<std::string>("model_params.nets.head.precision");
  std::string head_engine_path =
    this->declare_parameter<std::string>("model_params.nets.head.engine_path");
  autoware::tensorrt_common::TrtCommonConfig head_trt_config(
    head_onnx_path, head_precision, head_engine_path, 5ULL << 30U);

  std::string head_no_prev_onnx_path =
    this->declare_parameter<std::string>("model_params.nets.head_no_prev.onnx_path");
  std::string head_no_prev_precision =
    this->declare_parameter<std::string>("model_params.nets.head_no_prev.precision");
  std::string head_no_prev_engine_path =
    this->declare_parameter<std::string>("model_params.nets.head_no_prev.engine_path");
  autoware::tensorrt_common::TrtCommonConfig head_no_prev_trt_config(
    head_no_prev_onnx_path, head_no_prev_precision, head_no_prev_engine_path, 5ULL << 30U);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "TrtCommon configurations loaded (5GB workspace):");
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000, "  Backbone - ONNX: %s, Precision: %s",
    backbone_onnx_path.c_str(), backbone_precision.c_str());
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000, "  Head - ONNX: %s, Precision: %s",
    head_onnx_path.c_str(), head_precision.c_str());
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000, "  Head No Prev - ONNX: %s, Precision: %s",
    head_no_prev_onnx_path.c_str(), head_no_prev_precision.c_str());

  return {backbone_trt_config, head_trt_config, head_no_prev_trt_config};
}

std::optional<VadOutputTopicData> VadNode::execute_inference(
  const VadInputTopicData & vad_input_topic_data)
{
  if (!vad_interface_ptr_ || !vad_model_ptr_) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "VAD interface or model not initialized");
    return std::nullopt;
  }

  try {
    if (!vad_input_topic_data.is_complete()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Skipping inference: input frame incomplete (images/camera_info/state/accel)");
      return std::nullopt;
    }
    // Convert to VadInputData through VadInterface
    const auto vad_input = vad_interface_ptr_->convert_input(vad_input_topic_data);

    // Execute inference with VadModel
    const auto vad_output = vad_model_ptr_->infer(vad_input);

    const auto [base2map_transform, map2base_transform] =
      utils::get_transform_matrix(*vad_input_topic_data.kinematic_state);
    // Convert to ROS types through VadInterface
    if (vad_output.has_value()) {
      const auto vad_output_topic_data = vad_interface_ptr_->convert_output(
        *vad_output, this->now(), trajectory_timestep_, base2map_transform);
      // Return VadOutputTopicData
      return vad_output_topic_data;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Exception during inference: %s", e.what());
    return std::nullopt;
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Unknown exception during inference");
    return std::nullopt;
  }

  return std::nullopt;
}

void VadNode::publish(const VadOutputTopicData & vad_output_topic_data)
{
  // Publish selected trajectory
  trajectory_publisher_->publish(vad_output_topic_data.trajectory);

  // Publish candidate trajectories
  candidate_trajectories_publisher_->publish(vad_output_topic_data.candidate_trajectories);

  // // Publish predicted objects
  predicted_objects_publisher_->publish(vad_output_topic_data.objects);

  // Publish map points
  map_points_publisher_->publish(vad_output_topic_data.map_points);

  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000, "Published trajectories and predicted objects");
}

void VadNode::create_camera_image_subscribers(const rclcpp::QoS & sensor_qos)
{
  try {
    camera_image_subs_.resize(num_cameras_);
    std::vector<bool> use_raw_cameras =
      this->declare_parameter<std::vector<bool>>("node_params.use_raw");

    // Validate use_raw parameter size matches num_cameras
    if (static_cast<int32_t>(use_raw_cameras.size()) != num_cameras_) {
      RCLCPP_ERROR(
        this->get_logger(), "use_raw parameter size (%zu) does not match num_cameras (%d)",
        use_raw_cameras.size(), num_cameras_);
      throw std::runtime_error(
        "Parameter array length mismatch: use_raw size must match num_cameras");
    }

    auto resolve_topic_name = [this](const std::string & query) {
      return this->get_node_topics_interface()->resolve_topic_name(query);
    };
    for (int32_t i = 0; i < num_cameras_; ++i) {
      const auto transport = use_raw_cameras[i] ? "raw" : "compressed";
      auto callback = [this, i](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        this->image_callback(msg, i);
      };

      const auto image_topic = resolve_topic_name("~/input/image" + std::to_string(i));
      RCLCPP_INFO(
        this->get_logger(), "Creating image subscriber %d for topic: %s, transport: %s", i,
        image_topic.c_str(), transport);
      camera_image_subs_[i] = image_transport::create_subscription(
        this, image_topic, callback, transport, sensor_qos.get_rmw_qos_profile());
      RCLCPP_INFO(this->get_logger(), "Image subscriber %d created successfully", i);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in create_camera_image_subscribers: %s", e.what());
    throw;  // Re-throw to prevent partial initialization
  }
}

void VadNode::create_camera_info_subscribers(const rclcpp::QoS & camera_info_qos)
{
  camera_info_subs_.resize(num_cameras_);
  for (int32_t i = 0; i < num_cameras_; ++i) {
    auto callback = [this, i](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      this->camera_info_callback(msg, i);
    };

    camera_info_subs_[i] = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "~/input/camera_info" + std::to_string(i), camera_info_qos, callback);
  }
}

}  // namespace autoware::tensorrt_vad

// Register the component with the ROS 2 component system
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_vad::VadNode)
