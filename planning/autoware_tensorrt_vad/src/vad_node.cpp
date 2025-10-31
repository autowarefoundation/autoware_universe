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

#include "autoware/tensorrt_vad/vad_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::tensorrt_vad
{
std::pair<Eigen::Matrix4d, Eigen::Matrix4d> get_transform_matrix(
  const nav_msgs::msg::Odometry & msg)
{
  // Extract position
  double x = msg.pose.pose.position.x;
  double y = msg.pose.pose.position.y;
  double z = msg.pose.pose.position.z;

  // Create Eigen quaternion and normalize it
  Eigen::Quaterniond q = std::invoke([&msg]() -> Eigen::Quaterniond {
    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;
    double qw = msg.pose.pose.orientation.w;

    // Create Eigen quaternion and normalize it
    Eigen::Quaterniond q(qw, qx, qy, qz);
    return (q.norm() < std::numeric_limits<double>::epsilon()) ? Eigen::Quaterniond::Identity()
                                                               : q.normalized();
  });

  // Rotation matrix (3x3)
  Eigen::Matrix3d R = q.toRotationMatrix();

  // Translation vector
  Eigen::Vector3d t(x, y, z);

  // Base_link → Map (forward)
  Eigen::Matrix4d bl2map = Eigen::Matrix4d::Identity();
  bl2map.block<3, 3>(0, 0) = R;
  bl2map.block<3, 1>(0, 3) = t;

  // Map → Base_link (inverse)
  Eigen::Matrix4d map2bl = Eigen::Matrix4d::Identity();
  map2bl.block<3, 3>(0, 0) = R.transpose();
  map2bl.block<3, 1>(0, 3) = -R.transpose() * t;

  return {bl2map, map2bl};
}

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
  vad_interface_config_(
    declare_parameter<int32_t>("interface_params.target_image_width"),
    declare_parameter<int32_t>("interface_params.target_image_height"),
    declare_parameter<std::vector<double>>("interface_params.detection_range"),
    declare_parameter<int32_t>("model_params.default_command"),
    declare_parameter<std::vector<std::string>>("model_params.map_class_names"),
    declare_parameter<std::vector<double>>("interface_params.map_colors"),
    declare_parameter<std::vector<std::string>>("class_mapping"),
    declare_parameter<std::vector<std::string>>("model_params.object_class_names")),
  front_camera_id_(declare_parameter<int32_t>("sync_params.front_camera_id")),
  trajectory_timestep_(declare_parameter<double>("interface_params.trajectory_timestep")),
  vad_input_topic_data_current_frame_(num_cameras_)
{
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
  vad_interface_ptr_ = std::make_unique<VadInterface>(vad_interface_config_, tf_buffer_shared);

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
  vad_config.num_cameras = this->get_parameter("node_params.num_cameras").as_int();

  const auto declare_network_param = [this](const std::string & key) {
    return this->declare_parameter<int32_t>("model_params.network_io_params." + key);
  };

  vad_config.bev_h = declare_network_param("bev_h");
  vad_config.bev_w = declare_network_param("bev_w");
  vad_config.bev_feature_dim = declare_network_param("bev_feature_dim");
  vad_config.downsample_factor = declare_network_param("downsample_factor");
  vad_config.num_decoder_layers = declare_network_param("num_decoder_layers");
  vad_config.prediction_num_queries = declare_network_param("prediction_num_queries");
  vad_config.prediction_num_classes = declare_network_param("prediction_num_classes");
  vad_config.prediction_bbox_pred_dim = declare_network_param("prediction_bbox_pred_dim");
  vad_config.prediction_trajectory_modes = declare_network_param("prediction_trajectory_modes");
  vad_config.prediction_timesteps = declare_network_param("prediction_timesteps");
  vad_config.planning_ego_commands = declare_network_param("planning_ego_commands");
  vad_config.planning_timesteps = declare_network_param("planning_timesteps");
  vad_config.map_num_queries = declare_network_param("map_num_queries");
  vad_config.map_num_class = declare_network_param("map_num_class");
  vad_config.map_points_per_polylines = declare_network_param("map_points_per_polylines");
  vad_config.can_bus_dim = declare_network_param("can_bus_dim");

  vad_config.target_image_width =
    this->get_parameter("interface_params.target_image_width").as_int();
  vad_config.target_image_height =
    this->get_parameter("interface_params.target_image_height").as_int();

  load_detection_range(vad_config);
  load_map_configuration(vad_config);
  load_object_configuration(vad_config);

  vad_config.plugins_path = this->declare_parameter<std::string>("model_params.plugins_path");
  vad_config.input_image_width =
    this->declare_parameter<int32_t>("interface_params.input_image_width");
  vad_config.input_image_height =
    this->declare_parameter<int32_t>("interface_params.input_image_height");

  load_image_normalization(vad_config);
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

void VadNode::load_map_configuration(VadConfig & config)
{
  const auto map_class_names =
    this->get_parameter("model_params.map_class_names").as_string_array();
  const auto map_thresholds =
    this->get_parameter("model_params.map_confidence_thresholds").as_double_array();

  if (map_class_names.size() != map_thresholds.size()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "map_class_names (%zu) and map_confidence_thresholds (%zu) must have the same size",
      map_class_names.size(), map_thresholds.size());
    throw std::runtime_error(
      "Parameter array length mismatch: map_class_names and map_confidence_thresholds");
  }

  config.map_class_names.assign(map_class_names.begin(), map_class_names.end());
  config.map_num_classes = static_cast<int32_t>(map_class_names.size());
  config.map_confidence_thresholds.clear();

  for (std::size_t i = 0; i < map_class_names.size(); ++i) {
    config.map_confidence_thresholds[map_class_names[i]] = static_cast<float>(map_thresholds[i]);
  }
}

void VadNode::load_object_configuration(VadConfig & config)
{
  const auto object_class_names =
    this->get_parameter("model_params.object_class_names").as_string_array();
  const auto object_thresholds =
    this->get_parameter("model_params.object_confidence_thresholds").as_double_array();

  if (object_class_names.size() != object_thresholds.size()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "object_class_names (%zu) and object_confidence_thresholds (%zu) must have the same size",
      object_class_names.size(), object_thresholds.size());
    throw std::runtime_error(
      "Parameter array length mismatch: object_class_names and object_confidence_thresholds");
  }

  config.bbox_class_names.assign(object_class_names.begin(), object_class_names.end());
  config.object_confidence_thresholds.clear();

  for (std::size_t i = 0; i < object_class_names.size(); ++i) {
    config.object_confidence_thresholds[object_class_names[i]] =
      static_cast<float>(object_thresholds[i]);
  }
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
      get_transform_matrix(*vad_input_topic_data.kinematic_state);
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
// NOLINTNEXTLINE(readability-identifier-naming,cppcoreguidelines-avoid-non-const-global-variables)
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_vad::VadNode)
