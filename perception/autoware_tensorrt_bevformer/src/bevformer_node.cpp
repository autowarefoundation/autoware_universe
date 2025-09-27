// Copyright 2025 The Autoware Contributors
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
//
/*
 * Copyright (c) 2025 Multicoreware, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// cspell:ignore thre, caminfo, intrin, Canbus, BEVFormer

#include "bevformer_node.hpp"

#include "marker_util.hpp"
#include "postprocessing/postprocessing.hpp"
#include "preprocessing/preprocessing_pipeline.hpp"
#include "ros_utils.hpp"

#include <Eigen/LU>

#include "autoware_localization_msgs/msg/kinematic_state.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <autoware_perception_msgs/msg/detected_objects.hpp>

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <dlfcn.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace autoware
{
namespace tensorrt_bevformer
{

TRTBEVFormerNode::TRTBEVFormerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("tensorrt_bevformer_node", node_options)
{
  RCLCPP_INFO(this->get_logger(), "=== Starting BEVFormer Node Initialization ===");

  // Data parameters
  img_N_ = this->get_parameter_or("data_params.CAM_NUM", 6);
  this->declare_parameter<std::vector<double>>("data_params.mean", {103.530, 116.280, 123.675});
  src_img_w_ = this->declare_parameter<int>("data_params.input_width", 1600);
  src_img_h_ = this->declare_parameter<int>("data_params.input_height", 900);
  this->declare_parameter<std::vector<double>>("data_params.std", {1.0, 1.0, 1.0});
  this->declare_parameter<bool>("data_params.to_rgb", false);
  this->declare_parameter<int>("data_params.pad_divisor", 32);
  this->declare_parameter<double>("data_params.scale_factor", 0.8);

  // Model parameters
  model_shape_params_["batch_size"] = this->declare_parameter<int>("model_params.batch_size", 1);
  model_shape_params_["cameras"] = this->declare_parameter<int>("data_params.CAM_NUM", 6);
  model_shape_params_["bev_h"] = this->declare_parameter<int>("model_params.bev_h", 150);
  model_shape_params_["bev_w"] = this->declare_parameter<int>("model_params.bev_w", 150);
  model_shape_params_["nb_dec"] = this->declare_parameter<int>("model_params.nb_dec", 6);
  model_shape_params_["dim"] = this->declare_parameter<int>("model_params.dim", 256);
  model_shape_params_["num_query"] = this->declare_parameter<int>("model_params.num_query", 900);
  model_shape_params_["num_classes"] = this->declare_parameter<int>("model_params.num_classes", 10);
  model_shape_params_["code_size"] = this->declare_parameter<int>("model_params.code_size", 10);

  // Post-process parameters
  score_thre_ = this->declare_parameter<float>("post_process_params.score_thre", 0.2f);
  has_twist_ = this->declare_parameter<bool>("post_process_params.has_twist", true);
  class_names_ = this->declare_parameter<std::vector<std::string>>(
    "post_process_params.class_names", {"car", "truck", "bus", "bicycle", "motorcycle",
                                        "pedestrian", "traffic_cone", "barrier", "unknown"});
  pc_range_ = this->declare_parameter<std::vector<double>>(
    "post_process_params.pc_range", {-51.2, -51.2, -5.0, 51.2, 51.2, 3.0});
  post_center_range_ = this->declare_parameter<std::vector<double>>(
    "post_process_params.post_center_range", {-61.2, -61.2, -10.0, 61.2, 61.2, 10.0});

  // Output shape definitions
  model_output_shapes_["bev_embed"] = this->declare_parameter<std::vector<std::string>>(
    "model_params.output_shapes.bev_embed", {"bev_h*bev_w", "batch_size", "dim"});
  model_output_shapes_["outputs_classes"] = this->declare_parameter<std::vector<std::string>>(
    "model_params.output_shapes.outputs_classes",
    {"cameras", "batch_size", "num_query", "num_classes"});
  model_output_shapes_["outputs_coords"] = this->declare_parameter<std::vector<std::string>>(
    "model_params.output_shapes.outputs_coords",
    {"cameras", "batch_size", "num_query", "code_size"});

  // Engine and ONNX file parameters
  engine_file_ = this->declare_parameter<std::string>("model_params.engine_file", "");
  onnx_file_ = this->declare_parameter<std::string>("model_params.onnx_file", "");
  workspace_size_ = this->declare_parameter<int>("model_params.workspace_size", 4096);
  auto_convert_ = this->declare_parameter<bool>("model_params.auto_convert", true);
  precision_ = this->declare_parameter<std::string>("model_params.precision", "fp16");
  debug_mode_ = this->declare_parameter<bool>("post_process_params.debug_mode", false);

  RCLCPP_INFO(this->get_logger(), "Debug mode: %s", debug_mode_ ? "enabled" : "disabled");

  // Initialize data structures
  caminfo_received_ = std::vector<bool>(img_N_, false);
  cams_intrin_ = std::vector<Eigen::Matrix3d>(img_N_);
  cams2ego_rot_ = std::vector<Eigen::Quaternion<double>>(img_N_);
  cams2ego_trans_ = std::vector<Eigen::Translation3d>(img_N_);
  viewpad_matrices_.resize(6);
  sensor2lidar_rotation_.resize(6);
  sensor2lidar_translation_.resize(6);

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create publisher for detected
  pub_boxes_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "~/output_boxes", rclcpp::QoS{1});

  // Only create a marker publisher if debug mode is enabled in launch command
  if (debug_mode_) {
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/output_bboxes", rclcpp::QoS{1});
  }

  // Initialize model and subscriptions
  RCLCPP_INFO(this->get_logger(), "Initializing TensorRT engine...");
  initModel();

  RCLCPP_INFO(this->get_logger(), "Starting camera info subscription...");
  startCameraInfoSubscription();

  RCLCPP_INFO(this->get_logger(), "Waiting for camera info...");
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&TRTBEVFormerNode::checkInitialization, this));

  RCLCPP_INFO(this->get_logger(), "=== BEVFormer Node Initialization Complete ===");
}

void TRTBEVFormerNode::initModel()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Loading config...");

  // Initialize modular components
  try {
    RCLCPP_INFO(this->get_logger(), "Initializing data manager...");
    data_manager_ = std::make_unique<BEVFormerDataManager>(this->get_logger());

    RCLCPP_INFO(this->get_logger(), "Initializing inference engine...");
    inference_engine_ = std::make_unique<BEVFormerInferenceEngine>(this->get_logger());

    RCLCPP_INFO(this->get_logger(), "Initializing preprocessor...");
    preprocessor_ = std::make_unique<BEVFormerPreprocessor>(this->get_logger(), this);

    RCLCPP_INFO(this->get_logger(), "All modules initialized successfully");
  } catch (const std::bad_alloc & e) {
    RCLCPP_ERROR(this->get_logger(), "Memory allocation failed: %s", e.what());
    throw;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize modules: %s", e.what());
    throw;
  }

  // Initialize the TensorRT engine
  try {
    // Check if auto-convert is enabled
    if (auto_convert_ && !onnx_file_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Auto-conversion enabled. Checking for engine file...");

      // If engine file is not specified, generate it from the ONNX file
      if (engine_file_.empty()) {
        // Create engine file path by replacing .onnx with .engine
        std::string base_path = onnx_file_;
        size_t pos = base_path.rfind(".onnx");
        if (pos != std::string::npos) {
          base_path.replace(pos, 5, ".engine");
        } else {
          base_path += ".engine";
        }
        engine_file_ = base_path;
        RCLCPP_INFO(this->get_logger(), "Auto-generated engine path: %s", engine_file_.c_str());
      }

      // Determine precision type
      PrecisionType precision_type = PrecisionType::FP16;  // Default to FP16
      if (precision_ == "fp32") {
        precision_type = PrecisionType::FP32;
        RCLCPP_INFO(this->get_logger(), "Using FP32 precision as specified");
      } else {
        RCLCPP_INFO(this->get_logger(), "Using FP16 precision (default or specified)");
      }

      // Build engine from ONNX (this will skip if engine already exists)
      if (!inference_engine_->buildEngineFromOnnx(
            onnx_file_, engine_file_, plugin_path_, workspace_size_, precision_type)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to build engine from ONNX");
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully built/loaded engine");
        // Initialize data manager with prev_bev shape
        data_manager_->initializePrevBev(inference_engine_->getInputPrevBevShape());
      }
    } else {
      if (!engine_file_.empty()) {
        if (!inference_engine_->initialize(engine_file_, plugin_path_)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to initialize TensorRT inference engine");
        } else {
          RCLCPP_INFO(this->get_logger(), "TensorRT inference engine initialized successfully");
          data_manager_->initializePrevBev(inference_engine_->getInputPrevBevShape());
        }
      } else {
        RCLCPP_ERROR(
          this->get_logger(), "Neither engine_file nor onnx_file with auto_convert is specified");
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Exception during TensorRT engine initialization: %s", e.what());
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "BEVFormer initialization complete");
}

void TRTBEVFormerNode::checkInitialization()
{
  // Check camera info and static transforms
  bool all_camera_info =
    std::all_of(caminfo_received_.begin(), caminfo_received_.end(), [](bool v) { return v; });

  if (all_camera_info && lidar2ego_transforms_ready_) {
    RCLCPP_INFO(
      this->get_logger(),
      "All camera info and static transforms ready, starting image processing...");
    timer_->cancel();
    startImageSubscription();
    RCLCPP_INFO(this->get_logger(), "Image processing pipeline is now active.");
  } else {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Waiting for initialization...");

    if (!all_camera_info) {
      for (size_t i = 0; i < caminfo_received_.size(); ++i) {
        if (!caminfo_received_[i]) {
          RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000, "Camera %zu info not received", i);
        }
      }
    }
    if (!lidar2ego_transforms_ready_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "Static lidar2ego transform not ready");
    }
  }
}

void TRTBEVFormerNode::startImageSubscription()
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  using std::placeholders::_5;
  using std::placeholders::_6;
  using std::placeholders::_7;

  // Subscribe to topics
  sub_fl_img_.subscribe(this, "~/input/topic_img_fl", rclcpp::QoS{1}.get_rmw_qos_profile());
  sub_f_img_.subscribe(this, "~/input/topic_img_f", rclcpp::QoS{1}.get_rmw_qos_profile());
  sub_fr_img_.subscribe(this, "~/input/topic_img_fr", rclcpp::QoS{1}.get_rmw_qos_profile());
  sub_bl_img_.subscribe(this, "~/input/topic_img_bl", rclcpp::QoS{1}.get_rmw_qos_profile());
  sub_b_img_.subscribe(this, "~/input/topic_img_b", rclcpp::QoS{1}.get_rmw_qos_profile());
  sub_br_img_.subscribe(this, "~/input/topic_img_br", rclcpp::QoS{1}.get_rmw_qos_profile());
  sub_kinematic_state.subscribe(this, "~/input/can_bus", rclcpp::QoS{1}.get_rmw_qos_profile());

  // Synchronize
  sync_ = std::make_shared<Sync>(
    MultiSensorSyncPolicy(10), sub_fl_img_, sub_f_img_, sub_fr_img_, sub_bl_img_, sub_b_img_,
    sub_br_img_, sub_kinematic_state);

  sync_->registerCallback(std::bind(&TRTBEVFormerNode::callback, this, _1, _2, _3, _4, _5, _6, _7));
}

void TRTBEVFormerNode::startCameraInfoSubscription()
{
  sub_f_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_f/camera_info", rclcpp::QoS{1},
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(0, msg); });

  sub_fr_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_fr/camera_info", rclcpp::QoS{1},
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(1, msg); });

  sub_fl_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_fl/camera_info", rclcpp::QoS{1},
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(2, msg); });

  sub_b_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_b/camera_info", rclcpp::QoS{1},
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(3, msg); });

  sub_bl_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_bl/camera_info", rclcpp::QoS{1},
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(4, msg); });

  sub_br_caminfo_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/topic_img_br/camera_info", rclcpp::QoS{1},
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(5, msg); });
}

void TRTBEVFormerNode::cameraInfoCallback(
  int idx, const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (caminfo_received_[idx]) {
    return;
  }

  // Get camera intrinsics
  Eigen::Matrix3d intrinsics;
  getCameraIntrinsics(msg, intrinsics);
  cams_intrin_[idx] = intrinsics;

  // Get camera to ego transform (static)
  Eigen::Quaternion<double> rot;
  Eigen::Translation3d translation;
  getTransform(
    tf_buffer_->lookupTransform("base_link", msg->header.frame_id, rclcpp::Time(0)), rot,
    translation);
  cams2ego_rot_[idx] = rot;
  cams2ego_trans_[idx] = translation;

  Eigen::Matrix4d viewpad = Eigen::Matrix4d::Identity();
  viewpad.block<3, 3>(0, 0) = intrinsics;
  viewpad_matrices_[idx] = viewpad;

  caminfo_received_[idx] = true;
  camera_info_received_flag_ =
    std::all_of(caminfo_received_.begin(), caminfo_received_.end(), [](bool i) { return i; });

  // Calculate static lidar2ego transform only once when all camera info are available
  if (camera_info_received_flag_ && !lidar2ego_transforms_ready_) {
    calculateStaticLidar2EgoTransform();
  }
}

void TRTBEVFormerNode::calculateStaticLidar2EgoTransform()
{
  rclcpp::Time lidar_time = this->now();

  try {
    if (tf_buffer_->canTransform(
          "base_link", "LIDAR_TOP", lidar_time, rclcpp::Duration::from_seconds(1.0))) {
      auto tf_lidar2ego = tf_buffer_->lookupTransform("base_link", "LIDAR_TOP", lidar_time);
      getTransform(tf_lidar2ego, lidar2ego_rot_static_, lidar2ego_trans_static_);
      lidar2ego_transforms_ready_ = true;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Static lidar2ego transform lookup failed: %s", ex.what());
  }
}

void TRTBEVFormerNode::calculateSensor2LidarTransformsFromTF(
  const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> & image_msgs,
  const Eigen::Quaterniond & ego2global_rot_ref, const Eigen::Translation3d & ego2global_trans_ref)
{
  if (!lidar2ego_transforms_ready_) {
    RCLCPP_ERROR(this->get_logger(), "Static lidar2ego transforms not ready");
    return;
  }

  // Use static lidar2ego and ego2global from LiDAR timestamp
  Eigen::Matrix3d l2e_r_mat = lidar2ego_rot_static_.cast<double>().toRotationMatrix();
  Eigen::RowVector3d l2e_t = lidar2ego_trans_static_.cast<double>().vector().transpose();
  Eigen::Matrix3d e2g_r_mat = ego2global_rot_ref.toRotationMatrix();
  Eigen::RowVector3d e2g_t = ego2global_trans_ref.vector().transpose();

  std::vector<std::string> camera_names = {"CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_FRONT_LEFT",
                                           "CAM_BACK",  "CAM_BACK_LEFT",   "CAM_BACK_RIGHT"};

  for (size_t i = 0; i < img_N_; ++i) {
    if (i >= image_msgs.size()) {
      RCLCPP_ERROR(this->get_logger(), "Camera %zu data not available", i);
      continue;
    }

    std::string camera_name = camera_names[i];
    rclcpp::Time camera_timestamp = image_msgs[i]->header.stamp;

    // Get sensor2ego transforms
    Eigen::Matrix3d l2e_r_s_mat = cams2ego_rot_[i].cast<double>().toRotationMatrix();
    Eigen::RowVector3d l2e_t_s = cams2ego_trans_[i].cast<double>().vector().transpose();

    // Get ego2global at camera timestamp
    Eigen::Quaterniond ego2global_rot_cam;
    Eigen::Translation3d ego2global_trans_cam;
    bool got_camera_transform = false;
    try {
      if (tf_buffer_->canTransform(
            "world", "base_link", camera_timestamp, rclcpp::Duration::from_seconds(0.1))) {
        auto tf_ego2global_cam = tf_buffer_->lookupTransform(
          "world", "base_link", camera_timestamp, rclcpp::Duration::from_seconds(0.1));

        ego2global_rot_cam = Eigen::Quaterniond(
          tf_ego2global_cam.transform.rotation.w, tf_ego2global_cam.transform.rotation.x,
          tf_ego2global_cam.transform.rotation.y, tf_ego2global_cam.transform.rotation.z);
        ego2global_trans_cam = Eigen::Translation3d(
          tf_ego2global_cam.transform.translation.x, tf_ego2global_cam.transform.translation.y,
          tf_ego2global_cam.transform.translation.z);
        got_camera_transform = true;
      }
    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        this->get_logger(), "Camera %s: Failed exact lookup: %s", camera_name.c_str(), ex.what());
    }
    if (!got_camera_transform) {
      RCLCPP_DEBUG(this->get_logger(), "Camera transform for ego2global are not found");
    }

    Eigen::Matrix3d e2g_r_s_mat = ego2global_rot_cam.toRotationMatrix();
    Eigen::RowVector3d e2g_t_s = ego2global_trans_cam.vector().transpose();

    // Compute matrix inverses
    Eigen::Matrix3d e2g_r_mat_inv = e2g_r_mat.inverse();
    Eigen::Matrix3d l2e_r_mat_inv = l2e_r_mat.inverse();

    // Calculate R matrix
    Eigen::Matrix3d R = (l2e_r_s_mat.transpose() * e2g_r_s_mat.transpose()) *
                        (e2g_r_mat_inv.transpose() * l2e_r_mat_inv.transpose());

    // Calculate T vector
    Eigen::Matrix3d combined_inv = e2g_r_mat_inv.transpose() * l2e_r_mat_inv.transpose();
    Eigen::RowVector3d T_initial = (l2e_t_s * e2g_r_s_mat.transpose() + e2g_t_s) * combined_inv;

    Eigen::RowVector3d subtract_term1 = e2g_t * combined_inv;
    Eigen::RowVector3d subtract_term2 = l2e_t * l2e_r_mat_inv.transpose();

    Eigen::RowVector3d subtract_sum = subtract_term1 + subtract_term2;

    Eigen::RowVector3d T_row = T_initial - (subtract_term1 + subtract_term2);

    Eigen::Vector3d T = T_row.transpose();

    sensor2lidar_rotation_[i] = R.transpose();
    sensor2lidar_translation_[i] = T;
  }
}

std::vector<float> TRTBEVFormerNode::extractCanBusFromKinematicState(
  const autoware_localization_msgs::msg::KinematicState::ConstSharedPtr & kinematic_state_msg)
{
  std::vector<float> can_bus_data(18, 0.0f);

  // Map position (3 values)
  can_bus_data[0] = static_cast<float>(kinematic_state_msg->pose_with_covariance.pose.position.x);
  can_bus_data[1] = static_cast<float>(kinematic_state_msg->pose_with_covariance.pose.position.y);
  can_bus_data[2] = static_cast<float>(kinematic_state_msg->pose_with_covariance.pose.position.z);

  // Map orientation (4 values: w, x, y, z)
  can_bus_data[3] =
    static_cast<float>(kinematic_state_msg->pose_with_covariance.pose.orientation.w);
  can_bus_data[4] =
    static_cast<float>(kinematic_state_msg->pose_with_covariance.pose.orientation.x);
  can_bus_data[5] =
    static_cast<float>(kinematic_state_msg->pose_with_covariance.pose.orientation.y);
  can_bus_data[6] =
    static_cast<float>(kinematic_state_msg->pose_with_covariance.pose.orientation.z);

  // Map linear velocity (3 values)
  can_bus_data[7] = static_cast<float>(kinematic_state_msg->twist_with_covariance.twist.linear.x);
  can_bus_data[8] = static_cast<float>(kinematic_state_msg->twist_with_covariance.twist.linear.y);
  can_bus_data[9] = static_cast<float>(kinematic_state_msg->twist_with_covariance.twist.linear.z);

  // Map angular velocity (3 values)
  can_bus_data[10] = static_cast<float>(kinematic_state_msg->twist_with_covariance.twist.angular.x);
  can_bus_data[11] = static_cast<float>(kinematic_state_msg->twist_with_covariance.twist.angular.y);
  can_bus_data[12] = static_cast<float>(kinematic_state_msg->twist_with_covariance.twist.angular.z);

  // Map linear acceleration (3 values)
  can_bus_data[13] = static_cast<float>(kinematic_state_msg->accel_with_covariance.accel.linear.x);
  can_bus_data[14] = static_cast<float>(kinematic_state_msg->accel_with_covariance.accel.linear.y);
  can_bus_data[15] = static_cast<float>(kinematic_state_msg->accel_with_covariance.accel.linear.z);

  // Padding for compatibility (2 values)
  can_bus_data[16] = 0.0f;
  can_bus_data[17] = 0.0f;

  return can_bus_data;
}

void TRTBEVFormerNode::callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_fl_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_f_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_fr_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_bl_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_b_img,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_br_img,
  const autoware_localization_msgs::msg::KinematicState::ConstSharedPtr & can_bus_msg)
{
  auto t_preprocess_start = std::chrono::steady_clock::now();

  std::vector<float> latest_can_bus = extractCanBusFromKinematicState(can_bus_msg);

  // Create vector of image messages for transform calculation
  std::vector<sensor_msgs::msg::Image::ConstSharedPtr> image_msgs = {
    msg_f_img, msg_fr_img, msg_fl_img, msg_b_img, msg_bl_img, msg_br_img};

  rclcpp::Time ref_time = this->now();
  RCLCPP_DEBUG(this->get_logger(), "Ref time Initialized");
  Eigen::Quaterniond ego2global_rot;
  Eigen::Translation3d ego2global_trans;

  try {
    if (tf_buffer_->canTransform(
          "world", "base_link", ref_time, rclcpp::Duration::from_seconds(0.5))) {
      auto tf_ego2global = tf_buffer_->lookupTransform("world", "base_link", ref_time);
      getTransform(tf_ego2global, ego2global_rot, ego2global_trans);
      RCLCPP_DEBUG(this->get_logger(), "Got ego2global transform for CAN bus processing");
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Missing world->base_link transform at time %f", ref_time.seconds());
      return;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Transform lookup failed at time %f: %s", ref_time.seconds(), ex.what());
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "Transform functions done");

  // Update sensor2lidar transforms for each camera with its own timestamp
  calculateSensor2LidarTransformsFromTF(image_msgs, ego2global_rot, ego2global_trans);

  // Image conversion
  cv::Mat img_fl, img_f, img_fr, img_bl, img_b, img_br;
  try {
    img_fl = cloneAndResize(msg_fl_img);
    img_f = cloneAndResize(msg_f_img);
    img_fr = cloneAndResize(msg_fr_img);
    img_bl = cloneAndResize(msg_bl_img);
    img_b = cloneAndResize(msg_b_img);
    img_br = cloneAndResize(msg_br_img);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
    return;
  }

  std::vector<cv::Mat> raw_images = {img_f, img_fr, img_fl, img_b, img_bl, img_br};

  // Processing can_bus data
  std::vector<float> ego2global_translation_data = {
    static_cast<float>(ego2global_trans.x()), static_cast<float>(ego2global_trans.y()),
    static_cast<float>(ego2global_trans.z())};

  std::vector<float> ego2global_rotation_data = {
    static_cast<float>(ego2global_rot.w()), static_cast<float>(ego2global_rot.x()),
    static_cast<float>(ego2global_rot.y()), static_cast<float>(ego2global_rot.z())};

  std::vector<float> initial_can_bus = data_manager_->processCanBus(
    ego2global_translation_data, ego2global_rotation_data, latest_can_bus);

  if (!preprocessor_) {
    RCLCPP_ERROR(this->get_logger(), "Preprocessor is not initialized!");
    return;
  }

  // Use the preprocessor to process the images
  BEVFormerStructuredInput structured_input = preprocessor_->preprocessImages(
    raw_images, sensor2lidar_rotation_, sensor2lidar_translation_, cams_intrin_);

  auto t_preprocess_end = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    this->get_logger(), "Pre-processing : %ld ms",
    std::chrono::duration_cast<std::chrono::milliseconds>(t_preprocess_end - t_preprocess_start)
      .count());

  auto t_inference_start = t_preprocess_end;

  // Get use_prev_bev flag ONCE at the beginning of inference phase
  float use_prev_bev = data_manager_->getUsePrevBev();

  // Apply temporal processing with the determined flag
  std::vector<float> processed_can_bus =
    data_manager_->processCanbusWithTemporal(initial_can_bus, use_prev_bev);

  // Get the previous BEV features
  const std::vector<float> & prev_bev = data_manager_->getPrevBev();

  if (!inference_engine_ || !inference_engine_->isInitialized()) {
    RCLCPP_ERROR(this->get_logger(), "Inference engine is not initialized!");
    return;
  }

  // Run inference
  auto [outputs_classes, outputs_coords, bev_embed] = inference_engine_->runInference(
    structured_input.img_tensor, prev_bev, use_prev_bev, processed_can_bus,
    structured_input.lidar2img_flat);

  auto t_inference_end = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    this->get_logger(), "Inference : %ld ms",
    std::chrono::duration_cast<std::chrono::milliseconds>(t_inference_end - t_inference_start)
      .count());

  auto t_postprocess_start = t_inference_end;

  // Reshape outputs
  std::map<std::string, std::vector<float>> rawOutputs;
  rawOutputs["outputs_classes"] = outputs_classes;
  rawOutputs["outputs_coords"] = outputs_coords;
  rawOutputs["bev_embed"] = bev_embed;

  if (rawOutputs.count("bev_embed")) {
    data_manager_->updatePrevBev(std::move(rawOutputs["bev_embed"]));
    rawOutputs.erase("bev_embed");
  } else {
    RCLCPP_WARN(this->get_logger(), "Reshaped BEV embedding not found in outputs!");
  }

  // Update prev_frame_info AFTER inference
  data_manager_->updatePrevFrameInfo();

  // Post Processing
  std::map<std::string, std::vector<double>> reshapedOutputs;
  for (const auto & [key, vec] : rawOutputs) {
    reshapedOutputs[key] = std::vector<double>(vec.begin(), vec.end());
  }

  int max_num = this->get_parameter_or("post_process_params.max_num", 300);

  PostProcessor postProcessor(
    model_shape_params_["nb_dec"], model_shape_params_["num_query"],
    model_shape_params_["num_classes"], model_shape_params_["code_size"],
    model_shape_params_["bev_h"], model_shape_params_["bev_w"], score_thre_, max_num, pc_range_,
    post_center_range_);

  auto t_postprocess_end = std::chrono::steady_clock::now();
  RCLCPP_INFO(
    this->get_logger(), "Post-processing : %ld ms",
    std::chrono::duration_cast<std::chrono::milliseconds>(t_postprocess_end - t_postprocess_start)
      .count());

  std::vector<Box3D> batch_results;
  try {
    batch_results = postProcessor.post_process(reshapedOutputs);
    RCLCPP_INFO(this->get_logger(), "Detections: %zu", batch_results.size());
    RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Post-process failed: %s", e.what());
    return;
  }

  autoware_perception_msgs::msg::DetectedObjects bevformer_objects;
  bevformer_objects.header.frame_id = "LIDAR_TOP";
  bevformer_objects.header.stamp = msg_f_img->header.stamp;

  box3DToDetectedObjects(batch_results, bevformer_objects, class_names_, score_thre_, has_twist_);

  pub_boxes_->publish(bevformer_objects);

  // Only publish debug markers if debug mode is enabled
  if (debug_mode_ && pub_markers_) {
    publishDebugMarkers(pub_markers_, bevformer_objects);
  }
}

// Helper function to clone and resize an image
cv::Mat TRTBEVFormerNode::cloneAndResize(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  if (img.size() != cv::Size(src_img_w_, src_img_h_)) {
    cv::resize(img, img, cv::Size(src_img_w_, src_img_h_));
  }
  return img;
}

}  // namespace tensorrt_bevformer
}  // namespace autoware

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_bevformer::TRTBEVFormerNode)
