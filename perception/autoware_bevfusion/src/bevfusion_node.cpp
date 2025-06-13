// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/bevfusion/bevfusion_node.hpp"

#include "autoware/bevfusion/utils.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::bevfusion
{

BEVFusionNode::BEVFusionNode(const rclcpp::NodeOptions & options)
: Node("bevfusion", options), tf_buffer_(this->get_clock())
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor{}.set__read_only(true);

  // Modality
  sensor_fusion_ = this->declare_parameter<bool>("sensor_fusion", descriptor);

  // Non network parameters
  max_camera_lidar_delay_ = this->declare_parameter<float>("max_camera_lidar_delay", descriptor);

  // TensorRT parameters
  const std::string plugins_path = this->declare_parameter<std::string>("plugins_path", descriptor);

  // Network parameters
  const std::string onnx_path = this->declare_parameter<std::string>("onnx_path", descriptor);
  const std::string engine_path = this->declare_parameter<std::string>("engine_path", descriptor);
  const std::string trt_precision =
    this->declare_parameter<std::string>("trt_precision", descriptor);

  // Common parameters
  const auto out_size_factor = this->declare_parameter<std::int64_t>("out_size_factor", descriptor);

  auto to_float_vector = [](const auto & v) -> std::vector<float> {
    return std::vector<float>(v.begin(), v.end());
  };

  // Lidar branch parameters
  const auto cloud_capacity = this->declare_parameter<std::int64_t>("cloud_capacity", descriptor);
  const auto max_points_per_voxel =
    this->declare_parameter<std::int64_t>("max_points_per_voxel", descriptor);
  const auto voxels_num =
    this->declare_parameter<std::vector<std::int64_t>>("voxels_num", descriptor);
  const auto point_cloud_range =
    to_float_vector(this->declare_parameter<std::vector<double>>("point_cloud_range", descriptor));
  const auto voxel_size =
    to_float_vector(this->declare_parameter<std::vector<double>>("voxel_size", descriptor));

  // Camera branch parameters
  const auto d_bound =
    to_float_vector(this->declare_parameter<std::vector<double>>("d_bound", descriptor));
  const auto x_bound =
    to_float_vector(this->declare_parameter<std::vector<double>>("x_bound", descriptor));
  const auto y_bound =
    to_float_vector(this->declare_parameter<std::vector<double>>("y_bound", descriptor));
  const auto z_bound =
    to_float_vector(this->declare_parameter<std::vector<double>>("z_bound", descriptor));
  const auto num_cameras = this->declare_parameter<std::int64_t>("num_cameras", descriptor);
  const auto raw_image_height =
    this->declare_parameter<std::int64_t>("raw_image_height", descriptor);
  const auto raw_image_width = this->declare_parameter<std::int64_t>("raw_image_width", descriptor);
  const auto img_aug_scale_x = this->declare_parameter<float>("img_aug_scale_x", descriptor);
  const auto img_aug_scale_y = this->declare_parameter<float>("img_aug_scale_y", descriptor);
  const auto roi_height = this->declare_parameter<std::int64_t>("roi_height", descriptor);
  const auto roi_width = this->declare_parameter<std::int64_t>("roi_width", descriptor);
  const auto features_height = this->declare_parameter<std::int64_t>("features_height", descriptor);
  const auto features_width = this->declare_parameter<int>("features_width", descriptor);
  const auto num_depth_features = this->declare_parameter<int>("num_depth_features", descriptor);

  // Head parameters
  const auto num_proposals = this->declare_parameter<std::int64_t>("num_proposals", descriptor);
  class_names_ = this->declare_parameter<std::vector<std::string>>("class_names", descriptor);

  if (point_cloud_range.size() != 6) {
    RCLCPP_ERROR(rclcpp::get_logger("bevfusion"), "The size of point_cloud_range != 6");

    throw std::runtime_error("The size of point_cloud_range != 6");
  }
  if (voxel_size.size() != 3) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("bevfusion"), "The size of voxel_size != 3");

    throw std::runtime_error("The size of voxel_size != 3");
  }

  // pre-process
  const std::string densification_world_frame_id =
    this->declare_parameter<std::string>("densification_world_frame_id", descriptor);
  const int densification_num_past_frames =
    this->declare_parameter<std::int64_t>("densification_num_past_frames", descriptor);

  // post-process
  const float circle_nms_dist_threshold =
    static_cast<float>(this->declare_parameter<double>("circle_nms_dist_threshold", descriptor));
  {  // IoU NMS
    NMSParams p;
    p.search_distance_2d_ =
      this->declare_parameter<double>("iou_nms_search_distance_2d", descriptor);
    p.iou_threshold_ = this->declare_parameter<double>("iou_nms_threshold", descriptor);
    iou_bev_nms_.setParameters(p);
  }
  const auto yaw_norm_thresholds =
    this->declare_parameter<std::vector<double>>("yaw_norm_thresholds", descriptor);
  const float score_threshold =
    static_cast<float>(this->declare_parameter<double>("score_threshold", descriptor));

  DensificationParam densification_param(
    densification_world_frame_id, densification_num_past_frames);

  BEVFusionConfig config(
    sensor_fusion_, plugins_path, out_size_factor, cloud_capacity, max_points_per_voxel, voxels_num,
    point_cloud_range, voxel_size, d_bound, x_bound, y_bound, z_bound, num_cameras,
    raw_image_height, raw_image_width, img_aug_scale_x, img_aug_scale_y, roi_height, roi_width,
    features_height, features_width, num_depth_features, num_proposals, circle_nms_dist_threshold,
    yaw_norm_thresholds, score_threshold);

  const auto allow_remapping_by_area_matrix = this->declare_parameter<std::vector<std::int64_t>>(
    "allow_remapping_by_area_matrix", descriptor);
  const auto min_area_matrix =
    this->declare_parameter<std::vector<double>>("min_area_matrix", descriptor);
  const auto max_area_matrix =
    this->declare_parameter<std::vector<double>>("max_area_matrix", descriptor);
  detection_class_remapper_.setParameters(
    allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

  auto trt_config =
    tensorrt_common::TrtCommonConfig(onnx_path, trt_precision, engine_path, 1ULL << 32U);
  detector_ptr_ = std::make_unique<BEVFusionTRT>(trt_config, densification_param, config);
  diagnostics_detector_trt_ =
    std::make_unique<autoware_utils::DiagnosticsInterface>(this, "bevfusion_trt");

  cloud_sub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(&BEVFusionNode::cloudCallback, this, std::placeholders::_1));

  objects_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "~/output/objects", rclcpp::QoS(1));

  if (sensor_fusion_) {
    image_subs_.resize(num_cameras);
    camera_info_subs_.resize(num_cameras);
    image_msgs_.resize(num_cameras);
    camera_info_msgs_.resize(num_cameras);
    lidar2camera_extrinsics_.resize(num_cameras);

    for (std::int64_t camera_id = 0; camera_id < num_cameras; ++camera_id) {
      image_subs_[camera_id] = this->create_subscription<sensor_msgs::msg::Image>(
        "~/input/image" + std::to_string(camera_id), rclcpp::SensorDataQoS{}.keep_last(1),
        [this, camera_id](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
          this->imageCallback(msg, camera_id);
        });

      camera_info_subs_[camera_id] = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "~/input/camera_info" + std::to_string(camera_id), rclcpp::SensorDataQoS{}.keep_last(1),
        [this, camera_id](const sensor_msgs::msg::CameraInfo & msg) {
          this->cameraInfoCallback(msg, camera_id);
        });
    }
  }

  published_time_pub_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);

  // initialize debug tool
  {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("cyclic");
    stop_watch_ptr_->tic("processing/total");
  }

  if (this->declare_parameter<bool>("build_only", false, descriptor)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine was built. Shutting down the node.");
    rclcpp::shutdown();
  }
}

void BEVFusionNode::cloudCallback(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & pc_msg_ptr)
{
  lidar_frame_ = pc_msg_ptr->header.frame_id;

  if (sensor_fusion_ && (!extrinsics_available_ || !images_available_ || !intrinsics_available_)) {
    return;
  }

  if (sensor_fusion_ && !intrinsics_extrinsics_precomputed_) {
    std::vector<sensor_msgs::msg::CameraInfo> camera_info_msgs;
    std::vector<Matrix4f> lidar2camera_extrinsics;

    std::transform(
      camera_info_msgs_.begin(), camera_info_msgs_.end(), std::back_inserter(camera_info_msgs),
      [](const auto & opt) { return *opt; });

    std::transform(
      lidar2camera_extrinsics_.begin(), lidar2camera_extrinsics_.end(),
      std::back_inserter(lidar2camera_extrinsics), [](const auto & opt) { return *opt; });

    detector_ptr_->setIntrinsicsExtrinsics(camera_info_msgs, lidar2camera_extrinsics);
    intrinsics_extrinsics_precomputed_ = true;
  }

  const auto objects_sub_count =
    objects_pub_->get_subscription_count() + objects_pub_->get_intra_process_subscription_count();
  if (objects_sub_count < 1) {
    return;
  }

  if (stop_watch_ptr_) {
    stop_watch_ptr_->toc("processing/total", true);
  }
  diagnostics_detector_trt_->clear();

  double lidar_stamp = rclcpp::Time(pc_msg_ptr->header.stamp).seconds();
  camera_masks_.resize(camera_info_msgs_.size());
  for (std::size_t i = 0; i < camera_masks_.size(); ++i) {
    camera_masks_[i] =
      (lidar_stamp - rclcpp::Time(image_msgs_[i]->header.stamp).seconds()) < max_camera_lidar_delay_
        ? 1.0
        : 0.f;
  }

  std::vector<Box3D> det_boxes3d;
  std::unordered_map<std::string, double> proc_timing;
  bool is_num_voxels_within_range = true;
  bool is_success = detector_ptr_->detect(
    pc_msg_ptr, image_msgs_, camera_masks_, tf_buffer_, det_boxes3d, proc_timing,
    is_num_voxels_within_range);
  if (!is_success) {
    return;
  }

  diagnostics_detector_trt_->add_key_value(
    "is_num_voxels_within_range", is_num_voxels_within_range);
  if (!is_num_voxels_within_range) {
    std::stringstream message;
    message << "BEVFusionTRT::detect: The actual number of voxels exceeds its maximum value, "
            << "which may limit the detection performance.";
    diagnostics_detector_trt_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }

  std::vector<autoware_perception_msgs::msg::DetectedObject> raw_objects;
  raw_objects.reserve(det_boxes3d.size());
  for (const auto & box3d : det_boxes3d) {
    autoware_perception_msgs::msg::DetectedObject obj;
    box3DToDetectedObject(box3d, class_names_, obj);
    raw_objects.emplace_back(obj);
  }

  autoware_perception_msgs::msg::DetectedObjects output_msg;
  output_msg.header = pc_msg_ptr->header;
  output_msg.objects = iou_bev_nms_.apply(raw_objects);

  detection_class_remapper_.mapClasses(output_msg);

  if (objects_sub_count > 0) {
    objects_pub_->publish(output_msg);
    published_time_pub_->publish_if_subscribed(objects_pub_, output_msg.header.stamp);
  }

  diagnostics_detector_trt_->publish(pc_msg_ptr->header.stamp);

  // add processing time for debug
  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing/total", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - output_msg.header.stamp).nanoseconds()))
        .count();
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time/total_ms", processing_time_ms);
    for (const auto & [topic, time_ms] : proc_timing) {
      debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
        topic, time_ms);
    }
  }
}

void BEVFusionNode::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg, std::size_t camera_id)
{
  image_msgs_[camera_id] = msg;

  std::size_t num_valid_images = std::count_if(
    image_msgs_.begin(), image_msgs_.end(),
    [](const auto & image_msg) { return image_msg != nullptr; });

  images_available_ = num_valid_images == image_msgs_.size();
}

void BEVFusionNode::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo & msg, std::size_t camera_id)
{
  camera_info_msgs_[camera_id] = msg;

  std::size_t num_valid_intrinsics = std::count_if(
    camera_info_msgs_.begin(), camera_info_msgs_.end(),
    [](const auto & opt) { return opt.has_value(); });

  intrinsics_available_ = num_valid_intrinsics == camera_info_msgs_.size();

  if (
    lidar2camera_extrinsics_[camera_id].has_value() || !lidar_frame_.has_value() ||
    extrinsics_available_) {
    return;
  }

  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped =
      tf_buffer_.lookupTransform(msg.header.frame_id, *lidar_frame_, msg.header.stamp);

    Eigen::Matrix4f lidar2camera_transform =
      tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();

    Matrix4f lidar2camera_rowmajor_transform = lidar2camera_transform.eval();
    lidar2camera_extrinsics_[camera_id] = lidar2camera_rowmajor_transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  std::size_t num_valid_extrinsics = std::count_if(
    lidar2camera_extrinsics_.begin(), lidar2camera_extrinsics_.end(),
    [](const auto & opt) { return opt.has_value(); });

  extrinsics_available_ = num_valid_extrinsics == lidar2camera_extrinsics_.size();
}

// Check the processing time and delayed timestamp
// If the node is consistently delayed, publish an error diagnostic message
void BEVFusionNode::diagnoseProcessingTime(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const rclcpp::Time timestamp_now = this->get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus::_level_type diag_level =
    diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::stringstream message{"OK"};

  // Check if the node has performed inference
  if (last_processing_time_ms_) {
    // check processing time is acceptable
    if (last_processing_time_ms_ > max_allowed_processing_time_ms_) {
      stat.add("is_processing_time_ms_in_expected_range", false);

      message.clear();
      message << "Processing time exceeds the acceptable limit of "
              << max_allowed_processing_time_ms_ << " ms by "
              << (last_processing_time_ms_.value() - max_allowed_processing_time_ms_) << " ms.";

      // in case the processing starts with a delayed inference
      if (!last_in_time_processing_timestamp_) {
        last_in_time_processing_timestamp_ = timestamp_now;
      }

      diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      stat.add("is_processing_time_ms_in_expected_range", true);
      last_in_time_processing_timestamp_ = timestamp_now;
    }
    stat.add("processing_time_ms", last_processing_time_ms_.value());

    const double delayed_state_duration =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (timestamp_now - last_in_time_processing_timestamp_.value()).nanoseconds()))
        .count();

    // check consecutive delays
    if (delayed_state_duration > max_acceptable_consecutive_delay_ms_) {
      stat.add("is_consecutive_processing_delay_in_range", false);

      message << " Processing delay has consecutively exceeded the acceptable limit continuously.";

      diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else {
      stat.add("is_consecutive_processing_delay_in_range", true);
    }
    stat.add("consecutive_processing_delay_ms", delayed_state_duration);
  } else {
    stat.add("is_processing_time_ms_in_expected_range", true);
    stat.add("processing_time_ms", 0.0);
    stat.add("is_consecutive_processing_delay_in_range", true);
    stat.add("consecutive_processing_delay_ms", 0.0);

    message << "Waiting for the node to perform inference.";
  }

  stat.summary(diag_level, message.str());
}

}  // namespace autoware::bevfusion

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::bevfusion::BEVFusionNode)
