// Copyright 2021 TIER IV, Inc.
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

#include "autoware/lidar_centerpoint/node.hpp"

#include "autoware/lidar_centerpoint/centerpoint_config.hpp"
#include "autoware/lidar_centerpoint/preprocess/pointcloud_densification.hpp"
#include "autoware/lidar_centerpoint/ros_utils.hpp"
#include "autoware/lidar_centerpoint/utils.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl_ros/transforms.hpp>

#include <memory>
#include <string>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace autoware::lidar_centerpoint
{
LidarCenterPointNode::LidarCenterPointNode(const rclcpp::NodeOptions & node_options)
: Node("lidar_center_point", node_options), tf_buffer_(this->get_clock())
{
  const std::vector<double> score_thresholds_double =
    this->declare_parameter<std::vector<double>>("post_process_params.score_thresholds");
  const std::vector<float> score_thresholds(
    score_thresholds_double.begin(), score_thresholds_double.end());
  const float circle_nms_dist_threshold = static_cast<float>(
    this->declare_parameter<double>("post_process_params.circle_nms_dist_threshold"));
  const auto yaw_norm_thresholds =
    this->declare_parameter<std::vector<double>>("post_process_params.yaw_norm_thresholds");
  const std::string densification_world_frame_id =
    this->declare_parameter<std::string>("densification_params.world_frame_id");
  const int densification_num_past_frames =
    this->declare_parameter<int>("densification_params.num_past_frames");
  const std::string trt_precision = this->declare_parameter<std::string>("trt_precision");
  const std::size_t cloud_capacity = this->declare_parameter<std::int64_t>("cloud_capacity");
  const std::string encoder_onnx_path = this->declare_parameter<std::string>("encoder_onnx_path");
  const std::string encoder_engine_path =
    this->declare_parameter<std::string>("encoder_engine_path");
  const std::string head_onnx_path = this->declare_parameter<std::string>("head_onnx_path");
  const std::string head_engine_path = this->declare_parameter<std::string>("head_engine_path");
  class_names_ = this->declare_parameter<std::vector<std::string>>("model_params.class_names");
  has_twist_ = this->declare_parameter<bool>("model_params.has_twist");
  const std::size_t point_feature_size = static_cast<std::size_t>(
    this->declare_parameter<std::int64_t>("model_params.point_feature_size"));
  has_variance_ = this->declare_parameter<bool>("model_params.has_variance");
  const std::size_t max_voxel_size =
    static_cast<std::size_t>(this->declare_parameter<std::int64_t>("model_params.max_voxel_size"));
  const auto point_cloud_range =
    this->declare_parameter<std::vector<double>>("model_params.point_cloud_range");
  const auto voxel_size = this->declare_parameter<std::vector<double>>("model_params.voxel_size");
  const std::size_t downsample_factor = static_cast<std::size_t>(
    this->declare_parameter<std::int64_t>("model_params.downsample_factor"));
  const std::size_t encoder_in_feature_size = static_cast<std::size_t>(
    this->declare_parameter<std::int64_t>("model_params.encoder_in_feature_size"));
  const auto allow_remapping_by_area_matrix =
    this->declare_parameter<std::vector<int64_t>>("allow_remapping_by_area_matrix");
  const auto min_area_matrix = this->declare_parameter<std::vector<double>>("min_area_matrix");
  const auto max_area_matrix = this->declare_parameter<std::vector<double>>("max_area_matrix");

  // Set up logger name
  this->logger_name_ = this->declare_parameter<std::string>("logger_name", "lidar_centerpoint");

  detection_class_remapper_.setParameters(
    allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

  {
    NMSParams p;
    p.search_distance_2d_ =
      this->declare_parameter<double>("post_process_params.iou_nms_search_distance_2d");
    p.iou_threshold_ = this->declare_parameter<double>("post_process_params.iou_nms_threshold");
    iou_bev_nms_.setParameters(p);
  }

  TrtCommonConfig encoder_param(encoder_onnx_path, trt_precision, encoder_engine_path);
  TrtCommonConfig head_param(head_onnx_path, trt_precision, head_engine_path);
  DensificationParam densification_param(
    densification_world_frame_id, densification_num_past_frames, this->logger_name_);

  if (point_cloud_range.size() != 6) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger(this->logger_name_.c_str()),
      "The size of point_cloud_range != 6: use the default parameters.");
  }
  if (voxel_size.size() != 3) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger(this->logger_name_.c_str()),
      "The size of voxel_size != 3: use the default parameters.");
  }
  CenterPointConfig config(
    class_names_.size(), point_feature_size, cloud_capacity, max_voxel_size, point_cloud_range,
    voxel_size, downsample_factor, encoder_in_feature_size, score_thresholds,
    circle_nms_dist_threshold, yaw_norm_thresholds, has_variance_, this->logger_name_);
  detector_ptr_ =
    std::make_unique<CenterPointTRT>(encoder_param, head_param, densification_param, config);
  diagnostics_centerpoint_trt_ =
    std::make_unique<autoware_utils::DiagnosticsInterface>(this, "centerpoint_trt");

  // diagnostics parameters
  max_allowed_processing_time_ms_ =
    declare_parameter<double>("diagnostics.max_allowed_processing_time_ms");
  max_acceptable_consecutive_delay_ms_ =
    declare_parameter<double>("diagnostics.max_acceptable_consecutive_delay_ms");

  pointcloud_sub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(&LidarCenterPointNode::pointCloudCallback, this, std::placeholders::_1));
  objects_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "~/output/objects", rclcpp::QoS{1});

  // initialize debug tool
  {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, this->logger_name_.c_str());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  if (stop_watch_ptr_) {
    // processing time diagnostics
    const double validation_callback_interval_ms =
      declare_parameter<double>("diagnostics.validation_callback_interval_ms");

    diagnostic_processing_time_updater_.setHardwareID(this->get_name());
    diagnostic_processing_time_updater_.add(
      "processing_time_status", this, &LidarCenterPointNode::diagnoseProcessingTime);
    // msec -> sec
    diagnostic_processing_time_updater_.setPeriod(validation_callback_interval_ms / 1e3);
  }

  if (this->declare_parameter("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine is built and shutdown node.");
    rclcpp::shutdown();
  }
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

void LidarCenterPointNode::pointCloudCallback(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & input_pointcloud_msg)
{
  const auto objects_sub_count =
    objects_pub_->get_subscription_count() + objects_pub_->get_intra_process_subscription_count();
  if (objects_sub_count < 1) {
    return;
  }

  if (stop_watch_ptr_) {
    stop_watch_ptr_->toc("processing_time", true);
  }
  diagnostics_centerpoint_trt_->clear();

  std::vector<Box3D> det_boxes3d;
  bool is_num_pillars_within_range = true;
  bool is_success = detector_ptr_->detect(
    input_pointcloud_msg, tf_buffer_, det_boxes3d, is_num_pillars_within_range);
  if (!is_success) {
    return;
  }
  diagnostics_centerpoint_trt_->add_key_value(
    "is_num_pillars_within_range", is_num_pillars_within_range);
  if (!is_num_pillars_within_range) {
    std::stringstream message;
    message << "CenterPointTRT::detect: The actual number of pillars exceeds its maximum value, "
            << "which may limit the detection performance.";
    diagnostics_centerpoint_trt_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }

  std::vector<autoware_perception_msgs::msg::DetectedObject> raw_objects;
  raw_objects.reserve(det_boxes3d.size());
  for (const auto & box3d : det_boxes3d) {
    autoware_perception_msgs::msg::DetectedObject obj;
    box3DToDetectedObject(box3d, class_names_, has_twist_, has_variance_, obj);
    raw_objects.emplace_back(obj);
  }

  autoware_perception_msgs::msg::DetectedObjects output_msg;
  output_msg.header = input_pointcloud_msg->header;
  output_msg.objects = iou_bev_nms_.apply(raw_objects);

  detection_class_remapper_.mapClasses(output_msg);

  if (objects_sub_count > 0) {
    objects_pub_->publish(output_msg);
    published_time_publisher_->publish_if_subscribed(objects_pub_, output_msg.header.stamp);
  }
  diagnostics_centerpoint_trt_->publish(input_pointcloud_msg->header.stamp);

  // add processing time for debug
  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - output_msg.header.stamp).nanoseconds()))
        .count();
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);

    last_processing_time_ms_ = processing_time_ms;
  }
}

// Check the processing time and delayed timestamp
// If the node is consistently delayed, publish an error diagnostic message
void LidarCenterPointNode::diagnoseProcessingTime(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
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

}  // namespace autoware::lidar_centerpoint

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lidar_centerpoint::LidarCenterPointNode)
