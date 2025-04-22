// Copyright 2024 TIER IV, Inc.
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

#include "autoware/lidar_transfusion/lidar_transfusion_node.hpp"

#include "autoware/lidar_transfusion/utils.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::lidar_transfusion
{

LidarTransfusionNode::LidarTransfusionNode(const rclcpp::NodeOptions & options)
: Node("lidar_transfusion", options), tf_buffer_(this->get_clock())
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor{}.set__read_only(true);
  // network
  class_names_ = this->declare_parameter<std::vector<std::string>>("class_names", descriptor);
  const std::string trt_precision =
    this->declare_parameter<std::string>("trt_precision", descriptor);
  const std::size_t cloud_capacity =
    this->declare_parameter<std::int64_t>("cloud_capacity", descriptor);
  const auto voxels_num = this->declare_parameter<std::vector<int64_t>>("voxels_num", descriptor);
  const auto point_cloud_range =
    this->declare_parameter<std::vector<double>>("point_cloud_range", descriptor);
  const auto voxel_size = this->declare_parameter<std::vector<double>>("voxel_size", descriptor);
  const std::size_t num_proposals =
    this->declare_parameter<std::int64_t>("num_proposals", descriptor);
  const std::string onnx_path = this->declare_parameter<std::string>("onnx_path", descriptor);
  const std::string engine_path = this->declare_parameter<std::string>("engine_path", descriptor);

  if (point_cloud_range.size() != 6) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("lidar_transfusion"),
      "The size of point_cloud_range != 6: use the default parameters.");
  }
  if (voxel_size.size() != 3) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("lidar_transfusion"),
      "The size of voxel_size != 3: use the default parameters.");
  }

  // pre-process
  const std::string densification_world_frame_id =
    this->declare_parameter<std::string>("densification_world_frame_id", descriptor);
  const int densification_num_past_frames =
    this->declare_parameter<int64_t>("densification_num_past_frames", descriptor);

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
  TransfusionConfig config(
    cloud_capacity, voxels_num, point_cloud_range, voxel_size, num_proposals,
    circle_nms_dist_threshold, yaw_norm_thresholds, score_threshold);

  const auto allow_remapping_by_area_matrix =
    this->declare_parameter<std::vector<int64_t>>("allow_remapping_by_area_matrix", descriptor);
  const auto min_area_matrix =
    this->declare_parameter<std::vector<double>>("min_area_matrix", descriptor);
  const auto max_area_matrix =
    this->declare_parameter<std::vector<double>>("max_area_matrix", descriptor);
  detection_class_remapper_.setParameters(
    allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

  // diagnostics parameters
  max_allowed_processing_time_ms_ =
    declare_parameter<double>("diagnostics.max_allowed_processing_time_ms");
  max_acceptable_consecutive_delay_ms_ =
    declare_parameter<double>("diagnostics.max_acceptable_consecutive_delay_ms");

  auto trt_config = tensorrt_common::TrtCommonConfig(onnx_path, trt_precision, engine_path);

  detector_ptr_ = std::make_unique<TransfusionTRT>(trt_config, densification_param, config);

  cloud_sub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(&LidarTransfusionNode::cloudCallback, this, std::placeholders::_1));

  objects_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "~/output/objects", rclcpp::QoS(1));

  published_time_pub_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);

  // setup diagnostics
  {
    const double validation_callback_interval_ms =
      declare_parameter<double>("diagnostics.validation_callback_interval_ms");

    diagnostic_processing_time_updater_.setHardwareID(this->get_name());
    diagnostic_processing_time_updater_.add(
      "processing_time_status", this, &LidarTransfusionNode::diagnoseProcessingTime);
    // msec -> sec
    diagnostic_processing_time_updater_.setPeriod(validation_callback_interval_ms / 1e3);
  }

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
    RCLCPP_INFO(this->get_logger(), "TensorRT engine is built and shutdown node.");
    rclcpp::shutdown();
  }
}

void LidarTransfusionNode::cloudCallback(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg)
{
  const auto objects_sub_count =
    objects_pub_->get_subscription_count() + objects_pub_->get_intra_process_subscription_count();
  if (objects_sub_count < 1) {
    return;
  }

  if (stop_watch_ptr_) {
    stop_watch_ptr_->toc("processing/total", true);
  }

  std::vector<Box3D> det_boxes3d;
  std::unordered_map<std::string, double> proc_timing;
  bool is_success = detector_ptr_->detect(msg, tf_buffer_, det_boxes3d, proc_timing);
  if (!is_success) {
    return;
  }

  std::vector<autoware_perception_msgs::msg::DetectedObject> raw_objects;
  raw_objects.reserve(det_boxes3d.size());
  for (const auto & box3d : det_boxes3d) {
    autoware_perception_msgs::msg::DetectedObject obj;
    box3DToDetectedObject(box3d, class_names_, obj);
    raw_objects.emplace_back(obj);
  }

  autoware_perception_msgs::msg::DetectedObjects output_msg;
  output_msg.header = msg->header;
  output_msg.objects = iou_bev_nms_.apply(raw_objects);

  detection_class_remapper_.mapClasses(output_msg);

  if (objects_sub_count > 0) {
    objects_pub_->publish(output_msg);
    published_time_pub_->publish_if_subscribed(objects_pub_, output_msg.header.stamp);
  }
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

    last_processing_time_ms_ = processing_time_ms;
  }
}

void LidarTransfusionNode::diagnoseProcessingTime(
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
    message << "Waiting for the node to perform inference.";
  }

  stat.summary(diag_level, message.str());
}

}  // namespace autoware::lidar_transfusion

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lidar_transfusion::LidarTransfusionNode)
