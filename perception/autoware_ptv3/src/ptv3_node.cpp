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

#include "autoware/ptv3/ptv3_node.hpp"

#include "autoware/ptv3/ros_utils.hpp"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::ptv3
{

PTv3Node::PTv3Node(const rclcpp::NodeOptions & options) : Node("ptv3", options)
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor{}.set__read_only(true);

  auto to_float_vector = [](const auto & v) -> std::vector<float> {
    return std::vector<float>(v.begin(), v.end());
  };

  // TensorRT parameters
  const std::string plugins_path = this->declare_parameter<std::string>("plugins_path", descriptor);
  const std::string trt_precision =
    this->declare_parameter<std::string>("trt_precision", descriptor);
  const auto cloud_capacity = this->declare_parameter<std::int64_t>("cloud_capacity", descriptor);

  // Backbone parameters
  const std::string backbone_onnx_path =
    this->declare_parameter<std::string>("backbone.onnx_path", descriptor);
  const std::string backbone_engine_path =
    this->declare_parameter<std::string>("backbone.engine_path", descriptor);
  const auto voxels_num =
    this->declare_parameter<std::vector<std::int64_t>>("backbone.voxels_num", descriptor);
  const auto point_cloud_range = to_float_vector(
    this->declare_parameter<std::vector<double>>("backbone.point_cloud_range", descriptor));
  const auto voxel_size = to_float_vector(
    this->declare_parameter<std::vector<double>>("backbone.voxel_size", descriptor));
  const auto serialization_orders =
    this->declare_parameter<std::vector<std::string>>("backbone.serialization_orders", descriptor);
  const auto pooling_strides =
    this->declare_parameter<std::vector<std::int64_t>>("backbone.pooling_strides", descriptor);

  if (point_cloud_range.size() != 6) {
    throw std::runtime_error("The size of point_cloud_range != 6");
  }
  if (voxel_size.size() != 3) {
    throw std::runtime_error("The size of voxel_size != 3");
  }

  // Segmentation head parameters
  const bool use_seg3d_head = this->declare_parameter<bool>("segmentation3d.use_head", descriptor);
  std::optional<tensorrt_common::TrtCommonConfig> seg3d_head_trt_config;
  std::vector<std::string> segmentation_class_names;
  std::vector<std::int64_t> palette;
  float filter_class_probability_threshold = 0.0F;
  std::vector<std::string> filter_classes;
  std::string filter_output_format;
  std::string source_reconstruction = "none";
  if (use_seg3d_head) {
    const std::string seg3d_head_onnx_path =
      this->declare_parameter<std::string>("segmentation3d.onnx_path", descriptor);
    const std::string seg3d_head_engine_path =
      this->declare_parameter<std::string>("segmentation3d.engine_path", descriptor);
    segmentation_class_names =
      this->declare_parameter<std::vector<std::string>>("segmentation3d.class_names", descriptor);
    palette =
      this->declare_parameter<std::vector<std::int64_t>>("segmentation3d.palette", descriptor);
    filter_class_probability_threshold = static_cast<float>(this->declare_parameter<double>(
      "segmentation3d.filter.class_probability_threshold", descriptor));
    filter_classes = this->declare_parameter<std::vector<std::string>>(
      "segmentation3d.filter.classes", descriptor);
    filter_output_format =
      this->declare_parameter<std::string>("segmentation3d.filter.output_format", descriptor);
    source_reconstruction =
      this->declare_parameter<std::string>("segmentation3d.source_reconstruction", descriptor);
    seg3d_head_trt_config.emplace(
      seg3d_head_onnx_path, trt_precision, seg3d_head_engine_path, 1ULL << 30U);
  }

  // Detection head parameters
  const bool use_det3d_head = this->declare_parameter<bool>("detection3d.use_head", descriptor);
  std::optional<tensorrt_common::TrtCommonConfig> det3d_head_trt_config;
  std::vector<float> bbox_voxel_size;
  std::vector<float> distance_bin_upper_limits;
  std::vector<float> detection_score_thresholds;
  std::vector<float> yaw_norm_thresholds;
  perception_utils::IouBevNmsParams nms_params;
  std::size_t num_proposals{};
  std::vector<float> post_center_range;
  if (use_det3d_head) {
    const std::string det3d_head_onnx_path =
      this->declare_parameter<std::string>("detection3d.onnx_path", descriptor);
    const std::string det3d_head_engine_path =
      this->declare_parameter<std::string>("detection3d.engine_path", descriptor);

    detection_class_names_ =
      this->declare_parameter<std::vector<std::string>>("detection3d.class_names", descriptor);
    bbox_voxel_size = to_float_vector(
      this->declare_parameter<std::vector<double>>("detection3d.bbox_voxel_size", descriptor));
    has_twist_ = this->declare_parameter<bool>("detection3d.has_twist", descriptor);

    const auto allow_remapping_by_area_matrix = this->declare_parameter<std::vector<std::int64_t>>(
      "detection3d.allow_remapping_by_area_matrix", descriptor);
    const auto min_area_matrix =
      this->declare_parameter<std::vector<double>>("detection3d.min_area_matrix", descriptor);
    const auto max_area_matrix =
      this->declare_parameter<std::vector<double>>("detection3d.max_area_matrix", descriptor);
    detection_class_remapper_.setParameters(
      allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

    distance_bin_upper_limits = to_float_vector(this->declare_parameter<std::vector<double>>(
      "detection3d.detection_score_thresholds.distance_bin_upper_limits", descriptor));
    if (distance_bin_upper_limits.empty()) {
      throw std::runtime_error(
        "detection3d.detection_score_thresholds.distance_bin_upper_limits must not be empty.");
    }

    detection_score_thresholds.assign(
      detection_class_names_.size() * distance_bin_upper_limits.size(), 0.0F);
    for (std::size_t class_index = 0; class_index < detection_class_names_.size(); ++class_index) {
      const std::string param_name =
        "detection3d.detection_score_thresholds.min_confidence_scores." +
        detection_class_names_[class_index];
      const auto class_thresholds =
        to_float_vector(this->declare_parameter<std::vector<double>>(param_name, descriptor));
      if (class_thresholds.size() != distance_bin_upper_limits.size()) {
        throw std::runtime_error(
          "The number of confidence thresholds must match distance_bin_upper_limits for " +
          detection_class_names_[class_index] + ".");
      }
      for (std::size_t distance_index = 0; distance_index < class_thresholds.size();
           ++distance_index) {
        detection_score_thresholds[distance_index * detection_class_names_.size() + class_index] =
          class_thresholds[distance_index];
      }
    }

    yaw_norm_thresholds = to_float_vector(this->declare_parameter<std::vector<double>>(
      "detection3d.post_process_params.yaw_norm_thresholds", descriptor));

    nms_params.search_distance_2d = this->declare_parameter<double>(
      "detection3d.post_process_params.iou_nms_search_distance_2d", descriptor);
    nms_params.iou_threshold = this->declare_parameter<double>(
      "detection3d.post_process_params.iou_nms_threshold", descriptor);

    const auto num_proposals_param =
      this->declare_parameter<std::int64_t>("detection3d.num_proposals", descriptor);
    if (num_proposals_param <= 0) {
      throw std::runtime_error("detection3d.num_proposals must be positive.");
    }
    num_proposals = static_cast<std::size_t>(num_proposals_param);
    post_center_range = to_float_vector(
      this->declare_parameter<std::vector<double>>("detection3d.post_center_range", descriptor));

    det3d_head_trt_config.emplace(
      det3d_head_onnx_path, trt_precision, det3d_head_engine_path, 1ULL << 32U);
  }

  PTv3Config config(
    use_seg3d_head, use_det3d_head, plugins_path, cloud_capacity, voxels_num, point_cloud_range,
    voxel_size, segmentation_class_names, serialization_orders, pooling_strides, palette,
    filter_class_probability_threshold, filter_classes, filter_output_format, source_reconstruction,
    detection_class_names_, bbox_voxel_size, distance_bin_upper_limits, detection_score_thresholds,
    yaw_norm_thresholds, has_twist_, num_proposals, post_center_range);

  const auto backbone_trt_config = tensorrt_common::TrtCommonConfig(
    backbone_onnx_path, trt_precision, backbone_engine_path, 1ULL << 33U);

  model_ptr_ = std::make_unique<PTv3TRT>(
    backbone_trt_config, seg3d_head_trt_config, det3d_head_trt_config, config);

  pointcloud_sub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(&PTv3Node::cloudCallback, this, std::placeholders::_1));

  if (use_seg3d_head) {
    segmented_pointcloud_pub_ =
      std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
        *this, "~/output/pointcloud/segmentation");
    visualization_pointcloud_pub_ =
      std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
        *this, "~/output/pointcloud/visualization");
    filtered_pointcloud_pub_ =
      std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
        *this, "~/output/pointcloud/filtered");

    model_ptr_->setPublishSegmentedPointcloud(
      std::bind(&PTv3Node::publishSegmentedPointcloud, this, std::placeholders::_1));
    model_ptr_->setPublishVisualizationPointcloud(
      std::bind(&PTv3Node::publishVisualizationPointcloud, this, std::placeholders::_1));
    model_ptr_->setPublishFilteredPointcloud(
      std::bind(&PTv3Node::publishFilteredPointcloud, this, std::placeholders::_1));
  }

  if (use_det3d_head) {
    detected_objects_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
      "~/output/objects", rclcpp::QoS{1});
    iou_bev_nms_.setParameters(nms_params);
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

void PTv3Node::publishSegmentedPointcloud(
  std::unique_ptr<const cuda_blackboard::CudaPointCloud2> msg_ptr)
{
  if (segmented_pointcloud_pub_) {
    segmented_pointcloud_pub_->publish(std::move(msg_ptr));
  }
}

void PTv3Node::publishVisualizationPointcloud(
  std::unique_ptr<const cuda_blackboard::CudaPointCloud2> msg_ptr)
{
  if (visualization_pointcloud_pub_) {
    visualization_pointcloud_pub_->publish(std::move(msg_ptr));
  }
}

void PTv3Node::publishFilteredPointcloud(
  std::unique_ptr<const cuda_blackboard::CudaPointCloud2> msg_ptr)
{
  if (filtered_pointcloud_pub_) {
    filtered_pointcloud_pub_->publish(std::move(msg_ptr));
  }
}

void PTv3Node::cloudCallback(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr)
{
  const auto segmented_sub_count =
    segmented_pointcloud_pub_ ? (segmented_pointcloud_pub_->get_subscription_count() +
                                 segmented_pointcloud_pub_->get_intra_process_subscription_count())
                              : 0u;
  const auto visualization_sub_count =
    visualization_pointcloud_pub_
      ? (visualization_pointcloud_pub_->get_subscription_count() +
         visualization_pointcloud_pub_->get_intra_process_subscription_count())
      : 0u;
  const auto filtered_sub_count =
    filtered_pointcloud_pub_ ? (filtered_pointcloud_pub_->get_subscription_count() +
                                filtered_pointcloud_pub_->get_intra_process_subscription_count())
                             : 0u;
  const auto objects_sub_count = detected_objects_pub_
                                   ? (detected_objects_pub_->get_subscription_count() +
                                      detected_objects_pub_->get_intra_process_subscription_count())
                                   : 0u;

  if (segmented_sub_count + visualization_sub_count + filtered_sub_count + objects_sub_count == 0) {
    return;
  }

  if (stop_watch_ptr_) {
    stop_watch_ptr_->toc("processing/total", true);
  }

  std::unordered_map<std::string, double> proc_timing;
  std::optional<std::vector<Box3D>> det_boxes3d;
  bool is_success = model_ptr_->infer(
    msg_ptr, segmented_sub_count, visualization_sub_count, filtered_sub_count,
    objects_sub_count > 0u, det_boxes3d, proc_timing);
  if (!is_success) {
    return;
  }

  if (objects_sub_count > 0u && detected_objects_pub_ && det_boxes3d.has_value()) {
    std::vector<autoware_perception_msgs::msg::DetectedObject> raw_objects;
    raw_objects.reserve(det_boxes3d->size());
    for (const auto & box3d : *det_boxes3d) {
      autoware_perception_msgs::msg::DetectedObject object;
      box3d_to_detected_object(box3d, detection_class_names_, has_twist_, object);
      raw_objects.emplace_back(std::move(object));
    }

    autoware_perception_msgs::msg::DetectedObjects output_msg;
    output_msg.header = msg_ptr->header;
    output_msg.objects = iou_bev_nms_.apply(raw_objects);
    detection_class_remapper_.mapClasses(output_msg);
    detected_objects_pub_->publish(output_msg);
    published_time_pub_->publish_if_subscribed(detected_objects_pub_, output_msg.header.stamp);
  }

  // add processing time for debug
  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing/total", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - msg_ptr->header.stamp).nanoseconds()))
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

}  // namespace autoware::ptv3

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ptv3::PTv3Node)
