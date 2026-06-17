// Copyright 2022 TIER IV, Inc.
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

#include "autoware/tensorrt_yolox/tensorrt_yolox_node.hpp"

#include <memory>
#include <optional>
#include <string>

// cspell: ignore semseg

namespace autoware::tensorrt_yolox
{
TrtYoloXNode::TrtYoloXNode(const rclcpp::NodeOptions & node_options)
: Node("tensorrt_yolox", node_options)
{
  {
    stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<autoware_utils::DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  TrtYoloXDetectorConfig config;
  config.model_path = this->declare_parameter<std::string>("model_path");
  config.precision = this->declare_parameter<std::string>("precision");
  config.score_threshold = static_cast<float>(this->declare_parameter<double>("score_threshold"));
  config.nms_threshold = static_cast<float>(this->declare_parameter<double>("nms_threshold"));
  config.calibration_algorithm = this->declare_parameter<std::string>("calibration_algorithm");
  config.dla_core_id = this->declare_parameter<int>("dla_core_id");
  config.quantize_first_layer = this->declare_parameter<bool>("quantize_first_layer");
  config.quantize_last_layer = this->declare_parameter<bool>("quantize_last_layer");
  config.profile_per_layer = this->declare_parameter<bool>("profile_per_layer");
  config.clip_value = this->declare_parameter<double>("clip_value");
  config.preprocess_on_gpu = this->declare_parameter<bool>("preprocess_on_gpu");
  config.calibration_image_list_path =
    this->declare_parameter<std::string>("calibration_image_list_path");
  config.gpu_id = this->declare_parameter<uint8_t>("gpu_id");

  config.label_path = this->declare_parameter<std::string>("label_path");
  config.semseg_color_map_path =
    this->declare_parameter<std::string>("semantic_segmentation_color_map_path", "");

  // if the remap file path is an empty string, it will not do remap the labels
  config.roi_remap_path = this->declare_parameter<std::string>("roi_remap_path", "");
  config.roi_to_semseg_remap_path =
    this->declare_parameter<std::string>("roi_to_semantic_segmentation_remap_path", "");

  config.is_roi_overlap_semseg = declare_parameter<bool>("is_roi_overlap_segmentation");
  config.is_publish_color_mask = declare_parameter<bool>("is_publish_color_mask");
  config.overlap_roi_score_threshold = declare_parameter<float>("overlap_roi_score_threshold");
  config.roi_overlay_semseg_labels.UNKNOWN =
    declare_parameter<bool>("roi_overlay_segmentation_label.UNKNOWN");
  config.roi_overlay_semseg_labels.CAR =
    declare_parameter<bool>("roi_overlay_segmentation_label.CAR");
  config.roi_overlay_semseg_labels.TRUCK =
    declare_parameter<bool>("roi_overlay_segmentation_label.TRUCK");
  config.roi_overlay_semseg_labels.BUS =
    declare_parameter<bool>("roi_overlay_segmentation_label.BUS");
  config.roi_overlay_semseg_labels.MOTORCYCLE =
    declare_parameter<bool>("roi_overlay_segmentation_label.MOTORCYCLE");
  config.roi_overlay_semseg_labels.BICYCLE =
    declare_parameter<bool>("roi_overlay_segmentation_label.BICYCLE");
  config.roi_overlay_semseg_labels.PEDESTRIAN =
    declare_parameter<bool>("roi_overlay_segmentation_label.PEDESTRIAN");
  config.roi_overlay_semseg_labels.ANIMAL =
    declare_parameter<bool>("roi_overlay_segmentation_label.ANIMAL");
  config.roi_overlay_semseg_labels.HAZARD =
    declare_parameter<bool>("roi_overlay_segmentation_label.HAZARD");

  detector_ = std::make_unique<TrtYoloXDetector>(config);

  if (!detector_->isGPUInitialized()) {
    RCLCPP_ERROR(this->get_logger(), "GPU %d does not exist or is not suitable.", config.gpu_id);
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "GPU %d is selected for the inference!", config.gpu_id);

  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&TrtYoloXNode::onConnect, this));

  objects_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "~/out/objects", 1);
  mask_pub_ = image_transport::create_publisher(this, "~/out/mask");
  color_mask_pub_ = image_transport::create_publisher(this, "~/out/color_mask");
  image_pub_ = image_transport::create_publisher(this, "~/out/image");

  if (declare_parameter("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine file is built and exit.");
    rclcpp::shutdown();
  }
}

void TrtYoloXNode::onConnect()
{
  using std::placeholders::_1;
  if (
    objects_pub_->get_subscription_count() == 0 &&
    objects_pub_->get_intra_process_subscription_count() == 0 &&
    image_pub_.getNumSubscribers() == 0 && mask_pub_.getNumSubscribers() == 0 &&
    color_mask_pub_.getNumSubscribers() == 0) {
    image_sub_.shutdown();
  } else if (!image_sub_) {
    image_sub_ = image_transport::create_subscription(
      this, "~/in/image", std::bind(&TrtYoloXNode::onImage, this, _1), "raw",
      rmw_qos_profile_sensor_data);
  }
}

void TrtYoloXNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  stop_watch_ptr_->toc("processing_time", true);

  const auto result = detector_->detect(*msg);
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "detection failed: %s", result.error().c_str());
    return;
  }

  if (result->mask) {
    mask_pub_.publish(*result->mask);
  }

  image_pub_.publish(result->image);

  objects_pub_->publish(result->objects);

  if (debug_publisher_) {
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - result->objects.header.stamp).nanoseconds()))
        .count();
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }

  if (result->color_mask) {
    color_mask_pub_.publish(*result->color_mask);
  }
}

}  // namespace autoware::tensorrt_yolox

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_yolox::TrtYoloXNode)
