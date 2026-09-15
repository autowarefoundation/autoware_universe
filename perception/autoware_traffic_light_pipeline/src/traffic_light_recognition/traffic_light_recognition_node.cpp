// Copyright 2026 TIER IV, Inc.
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

#include "traffic_light_recognition_node.hpp"

#include <algorithm>
#include <exception>
#include <memory>
#include <string>
#include <vector>

namespace autoware::traffic_light
{
namespace
{

std::string resolve_artifact(
  rclcpp::Node * node, const std::string & ml_model_path, const std::string & name)
{
  const auto relative_path = node->declare_parameter<std::string>(name);
  return ml_model_path.empty() ? relative_path : ml_model_path + "/" + relative_path;
}

std::vector<float> declare_normalization(rclcpp::Node * node, const std::string & name)
{
  const auto values = node->declare_parameter<std::vector<double>>(name);
  return std::vector<float>(values.begin(), values.end());
}

// `prefix` is "car_classifier" or "pedestrian_classifier".
ClassifierModelConfig declare_classifier_config(
  rclcpp::Node * node, const std::string & ml_model_path, const std::string & prefix)
{
  ClassifierModelConfig classifier_config;
  classifier_config.model_path = resolve_artifact(node, ml_model_path, prefix + ".model_path");
  classifier_config.label_path = resolve_artifact(node, ml_model_path, prefix + ".label_path");
  classifier_config.precision = node->declare_parameter<std::string>(prefix + ".precision");
  classifier_config.mean = declare_normalization(node, prefix + ".mean");
  classifier_config.std = declare_normalization(node, prefix + ".std");
  return classifier_config;
}

TrafficLightRecognitionConfig declare_recognition_config(rclcpp::Node * node)
{
  TrafficLightRecognitionConfig config;

  const auto ml_model_path = node->declare_parameter<std::string>("ml_model_path", "");

  config.whole_image_detector_model_path =
    resolve_artifact(node, ml_model_path, "whole_image_detector.model_path");
  config.whole_image_detector_label_path =
    resolve_artifact(node, ml_model_path, "whole_image_detector.label_path");
  // Not an ML artifact: it ships in autoware_tensorrt_yolox's share directory, so the launch file
  // passes it as an absolute path rather than relative to ml_model_path. Declared with no default
  // because an empty remap leaves every detector label unmapped, which makes TrtYoloXDetector
  // discard every detection -- a silent total failure rather than a degraded one.
  config.whole_image_detector_roi_remap_path =
    node->declare_parameter<std::string>("whole_image_detector.roi_remap_path");
  config.whole_image_detector_score_threshold =
    static_cast<float>(node->declare_parameter<double>("whole_image_detector.score_threshold"));
  config.whole_image_detector_nms_threshold =
    static_cast<float>(node->declare_parameter<double>("whole_image_detector.nms_threshold"));
  config.whole_image_detector_precision =
    node->declare_parameter<std::string>("whole_image_detector.precision");

  config.min_timestamp_offset =
    node->declare_parameter<double>("map_based_detector.min_timestamp_offset");
  config.max_timestamp_offset =
    node->declare_parameter<double>("map_based_detector.max_timestamp_offset");

  config.car_classifier = declare_classifier_config(node, ml_model_path, "car_classifier");
  config.pedestrian_classifier =
    declare_classifier_config(node, ml_model_path, "pedestrian_classifier");

  config.over_exposure_threshold =
    node->declare_parameter<double>("classifier.over_exposure_threshold");
  config.under_exposure_threshold =
    node->declare_parameter<double>("classifier.under_exposure_threshold");

  config.diagnostics_node_name = node->get_name();

  return config;
}
}  // namespace

TrafficLightRecognitionNode::TrafficLightRecognitionNode(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_recognition", node_options),
  config_(declare_recognition_config(this)),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  if (declare_parameter<bool>("build_only")) {
    build_engines_and_shutdown();
    return;
  }

  // Load the TensorRT engines once here: the vector map is fed later through set_map(), which
  // rebuilds only the map based detector.
  try {
    recognition_ = std::make_unique<TrafficLightRecognition>(config_, tf_buffer_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "failed to initialize traffic light recognition: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  // Subscribers -------------------------------------------------------------------------------
  // transient_local is rejected by intra-process communication, which a composable node enables.
  rclcpp::SubscriptionOptions transient_local_options;
  transient_local_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  vector_map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightRecognitionNode::vector_map_callback, this, std::placeholders::_1),
    transient_local_options);
  route_sub_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightRecognitionNode::route_callback, this, std::placeholders::_1),
    transient_local_options);
  image_sub_.subscribe(this, "~/input/image", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  camera_info_sub_.subscribe(
    this, "~/input/camera_info", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sync_ = std::make_unique<Sync>(SyncPolicy(10), image_sub_, camera_info_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
  sync_->registerCallback(
    std::bind(
      &TrafficLightRecognitionNode::sync_callback, this, std::placeholders::_1,
      std::placeholders::_2));

  // Publishers --------------------------------------------------------------------------------
  signals_pub_ = create_publisher<tier4_perception_msgs::msg::TrafficLightArray>(
    "~/output/traffic_signals", rclcpp::QoS{1});
  rois_pub_ = create_publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>(
    "~/output/rois", rclcpp::QoS{1});
  diagnostics_pub_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS{10});
}

void TrafficLightRecognitionNode::build_engines_and_shutdown()
{
  RCLCPP_INFO(get_logger(), "build_only: building TensorRT engines and exiting.");
  try {
    build_engines(config_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "failed to build TensorRT engines: %s", e.what());
  }
  rclcpp::shutdown();
}

void TrafficLightRecognitionNode::vector_map_callback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  recognition_->set_map(*msg);
}

void TrafficLightRecognitionNode::route_callback(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg)
{
  const auto result = recognition_->set_route(*msg);
  if (!result) {
    RCLCPP_ERROR(get_logger(), "%s", result.error().c_str());
  }
}

void TrafficLightRecognitionNode::sync_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  // Wait for the transform at the exact moment to become available.
  const rclcpp::Time latest_required_stamp =
    rclcpp::Time(camera_info_msg->header.stamp) +
    rclcpp::Duration::from_seconds(std::max(0.0, config_.max_timestamp_offset));
  std::string tf_error_msg;
  if (!tf_buffer_.canTransform(
        "map", camera_info_msg->header.frame_id, latest_required_stamp,
        rclcpp::Duration::from_seconds(0.2), &tf_error_msg)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "failed to get transform from map frame to camera frame: %s", tf_error_msg.c_str());
    return;
  }

  const auto result = recognition_->run(*image_msg, *camera_info_msg);
  if (!result) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "run() failed: %s", result.error().c_str());
    return;
  }

  signals_pub_->publish(result->merged_signals);
  rois_pub_->publish(result->selected_rois);
  diagnostics_pub_->publish(result->diagnostics);
}

}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightRecognitionNode)
