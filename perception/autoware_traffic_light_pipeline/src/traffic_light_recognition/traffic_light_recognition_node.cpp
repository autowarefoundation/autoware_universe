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

#include <exception>
#include <memory>
#include <string>

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

  config.min_timestamp_offset =
    node->declare_parameter<double>("map_based_detector.min_timestamp_offset");
  config.max_timestamp_offset =
    node->declare_parameter<double>("map_based_detector.max_timestamp_offset");

  config.car_classifier_model_path =
    resolve_artifact(node, ml_model_path, "car_classifier.model_path");
  config.car_classifier_label_path =
    resolve_artifact(node, ml_model_path, "car_classifier.label_path");

  config.pedestrian_classifier_model_path =
    resolve_artifact(node, ml_model_path, "pedestrian_classifier.model_path");
  config.pedestrian_classifier_label_path =
    resolve_artifact(node, ml_model_path, "pedestrian_classifier.label_path");

  config.over_exposure_threshold =
    node->declare_parameter<double>("classifier.over_exposure_threshold");
  config.under_exposure_threshold =
    node->declare_parameter<double>("classifier.under_exposure_threshold");

  return config;
}
}  // namespace

TrafficLightRecognitionNode::TrafficLightRecognitionNode(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_recognition", node_options),
  config_(declare_recognition_config(this)),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Subscribers -------------------------------------------------------------------------------
  vector_map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightRecognitionNode::vector_map_callback, this, std::placeholders::_1));
  route_sub_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightRecognitionNode::route_callback, this, std::placeholders::_1));
  image_sub_.subscribe(this, "~/input/image", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  camera_info_sub_.subscribe(
    this, "~/input/camera_info", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  sync_ = std::make_unique<Sync>(SyncPolicy(10), image_sub_, camera_info_sub_);
  sync_->registerCallback(
    std::bind(
      &TrafficLightRecognitionNode::sync_callback, this, std::placeholders::_1,
      std::placeholders::_2));

  // Publishers --------------------------------------------------------------------------------
  signals_pub_ = create_publisher<tier4_perception_msgs::msg::TrafficLightArray>(
    "~/output/traffic_signals", rclcpp::QoS{1});
  rois_pub_ = create_publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>(
    "~/output/rois", rclcpp::QoS{1});

  if (declare_parameter<bool>("build_only")) {
    build_engines_and_shutdown();
  }
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
  recognition_ = std::make_unique<TrafficLightRecognition>(config_, *msg, tf_buffer_);
}

void TrafficLightRecognitionNode::route_callback(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg)
{
  if (!recognition_) {
    RCLCPP_ERROR(get_logger(), "vector map not received yet: dropping route");
    return;
  }
  const auto error = recognition_->set_route(*msg);
  if (error) {
    RCLCPP_ERROR(get_logger(), "%s", error->message.c_str());
  }
}

void TrafficLightRecognitionNode::sync_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  if (!recognition_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "vector map not received yet: dropping frame");
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
}

}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightRecognitionNode)
