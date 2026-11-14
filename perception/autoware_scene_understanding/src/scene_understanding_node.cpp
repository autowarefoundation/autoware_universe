// Copyright 2025 TIER IV, Inc.
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

#include "scene_understanding_tensorrt/scene_understanding_node.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace scene_understanding_tensorrt
{

SceneUnderstandingNode::SceneUnderstandingNode(const rclcpp::NodeOptions & options)
: Node("scene_understanding_node", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("onnx_model_path", "scene_understanding_model.onnx");
  this->declare_parameter<std::string>("input_image_topic", "/image_raw");
  this->declare_parameter<std::string>("output_topic", "/scene_understanding");
  this->declare_parameter<bool>("use_fp16", true);
  this->declare_parameter<bool>("use_int8", false);
  this->declare_parameter<int>("max_batch_size", 1);

  // Get parameters
  onnx_model_path_ = this->get_parameter("onnx_model_path").as_string();
  auto input_topic = this->get_parameter("input_image_topic").as_string();
  auto output_topic = this->get_parameter("output_topic").as_string();
  use_fp16_ = this->get_parameter("use_fp16").as_bool();
  use_int8_ = this->get_parameter("use_int8").as_bool();
  max_batch_size_ = this->get_parameter("max_batch_size").as_int();

  RCLCPP_INFO(this->get_logger(), "Initializing Scene Understanding Node");
  RCLCPP_INFO(this->get_logger(), "ONNX Model Path: %s", onnx_model_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Input Topic: %s", input_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Output Topic: %s", output_topic.c_str());

  // Initialize TensorRT engine
  tensorrt_engine_ = std::make_unique<TensorRTEngine>();
  if (!tensorrt_engine_->initialize(onnx_model_path_, max_batch_size_, use_fp16_, use_int8_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize TensorRT engine");
    throw std::runtime_error("TensorRT initialization failed");
  }

  // Create subscriber and publisher
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_topic, rclcpp::SensorDataQoS(),
    std::bind(&SceneUnderstandingNode::imageCallback, this, std::placeholders::_1));

  result_pub_ = this->create_publisher<msg::SceneUnderstanding>(output_topic, rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "Scene Understanding Node initialized successfully");
}

void SceneUnderstandingNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Convert ROS image to OpenCV
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Run inference
  SceneUnderstandingResult result;
  auto start = std::chrono::high_resolution_clock::now();

  if (!tensorrt_engine_->infer(cv_ptr->image, result)) {
    RCLCPP_ERROR(this->get_logger(), "Inference failed");
    return;
  }

  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

  RCLCPP_DEBUG(this->get_logger(), "Total processing time: %ld ms", duration.count());

  // Convert result to ROS message and publish
  auto result_msg = resultToMessage(result, msg->header);
  result_pub_->publish(result_msg);
}

msg::SceneUnderstanding SceneUnderstandingNode::resultToMessage(
  const SceneUnderstandingResult & result, const std_msgs::msg::Header & header)
{
  msg::SceneUnderstanding msg;
  msg.header = header;

  // Convert exclusive categories
  msg.intersection = TensorRTEngine::intersectionToString(result.intersection);
  msg.lane_position = TensorRTEngine::lanePositionToString(result.lane_position);
  msg.lighting = TensorRTEngine::lightingToString(result.lighting);
  msg.road_geometry = TensorRTEngine::roadGeometryToString(result.road_geometry);
  msg.road_surface = TensorRTEngine::roadSurfaceToString(result.road_surface);
  msg.road_type = TensorRTEngine::roadTypeToString(result.road_type);
  msg.traffic_density = TensorRTEngine::trafficDensityToString(result.traffic_density);
  msg.visibility = TensorRTEngine::visibilityToString(result.visibility);
  msg.weather = TensorRTEngine::weatherToString(result.weather);

  // Copy binary categories
  msg.road_features_construction_zone = result.road_features_construction_zone;
  msg.road_features_crosswalk = result.road_features_crosswalk;
  msg.road_features_merge_area = result.road_features_merge_area;
  msg.road_features_railway_crossing = result.road_features_railway_crossing;
  msg.special_vehicles_emergency_vehicle = result.special_vehicles_emergency_vehicle;
  msg.special_vehicles_heavy_vehicle = result.special_vehicles_heavy_vehicle;
  msg.special_vehicles_public_transport = result.special_vehicles_public_transport;
  msg.traffic_control_no_traffic_controls = result.traffic_control_no_traffic_controls;
  msg.traffic_control_stop_sign = result.traffic_control_stop_sign;
  msg.traffic_control_traffic_light = result.traffic_control_traffic_light;
  msg.traffic_control_yield_sign = result.traffic_control_yield_sign;
  msg.vulnerable_road_users_animals = result.vulnerable_road_users_animals;
  msg.vulnerable_road_users_cyclists = result.vulnerable_road_users_cyclists;
  msg.vulnerable_road_users_pedestrians = result.vulnerable_road_users_pedestrians;

  return msg;
}

}  // namespace scene_understanding_tensorrt

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(scene_understanding_tensorrt::SceneUnderstandingNode)
