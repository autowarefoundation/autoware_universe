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

#include "scene_understanding_tensorrt/scene_visualization_node.hpp"

#include <opencv2/opencv.hpp>

#include <memory>
#include <string>
#include <vector>

namespace scene_understanding_tensorrt
{

SceneVisualizationNode::SceneVisualizationNode(const rclcpp::NodeOptions & options)
: Node("scene_visualization_node", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("input_image_topic", "/image_raw");
  this->declare_parameter<std::string>("input_scene_topic", "/scene_understanding");
  this->declare_parameter<std::string>("output_image_topic", "/scene_visualization");
  this->declare_parameter<double>("font_scale", 1.5);
  this->declare_parameter<int>("font_thickness", 3);
  this->declare_parameter<int>("line_spacing", 60);

  // Get parameters
  auto input_image_topic = this->get_parameter("input_image_topic").as_string();
  auto input_scene_topic = this->get_parameter("input_scene_topic").as_string();
  auto output_image_topic = this->get_parameter("output_image_topic").as_string();
  font_scale_ = this->get_parameter("font_scale").as_double();
  font_thickness_ = this->get_parameter("font_thickness").as_int();
  line_spacing_ = this->get_parameter("line_spacing").as_int();

  RCLCPP_INFO(this->get_logger(), "Initializing Scene Visualization Node");
  RCLCPP_INFO(this->get_logger(), "Input Image Topic: %s", input_image_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Input Scene Topic: %s", input_scene_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Output Image Topic: %s", output_image_topic.c_str());

  // Create message filter subscribers
  image_sub_ =
    std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, input_image_topic);
  scene_sub_ =
    std::make_shared<message_filters::Subscriber<SceneUnderstanding>>(this, input_scene_topic);

  // Create synchronizer
  synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), *image_sub_, *scene_sub_);

  // Register callback
  synchronizer_->registerCallback(std::bind(
    &SceneVisualizationNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Create publisher
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic, rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(), "Scene Visualization Node initialized successfully");
}

void SceneVisualizationNode::syncCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const SceneUnderstanding::ConstSharedPtr & scene_msg)
{
  // Convert ROS image to OpenCV
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Visualize scene understanding on image
  cv::Mat visualized_image = visualizeImage(cv_ptr->image, scene_msg);

  // Convert back to ROS message and publish
  sensor_msgs::msg::Image::SharedPtr output_msg =
    cv_bridge::CvImage(image_msg->header, "bgr8", visualized_image).toImageMsg();

  image_pub_->publish(*output_msg);
}

cv::Mat SceneVisualizationNode::visualizeImage(
  const cv::Mat & image, const SceneUnderstanding::ConstSharedPtr & scene_msg)
{
  // Create a copy of the image to draw on
  cv::Mat output_image = image.clone();

  // Prepare metadata to display
  std::vector<std::string> metadata_to_display = {
    "Intersections: " + scene_msg->intersection,
    "Lane Position: " + scene_msg->lane_position,
    "Road Geometry: " + scene_msg->road_geometry,
    "Road Type: " + scene_msg->road_type,
    "Traffic Density: " + scene_msg->traffic_density,
    "Visibility: " + scene_msg->visibility,
    "Crosswalk: " +
      std::to_string(std::round(scene_msg->road_features_crosswalk * 100) / 100.0).substr(0, 4),
    "Heavy Vehicles: " +
      std::to_string(std::round(scene_msg->special_vehicles_heavy_vehicle * 100) / 100.0)
        .substr(0, 4),
    "Traffic Light: " +
      std::to_string(std::round(scene_msg->traffic_control_traffic_light * 100) / 100.0)
        .substr(0, 4),
    "Pedestrians: " +
      std::to_string(std::round(scene_msg->vulnerable_road_users_pedestrians * 100) / 100.0)
        .substr(0, 4)};

  // Set font parameters
  int font = cv::FONT_HERSHEY_SIMPLEX;
  cv::Scalar font_color(255, 170, 0);  // BGR format (blue, green, red)
  int start_y = 50;

  // Add text to the image
  for (size_t i = 0; i < metadata_to_display.size(); ++i) {
    int y_position = start_y + i * line_spacing_;

    // Add background rectangle for better visibility
    cv::Size text_size =
      cv::getTextSize(metadata_to_display[i], font, font_scale_, font_thickness_, nullptr);
    cv::Rect rect(
      15, y_position - text_size.height - 5, text_size.width + 10, text_size.height + 10);
    cv::rectangle(output_image, rect, cv::Scalar(0, 0, 0), -1);

    // Add text
    cv::putText(
      output_image, metadata_to_display[i], cv::Point(20, y_position), font, font_scale_,
      font_color, font_thickness_);
  }

  return output_image;
}

}  // namespace scene_understanding_tensorrt

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(scene_understanding_tensorrt::SceneVisualizationNode)
