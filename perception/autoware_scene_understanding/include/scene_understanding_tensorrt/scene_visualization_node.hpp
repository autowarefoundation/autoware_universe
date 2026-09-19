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

#ifndef SCENE_UNDERSTANDING_TENSORRT__SCENE_VISUALIZATION_NODE_HPP_
#define SCENE_UNDERSTANDING_TENSORRT__SCENE_VISUALIZATION_NODE_HPP_

#include "scene_understanding_tensorrt/msg/scene_understanding.hpp"

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <memory>
#include <string>

namespace scene_understanding_tensorrt
{

class SceneVisualizationNode : public rclcpp::Node
{
public:
  explicit SceneVisualizationNode(const rclcpp::NodeOptions & options);

private:
  using SceneUnderstanding = scene_understanding_tensorrt::msg::SceneUnderstanding;
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, SceneUnderstanding>;

  /**
   * @brief Callback for synchronized image and scene understanding messages
   * @param image_msg Image message
   * @param scene_msg Scene understanding message
   */
  void syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const SceneUnderstanding::ConstSharedPtr & scene_msg);

  /**
   * @brief Visualize scene understanding data on image
   * @param image OpenCV image to draw on
   * @param scene_msg Scene understanding message
   * @return Annotated image
   */
  cv::Mat visualizeImage(
    const cv::Mat & image, const SceneUnderstanding::ConstSharedPtr & scene_msg);

  // Message filters
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> image_sub_;
  std::shared_ptr<message_filters::Subscriber<SceneUnderstanding>> scene_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;

  // Image publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  // Parameters
  double font_scale_;
  int font_thickness_;
  int line_spacing_;
};

}  // namespace scene_understanding_tensorrt

#endif  // SCENE_UNDERSTANDING_TENSORRT__SCENE_VISUALIZATION_NODE_HPP_
