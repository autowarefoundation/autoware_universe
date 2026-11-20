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

#ifndef SCENE_UNDERSTANDING_TENSORRT__SCENE_UNDERSTANDING_NODE_HPP_
#define SCENE_UNDERSTANDING_TENSORRT__SCENE_UNDERSTANDING_NODE_HPP_

#include "scene_understanding_tensorrt/msg/scene_understanding.hpp"
#include "scene_understanding_tensorrt/tensorrt_engine.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

#include <memory>
#include <string>

namespace scene_understanding_tensorrt
{

class SceneUnderstandingNode : public rclcpp::Node
{
public:
  explicit SceneUnderstandingNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief Callback for incoming images
   * @param msg Image message
   */
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Convert TensorRT result to ROS message
   * @param result TensorRT inference result
   * @param header Message header from input image
   * @return SceneUnderstanding message
   */
  msg::SceneUnderstanding resultToMessage(
    const SceneUnderstandingResult & result, const std_msgs::msg::Header & header);

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<msg::SceneUnderstanding>::SharedPtr result_pub_;

  // TensorRT engine
  std::unique_ptr<TensorRTEngine> tensorrt_engine_;

  // Parameters
  std::string onnx_model_path_;
  bool use_fp16_;
  bool use_int8_;
  int max_batch_size_;
};

}  // namespace scene_understanding_tensorrt

#endif  // SCENE_UNDERSTANDING_TENSORRT__SCENE_UNDERSTANDING_NODE_HPP_
