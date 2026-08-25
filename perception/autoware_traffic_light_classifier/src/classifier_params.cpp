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

#include "classifier_params.hpp"

#if ENABLE_GPU
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#endif

namespace autoware::traffic_light
{
HSVConfig declare_hsv_config(rclcpp::Node * node)
{
  HSVConfig config;
  config.green_min_h = node->declare_parameter("green_min_h", config.green_min_h);
  config.green_min_s = node->declare_parameter("green_min_s", config.green_min_s);
  config.green_min_v = node->declare_parameter("green_min_v", config.green_min_v);
  config.green_max_h = node->declare_parameter("green_max_h", config.green_max_h);
  config.green_max_s = node->declare_parameter("green_max_s", config.green_max_s);
  config.green_max_v = node->declare_parameter("green_max_v", config.green_max_v);
  config.yellow_min_h = node->declare_parameter("yellow_min_h", config.yellow_min_h);
  config.yellow_min_s = node->declare_parameter("yellow_min_s", config.yellow_min_s);
  config.yellow_min_v = node->declare_parameter("yellow_min_v", config.yellow_min_v);
  config.yellow_max_h = node->declare_parameter("yellow_max_h", config.yellow_max_h);
  config.yellow_max_s = node->declare_parameter("yellow_max_s", config.yellow_max_s);
  config.yellow_max_v = node->declare_parameter("yellow_max_v", config.yellow_max_v);
  config.red_min_h = node->declare_parameter("red_min_h", config.red_min_h);
  config.red_min_s = node->declare_parameter("red_min_s", config.red_min_s);
  config.red_min_v = node->declare_parameter("red_min_v", config.red_min_v);
  config.red_max_h = node->declare_parameter("red_max_h", config.red_max_h);
  config.red_max_s = node->declare_parameter("red_max_s", config.red_max_s);
  config.red_max_v = node->declare_parameter("red_max_v", config.red_max_v);
  return config;
}

#if ENABLE_GPU
namespace
{
// Read the label file into a vector of lines. Logs and throws std::runtime_error if the file
// cannot be opened, so a misconfigured path fails node construction fast rather than leaving the
// classifier with an empty label table (which would be an out-of-range lookup at inference time).
std::vector<std::string> read_label_file(rclcpp::Node * node, const std::string & filepath)
{
  std::ifstream labels_file(filepath);
  if (!labels_file.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Could not open label file. [%s]", filepath.c_str());
    throw std::runtime_error("Could not open label file: " + filepath);
  }
  std::vector<std::string> labels;
  std::string label;
  while (std::getline(labels_file, label)) {
    labels.push_back(label);
  }
  return labels;
}
}  // namespace

CNNConfig declare_cnn_config(rclcpp::Node * node)
{
  const std::string precision = node->declare_parameter<std::string>("precision");
  const std::string label_path = node->declare_parameter<std::string>("label_path");
  const std::string model_path = node->declare_parameter<std::string>("model_path");
  const auto mean_d = node->declare_parameter<std::vector<double>>("mean");
  const auto std_d = node->declare_parameter<std::vector<double>>("std");

  CNNConfig config;
  config.model_path = model_path;
  config.precision = precision;
  config.labels = read_label_file(node, label_path);
  config.mean = std::vector<float>(mean_d.begin(), mean_d.end());
  config.std = std::vector<float>(std_d.begin(), std_d.end());
  return config;
}

CnnLampRecognizerConfig declare_lamp_config(rclcpp::Node * node)
{
  CnnLampRecognizerConfig config;
  config.model_path = node->declare_parameter<std::string>("model_path");
  config.precision = node->declare_parameter<std::string>("precision");
  config.score_threshold = static_cast<float>(node->declare_parameter<double>("score_threshold"));
  config.nms_threshold = static_cast<float>(node->declare_parameter<double>("nms_threshold"));
  config.max_batch_size = node->declare_parameter<int>("max_batch_size");

  auto & model_params = config.model_params;
  model_params.num_anchors = node->declare_parameter<int>("model_params.num_anchors");
  model_params.chans_per_anchor = node->declare_parameter<int>("model_params.chans_per_anchor");
  model_params.x_index = node->declare_parameter<int>("model_params.x_index");
  model_params.y_index = node->declare_parameter<int>("model_params.y_index");
  model_params.w_index = node->declare_parameter<int>("model_params.w_index");
  model_params.h_index = node->declare_parameter<int>("model_params.h_index");
  model_params.obj_index = node->declare_parameter<int>("model_params.obj_index");
  model_params.color_start = node->declare_parameter<int>("model_params.color_start");
  model_params.type_start = node->declare_parameter<int>("model_params.type_start");
  model_params.num_types = node->declare_parameter<int>("model_params.num_types");
  model_params.num_colors = node->declare_parameter<int>("model_params.num_colors");
  model_params.cos_index = node->declare_parameter<int>("model_params.cos_index");
  model_params.sin_index = node->declare_parameter<int>("model_params.sin_index");
  model_params.scale_x_y =
    static_cast<float>(node->declare_parameter<double>("model_params.scale_x_y"));

  const auto anchors_param = node->declare_parameter<std::vector<double>>("model_params.anchors");
  model_params.anchors.clear();
  model_params.anchors.reserve(anchors_param.size());
  for (double v : anchors_param) {
    model_params.anchors.push_back(static_cast<float>(v));
  }

  return config;
}
#endif
}  // namespace autoware::traffic_light
