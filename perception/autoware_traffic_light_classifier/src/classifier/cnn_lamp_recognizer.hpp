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

#ifndef CLASSIFIER__CNN_LAMP_RECOGNIZER_HPP_
#define CLASSIFIER__CNN_LAMP_RECOGNIZER_HPP_

#include "classifier_interface.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/cuda_utils/stream_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

// cppcheck-suppress preprocessorErrorDirective
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <memory>
#include <string>
#include <vector>

namespace autoware::traffic_light
{

// --- Detection / geometry types ---
struct BBox
{
  float x1{0.f}, y1{0.f};
  float x2{0.f}, y2{0.f};
};

struct BBoxInfo
{
  BBox box;
  int classId{0};  // type: circle=0, arrow=1, u-turn=2, ped=3, number=4, cross=5
  float prob{0.f};
  int subClassId{0};  // color: green=0, amber=1, red=2
  float sin{0.f};
  float cos{1.f};
};

enum class Color { GREEN = 0, AMBER = 1, RED = 2, UNKNOWN = 3 };

enum class ArrowDirection {
  UP_ARROW = 0,
  DOWN_ARROW = 1,
  LEFT_ARROW = 2,
  RIGHT_ARROW = 3,
  UP_LEFT_ARROW = 4,
  UP_RIGHT_ARROW = 5,
  DOWN_LEFT_ARROW = 6,
  DOWN_RIGHT_ARROW = 7,
  UNKNOWN = 8,
};

enum class Shape {
  CIRCLE = 0,
  ARROW = 1,
  U_TURN = 2,
  PED = 3,
  NUMBER = 4,
  CROSS = 5,
  UNKNOWN = 6,
};

struct LampElement
{
  Color color;
  Shape shape;
  ArrowDirection arrow_direction;
  BBox box;
  float confidence{0.f};
};

// --- CUDA / TensorRT aliases ---
using autoware::cuda_utils::CudaUniquePtr;
using autoware::cuda_utils::CudaUniquePtrHost;
using autoware::cuda_utils::makeCudaStream;
using autoware::cuda_utils::StreamUniquePtr;

/**
 * @brief Channel layout and anchors for the lamp regression (YOLO-style) head.
 */
struct LampRegressionArchitecture
{
  int num_anchors{3};
  int chans_per_anchor{16};
  int x_index{0};
  int y_index{1};
  int w_index{2};
  int h_index{3};
  int obj_index{4};
  int color_start{5};
  int type_start{8};
  int num_types{6};
  int num_colors{3};
  int cos_index{14};
  int sin_index{15};
  float scale_x_y{2.0f};
  /// YOLO center decode: 0.5f * (scale_x_y - 1.0f), set when loading parameters.
  float bbox_offset{0.5f};
  std::vector<float> anchors;
};

/**
 * @brief Lamp recognizer: per-lamp bbox + color + type + angle (ONNX/TensorRT).
 */
class CnnLampRecognizer : public ClassifierInterface
{
public:
  explicit CnnLampRecognizer(rclcpp::Node * node_ptr);
  ~CnnLampRecognizer() override = default;

  bool getTrafficSignals(
    const std::vector<cv::Mat> & images,
    tier4_perception_msgs::msg::TrafficLightArray & traffic_signals) override;

private:
  void preprocess(const std::vector<cv::Mat> & images);
  bool doInference(size_t batch_size);
  void decodeTlrOutput(size_t batch_size, std::vector<std::vector<BBoxInfo>> & detections_per_roi);

  rclcpp::Node * node_ptr_;
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> trt_common_;
  StreamUniquePtr stream_{makeCudaStream()};

  CudaUniquePtr<float[]> input_d_;
  std::vector<CudaUniquePtr<float[]>> output_d_;
  std::vector<CudaUniquePtrHost<float[]>> output_h_;

  int input_c_;
  int input_height_;
  int input_width_;
  int max_batch_size_;
  int out_c_;
  int output_grid_h_;
  int output_grid_w_;
  float score_threshold_;
  float nms_threshold_;

  LampRegressionArchitecture model_params_;

  image_transport::Publisher image_pub_;
};

}  // namespace autoware::traffic_light

#endif  // CLASSIFIER__CNN_LAMP_RECOGNIZER_HPP_
