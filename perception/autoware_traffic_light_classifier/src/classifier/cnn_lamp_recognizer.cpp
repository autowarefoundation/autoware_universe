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

#include "cnn_lamp_recognizer.hpp"

#include "../traffic_light_classifier_process.hpp"
#include "autoware_utils/math/constants.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
namespace autoware::traffic_light
{

using MsgTE = tier4_perception_msgs::msg::TrafficLightElement;
namespace
{
constexpr int kDebugImageWidth = 200;
constexpr int kDebugTextHeight = 50;
constexpr float PI = static_cast<float>(autoware_utils::pi);
ArrowDirection angleToArrowDirection(float angle_rad)
{
  if (angle_rad >= -PI / 8.0f && angle_rad < PI / 8.0f) return ArrowDirection::UP_ARROW;
  if (angle_rad >= PI / 8.0f && angle_rad < 3.0f * PI / 8.0f) return ArrowDirection::UP_RIGHT_ARROW;
  if (angle_rad >= 3.0f * PI / 8.0f && angle_rad < 5.0f * PI / 8.0f)
    return ArrowDirection::RIGHT_ARROW;
  if (angle_rad >= 5.0f * PI / 8.0f && angle_rad < 7.0f * PI / 8.0f)
    return ArrowDirection::DOWN_RIGHT_ARROW;
  if (angle_rad >= 7.0f * PI / 8.0f || angle_rad < -7.0f * PI / 8.0f)
    return ArrowDirection::DOWN_ARROW;
  if (angle_rad < -5.0f * PI / 8.0f) return ArrowDirection::DOWN_LEFT_ARROW;
  if (angle_rad < -3.0f * PI / 8.0f) return ArrowDirection::LEFT_ARROW;
  return ArrowDirection::UP_LEFT_ARROW;
}

static float get2DIoU(const BBox & bbox1, const BBox & bbox2)
{
  auto overlap1D = [](float x1min, float x1max, float x2min, float x2max) -> float {
    if (x1min > x2min) {
      std::swap(x1min, x2min);
      std::swap(x1max, x2max);
    }
    return x1max < x2min ? 0.0f : std::min(x1max, x2max) - x2min;
  };
  const float overlap_x = overlap1D(bbox1.x1, bbox1.x2, bbox2.x1, bbox2.x2);
  const float overlap_y = overlap1D(bbox1.y1, bbox1.y2, bbox2.y1, bbox2.y2);
  const float area1 = (bbox1.x2 - bbox1.x1) * (bbox1.y2 - bbox1.y1);
  const float area2 = (bbox2.x2 - bbox2.x1) * (bbox2.y2 - bbox2.y1);
  const float overlap_2d = overlap_x * overlap_y;
  const float u = area1 + area2 - overlap_2d;
  return (u == 0.0f) ? 0.0f : overlap_2d / u;
}

static void runNms(
  std::vector<BBoxInfo> & detections, float iou_threshold, std::vector<BBoxInfo> & out)
{
  out.clear();
  if (detections.empty()) return;
  std::stable_sort(
    detections.begin(), detections.end(),
    [](const BBoxInfo & b1, const BBoxInfo & b2) { return b1.prob > b2.prob; });
  for (const auto & i : detections) {
    bool keep = true;
    for (const auto & j : out) {
      if (keep) {
        const float overlap = get2DIoU(i.box, j.box);
        keep = (overlap <= iou_threshold);
      } else {
        break;
      }
    }
    if (keep) out.push_back(i);
  }
}

static void convertBBoxInfoToLampElement(const BBoxInfo & box_info, LampElement & element)
{
  element.color = static_cast<Color>(std::min(2, std::max(0, box_info.subClassId)));
  element.confidence = box_info.prob;
  element.box = box_info.box;
  element.shape = static_cast<Shape>(box_info.classId);
  if (element.shape == Shape::ARROW) {
    // If the network outputs a near-zero direction vector (sin, cos) ~= (0, 0), the direction is
    // considered uncertain. Prefer UNKNOWN rather than biasing inputs to atan2().
    constexpr float kDirectionVecNormSqThreshold = 1.0e-12f;  // (1e-6)^2
    const float norm_sq = box_info.sin * box_info.sin + box_info.cos * box_info.cos;
    if (norm_sq < kDirectionVecNormSqThreshold) {
      element.arrow_direction = ArrowDirection::UNKNOWN;
    } else {
      element.arrow_direction = angleToArrowDirection(std::atan2(box_info.sin, box_info.cos));
    }
  }
}

static void updateTrafficSignals(
  const std::vector<LampElement> & unique_elements,
  tier4_perception_msgs::msg::TrafficLight & traffic_signal)
{
  traffic_signal.elements.clear();
  bool is_pedestrian = traffic_signal.traffic_light_type == 1;
  if (unique_elements.empty()) {
    MsgTE unknown_elem;
    unknown_elem.color = MsgTE::UNKNOWN;
    unknown_elem.shape = MsgTE::UNKNOWN;
    unknown_elem.confidence = 0.0;
    traffic_signal.elements.push_back(unknown_elem);
    return;
  }
  for (const auto & e : unique_elements) {
    MsgTE element;
    element.confidence = e.confidence;
    switch (e.color) {
      case Color::GREEN:
        element.color = MsgTE::GREEN;
        break;
      case Color::AMBER:
        element.color = MsgTE::AMBER;
        break;
      case Color::RED:
        element.color = MsgTE::RED;
        break;
      default:
        element.color = MsgTE::UNKNOWN;
        break;
    }
    if (is_pedestrian || e.shape == Shape::PED) {
      element.shape = MsgTE::CIRCLE;
      traffic_signal.elements.push_back(element);
      continue;
    }
    switch (e.shape) {
      case Shape::CIRCLE:
        element.shape = MsgTE::CIRCLE;
        break;
      case Shape::ARROW:
        switch (e.arrow_direction) {
          case ArrowDirection::UP_ARROW:
            element.shape = MsgTE::UP_ARROW;
            break;
          case ArrowDirection::DOWN_ARROW:
            element.shape = MsgTE::DOWN_ARROW;
            break;
          case ArrowDirection::LEFT_ARROW:
            element.shape = MsgTE::LEFT_ARROW;
            break;
          case ArrowDirection::RIGHT_ARROW:
            element.shape = MsgTE::RIGHT_ARROW;
            break;
          case ArrowDirection::UP_LEFT_ARROW:
            element.shape = MsgTE::UP_LEFT_ARROW;
            break;
          case ArrowDirection::UP_RIGHT_ARROW:
            element.shape = MsgTE::UP_RIGHT_ARROW;
            break;
          case ArrowDirection::DOWN_LEFT_ARROW:
            element.shape = MsgTE::DOWN_LEFT_ARROW;
            break;
          case ArrowDirection::DOWN_RIGHT_ARROW:
            element.shape = MsgTE::DOWN_RIGHT_ARROW;
            break;
          default:
            element.shape = MsgTE::UNKNOWN;
            element.confidence = 0.0;
            break;
        }
        break;
      case Shape::PED:
        element.shape = MsgTE::CIRCLE;
        break;
      case Shape::CROSS:
        element.shape = MsgTE::CROSS;
        break;
      // TODO(badai-nguyen): update u-turn, number when msgs are updated
      default:
        element.shape = MsgTE::UNKNOWN;
        element.confidence = 0.0;
        break;
    }
    traffic_signal.elements.push_back(element);
  }
}

static void outputDebugImage(
  cv::Mat & debug_image, const tier4_perception_msgs::msg::TrafficLight & traffic_signal,
  const std::vector<LampElement> * elements)
{
  const int img_w = debug_image.cols;
  const int img_h = debug_image.rows;

  if (elements && !elements->empty()) {
    static const cv::Scalar colors[] = {
      cv::Scalar(0, 255, 0),    // green  (R, G, B)
      cv::Scalar(255, 255, 0),  // yellow/amber
      cv::Scalar(255, 0, 0),    // red
    };
    for (const auto & d : *elements) {
      const int x1 = static_cast<int>(d.box.x1 * img_w);
      const int y1 = static_cast<int>(d.box.y1 * img_h);
      const int x2 = static_cast<int>(d.box.x2 * img_w);
      const int y2 = static_cast<int>(d.box.y2 * img_h);
      const int color_idx = std::min(2, std::max(0, static_cast<int>(d.color)));
      cv::rectangle(debug_image, cv::Point(x1, y1), cv::Point(x2, y2), colors[color_idx], 2);
    }
  }

  float probability = 0.0f;
  std::string label;
  for (std::size_t i = 0; i < traffic_signal.elements.size(); i++) {
    auto light = traffic_signal.elements.at(i);
    const auto light_label =
      utils::convertColorT4toString(light.color) + "-" + utils::convertShapeT4toString(light.shape);
    label += light_label;
    // all lamp confidence are the same
    probability = light.confidence;
    if (i < traffic_signal.elements.size() - 1) {
      label += ",";
    }
  }
  const std::string text = label + " " + std::to_string(probability);
  const int expand_h =
    std::max(static_cast<int>((kDebugImageWidth * debug_image.rows) / debug_image.cols), 1);
  cv::resize(debug_image, debug_image, cv::Size(kDebugImageWidth, expand_h));
  cv::Mat text_img(cv::Size(kDebugImageWidth, kDebugTextHeight), CV_8UC3, cv::Scalar(0, 0, 0));
  cv::putText(
    text_img, text, cv::Point(5, 25), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
  cv::vconcat(debug_image, text_img, debug_image);
}

}  // namespace

CnnLampRecognizer::CnnLampRecognizer(rclcpp::Node * node_ptr) : node_ptr_(node_ptr)
{
  image_pub_ = image_transport::create_publisher(
    node_ptr_, "~/output/debug/image", rclcpp::QoS{1}.get_rmw_qos_profile());

  const std::string model_path = node_ptr_->declare_parameter<std::string>("model_path");
  const std::string precision = node_ptr_->declare_parameter<std::string>("precision");
  score_threshold_ = static_cast<float>(node_ptr_->declare_parameter<double>("score_threshold"));
  nms_threshold_ = static_cast<float>(node_ptr_->declare_parameter<double>("nms_threshold"));
  max_batch_size_ = node_ptr_->declare_parameter<int>("max_batch_size");

  model_params_.num_anchors = node_ptr_->declare_parameter<int>("model_params.num_anchors");
  model_params_.chans_per_anchor =
    node_ptr_->declare_parameter<int>("model_params.chans_per_anchor");
  model_params_.x_index = node_ptr_->declare_parameter<int>("model_params.x_index");
  model_params_.y_index = node_ptr_->declare_parameter<int>("model_params.y_index");
  model_params_.w_index = node_ptr_->declare_parameter<int>("model_params.w_index");
  model_params_.h_index = node_ptr_->declare_parameter<int>("model_params.h_index");
  model_params_.obj_index = node_ptr_->declare_parameter<int>("model_params.obj_index");
  model_params_.color_start = node_ptr_->declare_parameter<int>("model_params.color_start");
  model_params_.type_start = node_ptr_->declare_parameter<int>("model_params.type_start");
  model_params_.num_types = node_ptr_->declare_parameter<int>("model_params.num_types");
  model_params_.num_colors = node_ptr_->declare_parameter<int>("model_params.num_colors");
  model_params_.cos_index = node_ptr_->declare_parameter<int>("model_params.cos_index");
  model_params_.sin_index = node_ptr_->declare_parameter<int>("model_params.sin_index");
  model_params_.scale_x_y =
    static_cast<float>(node_ptr_->declare_parameter<double>("model_params.scale_x_y"));
  model_params_.bbox_offset = 0.5f * (model_params_.scale_x_y - 1.0f);
  {
    const auto anchors_param =
      node_ptr_->declare_parameter<std::vector<double>>("model_params.anchors");
    model_params_.anchors.clear();
    model_params_.anchors.reserve(anchors_param.size());
    for (double v : anchors_param) {
      model_params_.anchors.push_back(static_cast<float>(v));
    }
    if (static_cast<int>(model_params_.anchors.size()) != 2 * model_params_.num_anchors) {
      throw std::runtime_error(
        "CnnLampRecognizer: model_params.anchors must contain 2 * model_params.num_anchors "
        "values (w,h per anchor)");
    }
  }

  autoware::tensorrt_common::TrtCommonConfig config(
    model_path, precision, "", (1ULL << 30U), -1, false);
  trt_common_ = std::make_unique<autoware::tensorrt_common::TrtCommon>(config);

  nvinfer1::Dims input_dims = trt_common_->getInputDims(0);
  input_c_ = input_dims.d[1] > 0 ? input_dims.d[1] : 3;
  input_height_ = input_dims.d[2];
  input_width_ = input_dims.d[3];

  const int min_batch = 1;
  const int opt_batch = std::min(32, max_batch_size_);
  const int max_batch = max_batch_size_;

  nvinfer1::Dims dim_min, dim_opt, dim_max;
  dim_min.nbDims = 4;
  dim_min.d[0] = min_batch;
  dim_min.d[1] = input_c_;
  dim_min.d[2] = input_height_;
  dim_min.d[3] = input_width_;
  dim_opt.nbDims = 4;
  dim_opt.d[0] = opt_batch;
  dim_opt.d[1] = input_c_;
  dim_opt.d[2] = input_height_;
  dim_opt.d[3] = input_width_;
  dim_max.nbDims = 4;
  dim_max.d[0] = max_batch;
  dim_max.d[1] = input_c_;
  dim_max.d[2] = input_height_;
  dim_max.d[3] = input_width_;

  auto profile_dims = std::make_unique<std::vector<autoware::tensorrt_common::ProfileDims>>();
  profile_dims->push_back(autoware::tensorrt_common::ProfileDims(0, dim_min, dim_opt, dim_max));

  if (!trt_common_->setup(std::move(profile_dims), nullptr)) {
    throw std::runtime_error("CnnLampRecognizer: Failed to setup TensorRT engine");
  }

  const int32_t nb_io = trt_common_->getNbIOTensors();
  const int32_t num_outputs = nb_io - 1;
  if (num_outputs < 1) {
    throw std::runtime_error("CnnLampRecognizer: engine has no output bindings");
  }

  output_d_.resize(num_outputs);
  output_h_.resize(num_outputs);

  for (int32_t o = 0; o < num_outputs; ++o) {
    nvinfer1::Dims dims = trt_common_->getOutputDims(o);
    const int32_t batch_dim = dims.d[0] > 0 ? dims.d[0] : max_batch_size_;
    dims.d[0] = batch_dim >= 1 ? batch_dim : 1;
    if (o == 0) {
      out_c_ = dims.d[1] > 0 ? dims.d[1] : 48;
      output_grid_h_ = dims.d[2] > 0 ? dims.d[2] : 8;
      output_grid_w_ = dims.d[3] > 0 ? dims.d[3] : 8;
      dims.d[1] = out_c_;
      dims.d[2] = output_grid_h_;
      dims.d[3] = output_grid_w_;
      const int expected_channels = model_params_.num_anchors * model_params_.chans_per_anchor;
      if (out_c_ != expected_channels) {
        throw std::runtime_error(
          "CnnLampRecognizer: Model output channels (" + std::to_string(out_c_) +
          ") do not match YAML configuration (" + std::to_string(expected_channels) + ").");
      }
    }
    for (int32_t j = 1; j < dims.nbDims; ++j) {
      if (dims.d[j] <= 0) dims.d[j] = 1;
    }
    const size_t vol = std::accumulate(
      dims.d, dims.d + dims.nbDims, static_cast<size_t>(1), std::multiplies<size_t>());
    output_d_[o] = autoware::cuda_utils::make_unique<float[]>(vol);
    output_h_[o] = autoware::cuda_utils::make_unique_host<float[]>(vol, cudaHostAllocPortable);
  }

  const size_t input_vol =
    static_cast<size_t>(max_batch_size_) * input_c_ * input_height_ * input_width_;
  input_d_ = autoware::cuda_utils::make_unique<float[]>(input_vol);
}

void CnnLampRecognizer::preprocess(const std::vector<cv::Mat> & images)
{
  const float scale = 1.0f / 255.0f;
  const cv::Size input_size(input_width_, input_height_);
  cv::Mat blob =
    cv::dnn::blobFromImages(images, scale, input_size, cv::Scalar(0, 0, 0), false, false, CV_32F);
  if (!blob.isContinuous()) {
    blob = blob.clone();
  }
  const size_t copy_size = blob.total() * sizeof(float);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    input_d_.get(), blob.ptr<float>(), copy_size, cudaMemcpyHostToDevice, *stream_));
}

bool CnnLampRecognizer::doInference(size_t batch_size)
{
  nvinfer1::Dims input_dims = trt_common_->getInputDims(0);
  input_dims.d[0] = static_cast<int32_t>(batch_size);
  if (!trt_common_->setInputShape(0, input_dims)) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "LampRecognizer: setInputShape failed");
    return false;
  }

  std::vector<void *> buffers = {input_d_.get()};
  for (size_t i = 0; i < output_d_.size(); ++i) {
    buffers.push_back(output_d_.at(i).get());
  }
  if (!trt_common_->setTensorsAddresses(buffers)) {
    return false;
  }
  if (!trt_common_->enqueueV3(*stream_)) {
    return false;
  }

  for (size_t i = 0; i < output_d_.size(); ++i) {
    nvinfer1::Dims dims = trt_common_->getOutputDims(static_cast<int32_t>(i));
    dims.d[0] = static_cast<int32_t>(batch_size);
    for (int32_t j = 1; j < dims.nbDims; ++j) {
      if (dims.d[j] <= 0) dims.d[j] = 1;
    }
    const size_t output_size = std::accumulate(
      dims.d, dims.d + dims.nbDims, static_cast<size_t>(1), std::multiplies<size_t>());
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      output_h_.at(i).get(), output_d_.at(i).get(), output_size * sizeof(float),
      cudaMemcpyDeviceToHost, *stream_));
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(*stream_));
  return true;
}

void CnnLampRecognizer::decodeTlrOutput(
  size_t batch_size, std::vector<std::vector<BBoxInfo>> & detections_per_roi)
{
  const auto & ml_params = model_params_;
  detections_per_roi.resize(batch_size);

  const int grid_size = output_grid_h_ * output_grid_w_;
  const size_t batch_stride =
    static_cast<size_t>(out_c_) * static_cast<size_t>(output_grid_h_) * output_grid_w_;
  const float * out_base = output_h_.at(0).get();
  for (size_t b = 0; b < batch_size; ++b) {
    detections_per_roi[b].clear();

    std::vector<BBoxInfo> raw;
    const float * out = out_base + b * batch_stride;
    const float grid_w_f = static_cast<float>(output_grid_w_);
    const float grid_h_f = static_cast<float>(output_grid_h_);

    for (int y = 0; y < output_grid_h_; ++y) {
      for (int x = 0; x < output_grid_w_; ++x) {
        const int cell = y * output_grid_w_ + x;
        for (int a = 0; a < ml_params.num_anchors; ++a) {
          const int base = (a * ml_params.chans_per_anchor) * grid_size + cell;

          const float objectness = out[base + ml_params.obj_index * grid_size];
          if (objectness < score_threshold_) continue;

          const float pw = ml_params.anchors[static_cast<size_t>(a) * 2];
          const float ph = ml_params.anchors[static_cast<size_t>(a) * 2 + 1];
          const float tx = out[base + ml_params.x_index * grid_size];
          const float ty = out[base + ml_params.y_index * grid_size];
          const float tw = out[base + ml_params.w_index * grid_size];
          const float th = out[base + ml_params.h_index * grid_size];
          const float bx = x + ml_params.scale_x_y * tx - ml_params.bbox_offset;
          const float by = y + ml_params.scale_x_y * ty - ml_params.bbox_offset;
          const float bw = pw * std::pow(tw * 2.0f, 2.0f);
          const float bh = ph * std::pow(th * 2.0f, 2.0f);

          float max_type_prob = 0.0f;
          int type_idx = 0;
          for (int t = 0; t < ml_params.num_types; ++t) {
            const float p = out[base + (ml_params.type_start + t) * grid_size];
            if (p > max_type_prob) {
              max_type_prob = p;
              type_idx = t;
            }
          }
          float max_color_prob = 0.0f;
          int color_idx = 0;
          for (int c = 0; c < ml_params.num_colors; ++c) {
            const float p = out[base + (ml_params.color_start + c) * grid_size];
            if (p > max_color_prob) {
              max_color_prob = p;
              color_idx = c;
            }
          }

          const float score = objectness * max_type_prob;
          if (score < score_threshold_) continue;

          const float cos_val = out[base + ml_params.cos_index * grid_size];
          const float sin_val = out[base + ml_params.sin_index * grid_size];

          const float cx = bx / grid_w_f;
          const float cy = by / grid_h_f;
          const float input_w_f = static_cast<float>(input_width_);
          const float input_h_f = static_cast<float>(input_height_);
          const float half_w = (bw * 0.5f) / input_w_f;
          const float half_h = (bh * 0.5f) / input_h_f;
          float x1 = cx - half_w;
          float y1 = cy - half_h;
          float x2 = cx + half_w;
          float y2 = cy + half_h;
          x1 = std::max(0.0f, std::min(1.0f, x1));
          y1 = std::max(0.0f, std::min(1.0f, y1));
          x2 = std::max(0.0f, std::min(1.0f, x2));
          y2 = std::max(0.0f, std::min(1.0f, y2));

          BBoxInfo info;
          info.box.x1 = x1;
          info.box.y1 = y1;
          info.box.x2 = x2;
          info.box.y2 = y2;
          info.classId = type_idx;
          info.prob = score;
          info.subClassId = color_idx;
          info.sin = sin_val;
          info.cos = cos_val;
          raw.push_back(info);
        }
      }
    }

    runNms(raw, nms_threshold_, detections_per_roi[b]);
  }
}

bool CnnLampRecognizer::getTrafficSignals(
  const std::vector<cv::Mat> & images,
  tier4_perception_msgs::msg::TrafficLightArray & traffic_signals)
{
  if (images.size() != traffic_signals.signals.size()) {
    RCLCPP_WARN(
      node_ptr_->get_logger(), "LampRecognizer: image count (%zu) != signal count (%zu)",
      images.size(), traffic_signals.signals.size());
    return false;
  }

  size_t signal_i = 0;
  std::vector<cv::Mat> batch;
  std::vector<std::vector<LampElement>> unique_lamps_per_image;

  for (size_t image_i = 0; image_i < images.size(); image_i++) {
    batch.push_back(images[image_i]);
    const size_t current_batch_size = batch.size();
    const bool flush = (current_batch_size >= static_cast<size_t>(max_batch_size_)) ||
                       (image_i + 1 == images.size());

    if (!flush) continue;

    preprocess(batch);
    if (!doInference(current_batch_size)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "LampRecognizer: inference failed");
      return false;
    }

    std::vector<std::vector<BBoxInfo>> detections_per_roi;
    decodeTlrOutput(current_batch_size, detections_per_roi);
    for (size_t i = 0; i < current_batch_size; ++i) {
      const size_t sig_idx = signal_i + i;

      std::vector<LampElement> traffic_lamps;
      for (const auto & d : detections_per_roi[i]) {
        LampElement element;
        convertBBoxInfoToLampElement(d, element);
        traffic_lamps.push_back(element);
      }
      // if multiple detections have the same shape and arrow direction
      // keep the one with highest confidence
      std::vector<LampElement> unique_lamps;
      for (const auto & lamp : traffic_lamps) {
        auto it =
          std::find_if(unique_lamps.begin(), unique_lamps.end(), [&](const LampElement & e) {
            return e.shape == lamp.shape && e.arrow_direction == lamp.arrow_direction;
          });
        if (it == unique_lamps.end()) {
          unique_lamps.push_back(lamp);
        } else if (lamp.confidence > it->confidence) {
          *it = lamp;  // Overwrite the whole object to keep the better bounding box
        }
      }

      updateTrafficSignals(unique_lamps, traffic_signals.signals[sig_idx]);
      unique_lamps_per_image.push_back(unique_lamps);
    }
    signal_i += current_batch_size;
    batch.clear();
  }
  if (image_pub_.getNumSubscribers() > 0 && !images.empty()) {
    // build debug image by vertically concatenating each image's debug view
    const int strip_width = 200;
    const int strip_height = 130;
    cv::Mat debug_img;
    for (size_t i = 0; i < images.size(); i++) {
      cv::Mat debug_img_i = images[i].clone();
      outputDebugImage(debug_img_i, traffic_signals.signals[i], &unique_lamps_per_image[i]);
      cv::resize(debug_img_i, debug_img_i, cv::Size(strip_width, strip_height));
      if (i == 0) {
        debug_img = debug_img_i;
      } else {
        cv::vconcat(debug_img, debug_img_i, debug_img);
      }
    }
    const auto debug_image_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", debug_img).toImageMsg();
    image_pub_.publish(debug_image_msg);
  }
  return true;
}

}  // namespace autoware::traffic_light
