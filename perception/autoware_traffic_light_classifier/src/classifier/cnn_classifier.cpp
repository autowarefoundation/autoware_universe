// Copyright 2023 TIER IV, Inc.
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

#include "cnn_classifier.hpp"

#include "../traffic_light_classifier_process.hpp"

#include <opencv2/imgproc.hpp>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::traffic_light
{
// ============================= CNNClassifierCore =============================
// Node-free CNN (TensorRT) classification core.

CNNClassifierCore::CNNClassifierCore(const CNNConfig & config)
{
  if (config.mean.size() != 3 || config.std.size() != 3) {
    throw std::invalid_argument("mean and std must be of size 3");
  }
  labels_ = config.labels;
  classifier_ = std::make_unique<autoware::tensorrt_classifier::TrtClassifier>(
    config.model_path, config.precision, config.mean, config.std);
  batch_size_ = classifier_->getBatchSize();
}

CNNClassifierCore::ClassifierResult CNNClassifierCore::classify(const std::vector<cv::Mat> & images)
{
  ClassifierResult result;
  result.signals.signals.resize(images.size());

  std::vector<cv::Mat> image_batch;
  size_t signal_i = 0;
  for (size_t image_i = 0; image_i < images.size(); image_i++) {
    image_batch.emplace_back(images[image_i]);
    // keep the actual batch size
    const size_t true_batch_size = image_batch.size();
    // insert fake images since the TRT model requires a static batch size
    if (image_i + 1 == images.size()) {
      while (static_cast<int>(image_batch.size()) < batch_size_) {
        image_batch.emplace_back(image_batch.front());
      }
    }
    if (static_cast<int>(image_batch.size()) == batch_size_) {
      std::vector<float> confidences;
      std::vector<int> classes;
      const bool res = classifier_->doInference(image_batch, classes, confidences);
      if (!res || classes.empty() || confidences.empty()) {
        result.success = false;
        return result;
      }
      for (size_t i = 0; i < true_batch_size; i++) {
        const auto elements = decode_label(labels_[classes[i]], confidences[i]);
        auto & signal_elements = result.signals.signals[signal_i].elements;
        signal_elements.insert(signal_elements.end(), elements.begin(), elements.end());
        signal_i++;
      }
      image_batch.clear();
    }
  }
  result.success = true;
  return result;
}

std::vector<tier4_perception_msgs::msg::TrafficLightElement> CNNClassifierCore::decode_label(
  const std::string & label, float confidence)
{
  // label names are assumed to be comma-separated to represent each lamp
  // e.g.
  //   label:       "red,red-cross,right"
  //   split_label: ["red", "red-cross", "right"]
  // if a token has no color suffix, set GREEN to the color state.
  // if a token has no shape suffix, set CIRCLE to the shape state.
  std::vector<tier4_perception_msgs::msg::TrafficLightElement> elements;
  std::vector<std::string> split_label;
  boost::algorithm::split(split_label, label, boost::is_any_of(","));
  for (const auto & lamp_label : split_label) {
    tier4_perception_msgs::msg::TrafficLightElement element;
    if (lamp_label.find("-") != std::string::npos) {
      // found "-" delimiter in the label string
      std::vector<std::string> color_and_shape;
      boost::algorithm::split(color_and_shape, lamp_label, boost::is_any_of("-"));
      element.color = utils::convertColorStringtoT4(color_and_shape.at(0));
      element.shape = utils::convertShapeStringtoT4(color_and_shape.at(1));
    } else {
      if (lamp_label == std::string("unknown")) {
        // if the label is unknown, set UNKNOWN to color and shape
        element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
        element.shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
      } else if (utils::isColorLabel(lamp_label)) {
        element.color = utils::convertColorStringtoT4(lamp_label);
        element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
      } else {
        element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
        element.shape = utils::convertShapeStringtoT4(lamp_label);
      }
    }
    element.confidence = confidence;
    elements.push_back(element);
  }
  return elements;
}

cv::Mat CNNClassifierCore::make_debug_image(
  const cv::Mat & roi_image, const tier4_perception_msgs::msg::TrafficLight & signal)
{
  float confidence = 0.0f;
  std::string label;
  for (std::size_t i = 0; i < signal.elements.size(); i++) {
    const auto & light = signal.elements.at(i);
    const auto light_label =
      utils::convertColorT4toString(light.color) + "-" + utils::convertShapeT4toString(light.shape);
    label += light_label;
    // all lamp confidences are the same
    confidence = light.confidence;
    if (i < signal.elements.size() - 1) {
      label += ",";
    }
  }

  const int expand_w = 200;
  const int expand_h = std::max(static_cast<int>((expand_w * roi_image.rows) / roi_image.cols), 1);

  cv::Mat debug_image;
  cv::resize(roi_image, debug_image, cv::Size(expand_w, expand_h));
  cv::Mat text_img(cv::Size(expand_w, 50), CV_8UC3, cv::Scalar(0, 0, 0));
  const std::string text = label + " " + std::to_string(confidence);
  cv::putText(
    text_img, text, cv::Point(5, 25), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
  cv::vconcat(debug_image, text_img, debug_image);
  return debug_image;
}

// ============================== CNNClassifier ==============================
// ROS adapter: logs and delegates classification and debug-image rendering to the Node-free core.

CNNClassifier::CNNClassifier(rclcpp::Node * node_ptr, const CNNConfig & config)
: node_ptr_(node_ptr), core_(config)
{
}

bool CNNClassifier::getTrafficSignals(
  const std::vector<cv::Mat> & images,
  tier4_perception_msgs::msg::TrafficLightArray & traffic_signals)
{
  if (images.size() != traffic_signals.signals.size()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "image number should be equal to traffic signal number!");
    return false;
  }

  const CNNClassifierCore::ClassifierResult result = core_.classify(images);
  if (!result.success) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "failed to classify traffic light image by cnn");
    return false;
  }

  // Attach the core's per-image elements to the caller's pre-populated signals,
  // preserving the traffic_light_id / traffic_light_type set upstream.
  for (size_t i = 0; i < traffic_signals.signals.size(); i++) {
    auto & elements = traffic_signals.signals[i].elements;
    const auto & classified = result.signals.signals[i].elements;
    elements.insert(elements.end(), classified.begin(), classified.end());
  }

  // Keep the per-image classification so make_debug_image can render it afterwards; the node
  // owns the debug publisher and requests the image only when a consumer is attached.
  last_signals_ = result.signals;

  return true;
}

cv::Mat CNNClassifier::make_debug_image(const std::vector<cv::Mat> & images) const
{
  // Stack each ROI's debug view (fixed 200 px wide) into one vertical strip.
  cv::Mat debug_image;
  const size_t count = std::min(images.size(), last_signals_.signals.size());
  for (size_t i = 0; i < count; i++) {
    cv::Mat strip = CNNClassifierCore::make_debug_image(images[i], last_signals_.signals[i]);
    if (debug_image.empty()) {
      debug_image = strip;
    } else {
      cv::vconcat(debug_image, strip, debug_image);
    }
  }
  return debug_image;
}

}  // namespace autoware::traffic_light
