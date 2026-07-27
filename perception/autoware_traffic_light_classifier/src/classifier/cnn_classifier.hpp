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

#ifndef CLASSIFIER__CNN_CLASSIFIER_HPP_
#define CLASSIFIER__CNN_CLASSIFIER_HPP_

#include "classifier_interface.hpp"

#include <autoware/tensorrt_classifier/tensorrt_classifier.hpp>
#include <opencv2/core/core.hpp>

#include <tier4_perception_msgs/msg/traffic_light.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::traffic_light
{
// Configuration for CNNClassifierCore. model_path is a file path (TrtClassifier loads
// the model itself); labels are the pre-read label-file lines, so the core does no file
// I/O of its own.
struct CNNConfig
{
  std::string model_path;
  std::string precision;
  std::vector<std::string> labels;
  std::vector<float> mean;
  std::vector<float> std;
};

// CNN (TensorRT) classification core; the engine is whatever model is configured
// (e.g. MobileNet-v2 or EfficientNet-b1). Its constructor builds a TensorRT engine, so
// instantiating it needs a GPU and the model; decode_label and make_debug_image are
// static so they can be exercised without one.
class CNNClassifierCore
{
public:
  // One signal per input image, elements populated by the label decode. traffic_light_id
  // / type are left unset -- the caller associates them.
  struct ClassifierResult
  {
    tier4_perception_msgs::msg::TrafficLightArray signals;
    bool success = false;
  };

  // Builds the TensorRT engine from config.model_path and stores the label table. Throws
  // std::invalid_argument if mean/std are not size 3 (a TrtClassifier precondition).
  explicit CNNClassifierCore(const CNNConfig & config);

  // Classify each ROI image into one signal, batching up to the model's static batch
  // size. NON-const: TrtClassifier::doInference mutates the engine's internal buffers.
  ClassifierResult classify(const std::vector<cv::Mat> & images);

  // Decode one model label string into per-lamp elements: comma-separated lamps, each a
  // "color-shape" token, a bare color (-> CIRCLE), a bare shape (-> GREEN), or "unknown".
  // Every element gets `confidence`.
  static std::vector<tier4_perception_msgs::msg::TrafficLightElement> decode_label(
    const std::string & label, float confidence);

  // Render one debug image: the ROI resized to a fixed width with a label / confidence
  // text strip below. Returns a fresh Mat (does not mutate roi_image).
  static cv::Mat make_debug_image(
    const cv::Mat & roi_image, const tier4_perception_msgs::msg::TrafficLight & signal);

private:
  std::unique_ptr<autoware::tensorrt_classifier::TrtClassifier> classifier_;
  std::vector<std::string> labels_;
  int batch_size_ = 0;
};

// Thin, Node-free adapter around CNNClassifierCore: delegates classification and debug rendering
// to the core, and maps its per-image output into the caller's signals.
class CNNClassifier : public ClassifierInterface
{
public:
  explicit CNNClassifier(const CNNConfig & config);
  virtual ~CNNClassifier() = default;

  bool getTrafficSignals(
    const std::vector<cv::Mat> & images,
    tier4_perception_msgs::msg::TrafficLightArray & traffic_signals) override;

  cv::Mat make_debug_image(const std::vector<cv::Mat> & images) const override;

private:
  CNNClassifierCore core_;
  // Per-image classification output kept from the most recent getTrafficSignals so
  // make_debug_image can render the batch afterwards.
  tier4_perception_msgs::msg::TrafficLightArray last_signals_;
};

}  // namespace autoware::traffic_light

#endif  // CLASSIFIER__CNN_CLASSIFIER_HPP_
