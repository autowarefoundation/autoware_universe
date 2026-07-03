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

#ifndef CLASSIFIER__COLOR_CLASSIFIER_HPP_
#define CLASSIFIER__COLOR_CLASSIFIER_HPP_

#include "classifier_interface.hpp"

#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <vector>

namespace autoware::traffic_light
{
// HSV thresholds for the green / yellow / red bands. Defaults are the single
// source of truth for the classifier's out-of-the-box behavior; the ROS adapter
// exposes them as node parameters.
struct HSVConfig
{
  int green_min_h = 50;
  int green_min_s = 100;
  int green_min_v = 150;
  int green_max_h = 120;
  int green_max_s = 200;
  int green_max_v = 255;
  int yellow_min_h = 0;
  int yellow_min_s = 80;
  int yellow_min_v = 150;
  int yellow_max_h = 50;
  int yellow_max_s = 200;
  int yellow_max_v = 255;
  int red_min_h = 160;
  int red_min_s = 100;
  int red_min_v = 150;
  int red_max_h = 180;
  int red_max_s = 255;
  int red_max_v = 255;
};

// Node-free core of the HSV color classifier. Depends only on OpenCV and the
// perception message types -- no rclcpp, image_transport, or logging -- so it can
// be constructed and unit-tested without a running node.
class ColorClassifierCore
{
public:
  // Per-batch result: one signal (with a single color element) per input image,
  // plus whether the HSV pipeline ran without error. traffic_light_id / type on
  // the signals are left unset -- associating them is the caller's job.
  struct ClassifierResult
  {
    tier4_perception_msgs::msg::TrafficLightArray signals;
    bool success = false;
  };

  explicit ColorClassifierCore(const HSVConfig & config = HSVConfig{});

  // Classify each ROI image (a cropped traffic-light region) into a
  // TrafficLightElement by HSV color band.
  ClassifierResult classify(const std::vector<cv::Mat> & images) const;

  // Render one debug mosaic for a single ROI image. Independent of
  // classify: it re-runs the HSV pipeline internally, so the caller
  // invokes it only when a debug consumer is attached (a cold path).
  cv::Mat make_debug_image(const cv::Mat & roi_image) const;

  // Replace the HSV thresholds and rebuild the color bands (dynamic reconfigure).
  void set_config(const HSVConfig & config);

private:
  // The three pipeline stages of one color channel: filtered = cv::inRange output,
  // binarized = post-threshold, denoised = post-erode/dilate. classify_element reads
  // the denoised mask; make_debug_image renders all three.
  struct ColorStageImages
  {
    cv::Mat filtered;
    cv::Mat binarized;
    cv::Mat denoised;
  };
  // Per-image intermediate images for all three color channels.
  struct PipelineStages
  {
    ColorStageImages green;
    ColorStageImages yellow;
    ColorStageImages red;
  };
  // Output of run_pipeline: the per-color stage images plus whether the HSV filter
  // ran without error.
  struct PipelineResult
  {
    PipelineStages stages;
    bool filter_ok = true;
  };

  // Run the HSV filter -> binarize -> denoise pipeline for one ROI image. Shared by
  // classification (classify) and debug rendering (make_debug_image).
  PipelineResult run_pipeline(const cv::Mat & roi_image) const;
  // Pick one TrafficLightElement from the denoised per-color masks: the dominant
  // color band wins, with confidence scaled by its matching pixel count. Static: it
  // derives the element purely from the stage masks, using no member state.
  static tier4_perception_msgs::msg::TrafficLightElement classify_element(
    const PipelineStages & stages);
  bool filter_hsv(
    const cv::Mat & roi_image, cv::Mat & green_filtered_image, cv::Mat & yellow_filtered_image,
    cv::Mat & red_filtered_image) const;
  // Rebuild the cv::Scalar bands from hsv_config_.
  void update_thresholds();

  HSVConfig hsv_config_;
  cv::Scalar min_hsv_green_;
  cv::Scalar max_hsv_green_;
  cv::Scalar min_hsv_yellow_;
  cv::Scalar max_hsv_yellow_;
  cv::Scalar min_hsv_red_;
  cv::Scalar max_hsv_red_;
};

// Thin ROS adapter around ColorClassifierCore. Owns the node-facing concerns
// (parameter declaration, dynamic reconfigure, debug-image publishing, logging)
// and delegates classification to the core. Public API is unchanged.
class ColorClassifier : public ClassifierInterface
{
public:
  explicit ColorClassifier(rclcpp::Node * node_ptr);
  virtual ~ColorClassifier() = default;

  bool getTrafficSignals(
    const std::vector<cv::Mat> & images,
    tier4_perception_msgs::msg::TrafficLightArray & traffic_signals) override;

private:
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

private:
  image_transport::Publisher image_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rclcpp::Node * node_ptr_;

  HSVConfig hsv_config_;
  ColorClassifierCore core_;
};

}  // namespace autoware::traffic_light

#endif  // CLASSIFIER__COLOR_CLASSIFIER_HPP_
