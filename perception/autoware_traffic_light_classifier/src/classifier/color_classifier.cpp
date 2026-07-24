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
#include "color_classifier.hpp"

#include <opencv2/imgproc.hpp>

#include <opencv2/imgproc/imgproc_c.h>

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::traffic_light
{
// ============================ ColorClassifierCore ============================
// Node-free HSV classification core.

ColorClassifierCore::ColorClassifierCore(const HSVConfig & config)
{
  set_config(config);
}

void ColorClassifierCore::set_config(const HSVConfig & config)
{
  hsv_config_ = config;
  update_thresholds();
}

const HSVConfig & ColorClassifierCore::get_config() const
{
  return hsv_config_;
}

void ColorClassifierCore::update_thresholds()
{
  min_hsv_green_ =
    cv::Scalar(hsv_config_.green_min_h, hsv_config_.green_min_s, hsv_config_.green_min_v);
  max_hsv_green_ =
    cv::Scalar(hsv_config_.green_max_h, hsv_config_.green_max_s, hsv_config_.green_max_v);
  min_hsv_yellow_ =
    cv::Scalar(hsv_config_.yellow_min_h, hsv_config_.yellow_min_s, hsv_config_.yellow_min_v);
  max_hsv_yellow_ =
    cv::Scalar(hsv_config_.yellow_max_h, hsv_config_.yellow_max_s, hsv_config_.yellow_max_v);
  min_hsv_red_ = cv::Scalar(hsv_config_.red_min_h, hsv_config_.red_min_s, hsv_config_.red_min_v);
  max_hsv_red_ = cv::Scalar(hsv_config_.red_max_h, hsv_config_.red_max_s, hsv_config_.red_max_v);
}

bool ColorClassifierCore::filter_hsv(
  const cv::Mat & roi_image, cv::Mat & green_filtered_image, cv::Mat & yellow_filtered_image,
  cv::Mat & red_filtered_image) const
{
  cv::Mat hsv_image;
  cv::cvtColor(roi_image, hsv_image, cv::COLOR_BGR2HSV);
  try {
    cv::inRange(hsv_image, min_hsv_green_, max_hsv_green_, green_filtered_image);
    cv::inRange(hsv_image, min_hsv_yellow_, max_hsv_yellow_, yellow_filtered_image);
    cv::inRange(hsv_image, min_hsv_red_, max_hsv_red_, red_filtered_image);
  } catch (const cv::Exception &) {
    return false;
  }
  return true;
}

ColorClassifierCore::ClassifierResult ColorClassifierCore::classify(
  const std::vector<cv::Mat> & images) const
{
  ClassifierResult result;
  result.success = true;
  result.signals.signals.resize(images.size());
  for (size_t roi_i = 0; roi_i < images.size(); roi_i++) {
    const PipelineResult pipeline = run_pipeline(images[roi_i]);
    result.signals.signals[roi_i].elements.push_back(classify_element(pipeline.stages));
    if (!pipeline.filter_ok) {
      result.success = false;
    }
  }
  return result;
}

ColorClassifierCore::PipelineResult ColorClassifierCore::run_pipeline(
  const cv::Mat & roi_image) const
{
  PipelineResult result;
  cv::Mat green_filtered_image;
  cv::Mat yellow_filtered_image;
  cv::Mat red_filtered_image;
  result.filter_ok =
    filter_hsv(roi_image, green_filtered_image, yellow_filtered_image, red_filtered_image);
  // binarize
  cv::Mat green_binarized_image;
  cv::Mat yellow_binarized_image;
  cv::Mat red_binarized_image;
  const int bin_threshold = 127;
  cv::threshold(green_filtered_image, green_binarized_image, bin_threshold, 255, cv::THRESH_BINARY);
  cv::threshold(
    yellow_filtered_image, yellow_binarized_image, bin_threshold, 255, cv::THRESH_BINARY);
  cv::threshold(red_filtered_image, red_binarized_image, bin_threshold, 255, cv::THRESH_BINARY);
  // denoise (erode + dilate)
  cv::Mat green_denoised_image;
  cv::Mat yellow_denoised_image;
  cv::Mat red_denoised_image;
  cv::Mat element4 = (cv::Mat_<uchar>(3, 3) << 0, 1, 0, 1, 1, 1, 0, 1, 0);
  cv::erode(green_binarized_image, green_denoised_image, element4, cv::Point(-1, -1), 1);
  cv::erode(yellow_binarized_image, yellow_denoised_image, element4, cv::Point(-1, -1), 1);
  cv::erode(red_binarized_image, red_denoised_image, element4, cv::Point(-1, -1), 1);
  cv::dilate(green_denoised_image, green_denoised_image, cv::Mat(), cv::Point(-1, -1), 1);
  cv::dilate(yellow_denoised_image, yellow_denoised_image, cv::Mat(), cv::Point(-1, -1), 1);
  cv::dilate(red_denoised_image, red_denoised_image, cv::Mat(), cv::Point(-1, -1), 1);

  result.stages.green = {green_filtered_image, green_binarized_image, green_denoised_image};
  result.stages.yellow = {yellow_filtered_image, yellow_binarized_image, yellow_denoised_image};
  result.stages.red = {red_filtered_image, red_binarized_image, red_denoised_image};
  return result;
}

tier4_perception_msgs::msg::TrafficLightElement ColorClassifierCore::classify_element(
  const PipelineStages & stages)
{
  const cv::Mat & green_denoised_image = stages.green.denoised;
  const cv::Mat & yellow_denoised_image = stages.yellow.denoised;
  const cv::Mat & red_denoised_image = stages.red.denoised;

  const int green_pixel_num = cv::countNonZero(green_denoised_image);
  const int yellow_pixel_num = cv::countNonZero(yellow_denoised_image);
  const int red_pixel_num = cv::countNonZero(red_denoised_image);
  const double green_ratio =
    static_cast<double>(green_pixel_num) /
    static_cast<double>(green_denoised_image.rows * green_denoised_image.cols);
  const double yellow_ratio =
    static_cast<double>(yellow_pixel_num) /
    static_cast<double>(yellow_denoised_image.rows * yellow_denoised_image.cols);
  const double red_ratio = static_cast<double>(red_pixel_num) /
                           static_cast<double>(red_denoised_image.rows * red_denoised_image.cols);

  tier4_perception_msgs::msg::TrafficLightElement element;
  if (yellow_ratio < green_ratio && red_ratio < green_ratio) {
    element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
    element.confidence = std::min(1.0, static_cast<double>(green_pixel_num) / (20.0 * 20.0));
  } else if (green_ratio < yellow_ratio && red_ratio < yellow_ratio) {
    element.color = tier4_perception_msgs::msg::TrafficLightElement::AMBER;
    element.confidence = std::min(1.0, static_cast<double>(yellow_pixel_num) / (20.0 * 20.0));
  } else if (green_ratio < red_ratio && yellow_ratio < red_ratio) {
    element.color = ::tier4_perception_msgs::msg::TrafficLightElement::RED;
    element.confidence = std::min(1.0, static_cast<double>(red_pixel_num) / (20.0 * 20.0));
  } else {
    element.color = ::tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
    element.confidence = 0.0;
  }
  return element;
}

cv::Mat ColorClassifierCore::make_debug_image(const cv::Mat & roi_image) const
{
  const PipelineStages stages = run_pipeline(roi_image).stages;

  cv::Mat debug_raw_image;
  cv::Mat debug_green_image;
  cv::Mat debug_yellow_image;
  cv::Mat debug_red_image;
  cv::hconcat(roi_image, roi_image, debug_raw_image);
  cv::hconcat(debug_raw_image, roi_image, debug_raw_image);
  cv::hconcat(stages.green.filtered, stages.green.binarized, debug_green_image);
  cv::hconcat(debug_green_image, stages.green.denoised, debug_green_image);
  cv::hconcat(stages.yellow.filtered, stages.yellow.binarized, debug_yellow_image);
  cv::hconcat(debug_yellow_image, stages.yellow.denoised, debug_yellow_image);
  cv::hconcat(stages.red.filtered, stages.red.binarized, debug_red_image);
  cv::hconcat(debug_red_image, stages.red.denoised, debug_red_image);

  cv::Mat debug_image;
  cv::vconcat(debug_green_image, debug_yellow_image, debug_image);
  cv::vconcat(debug_image, debug_red_image, debug_image);
  cv::cvtColor(debug_image, debug_image, cv::COLOR_GRAY2RGB);
  cv::vconcat(debug_raw_image, debug_image, debug_image);
  const int width = roi_image.cols;
  const int height = roi_image.rows;
  cv::line(
    debug_image, cv::Point(0, 0), cv::Point(debug_image.cols, 0), cv::Scalar(255, 255, 255), 1,
    CV_AA, 0);
  cv::line(
    debug_image, cv::Point(0, height), cv::Point(debug_image.cols, height),
    cv::Scalar(255, 255, 255), 1, CV_AA, 0);
  cv::line(
    debug_image, cv::Point(0, height * 2), cv::Point(debug_image.cols, height * 2),
    cv::Scalar(255, 255, 255), 1, CV_AA, 0);
  cv::line(
    debug_image, cv::Point(0, height * 3), cv::Point(debug_image.cols, height * 3),
    cv::Scalar(255, 255, 255), 1, CV_AA, 0);

  cv::line(
    debug_image, cv::Point(0, 0), cv::Point(0, debug_image.rows), cv::Scalar(255, 255, 255), 1,
    CV_AA, 0);
  cv::line(
    debug_image, cv::Point(width, 0), cv::Point(width, debug_image.rows), cv::Scalar(255, 255, 255),
    1, CV_AA, 0);
  cv::line(
    debug_image, cv::Point(width * 2, 0), cv::Point(width * 2, debug_image.rows),
    cv::Scalar(255, 255, 255), 1, CV_AA, 0);
  cv::line(
    debug_image, cv::Point(width * 3, 0), cv::Point(width * 3, debug_image.rows),
    cv::Scalar(255, 255, 255), 1, CV_AA, 0);

  cv::putText(
    debug_image, "green", cv::Point(0, height * 1.5), cv::FONT_HERSHEY_SIMPLEX, 1.0,
    cv::Scalar(255, 255, 255), 1, CV_AA);
  cv::putText(
    debug_image, "yellow", cv::Point(0, height * 2.5), cv::FONT_HERSHEY_SIMPLEX, 1.0,
    cv::Scalar(255, 255, 255), 1, CV_AA);
  cv::putText(
    debug_image, "red", cv::Point(0, height * 3.5), cv::FONT_HERSHEY_SIMPLEX, 1.0,
    cv::Scalar(255, 255, 255), 1, CV_AA);
  return debug_image;
}

// ============================== ColorClassifier ==============================
// ROS adapter: wires dynamic reconfigure and logging, and delegates classification and
// debug-image rendering to the Node-free core.

namespace
{
// Set `value` from the parameter named `name` if present; return whether it was found.
bool update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, int & value)
{
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == name) {
      value = parameter.as_int();
      return true;
    }
  }
  return false;
}
}  // namespace

ColorClassifier::ColorClassifier(rclcpp::Node * node_ptr, const HSVConfig & config)
: node_ptr_(node_ptr), core_(config)
{
  using std::placeholders::_1;
  // set parameter callback
  set_param_res_ = node_ptr_->add_on_set_parameters_callback(
    std::bind(&ColorClassifier::parametersCallback, this, _1));
}

bool ColorClassifier::getTrafficSignals(
  const std::vector<cv::Mat> & images,
  tier4_perception_msgs::msg::TrafficLightArray & traffic_signals)
{
  if (images.size() != traffic_signals.signals.size()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "image number should be equal to traffic signal number!");
    return false;
  }

  const ColorClassifierCore::ClassifierResult result = core_.classify(images);

  // Attach the core's per-image color elements to the caller's pre-populated
  // signals, preserving the traffic_light_id / traffic_light_type set upstream.
  for (size_t i = 0; i < traffic_signals.signals.size(); i++) {
    auto & elements = traffic_signals.signals[i].elements;
    const auto & classified = result.signals.signals[i].elements;
    elements.insert(elements.end(), classified.begin(), classified.end());
  }

  if (!result.success) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "failed to filter image by hsv value");
  }
  return result.success;
}

cv::Mat ColorClassifier::make_debug_image(const std::vector<cv::Mat> & images) const
{
  // Stack each ROI's mosaic vertically. The core renders RGB-ordered pixels (raw ROI + masks) and
  // re-runs the HSV pipeline, so this stays off the hot path. Per-ROI mosaics differ in width, so
  // later ones are scaled to the first's width before stacking.
  cv::Mat debug_image;
  for (const auto & image : images) {
    cv::Mat mosaic = core_.make_debug_image(image);
    if (!debug_image.empty() && mosaic.cols != debug_image.cols) {
      cv::resize(
        mosaic, mosaic, cv::Size(debug_image.cols, mosaic.rows * debug_image.cols / mosaic.cols));
    }
    if (debug_image.empty()) {
      debug_image = mosaic;
    } else {
      cv::vconcat(debug_image, mosaic, debug_image);
    }
  }
  return debug_image;
}

rcl_interfaces::msg::SetParametersResult ColorClassifier::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  HSVConfig config = core_.get_config();
  update_param(parameters, "green_min_h", config.green_min_h);
  update_param(parameters, "green_min_s", config.green_min_s);
  update_param(parameters, "green_min_v", config.green_min_v);
  update_param(parameters, "green_max_h", config.green_max_h);
  update_param(parameters, "green_max_s", config.green_max_s);
  update_param(parameters, "green_max_v", config.green_max_v);
  update_param(parameters, "yellow_min_h", config.yellow_min_h);
  update_param(parameters, "yellow_min_s", config.yellow_min_s);
  update_param(parameters, "yellow_min_v", config.yellow_min_v);
  update_param(parameters, "yellow_max_h", config.yellow_max_h);
  update_param(parameters, "yellow_max_s", config.yellow_max_s);
  update_param(parameters, "yellow_max_v", config.yellow_max_v);
  update_param(parameters, "red_min_h", config.red_min_h);
  update_param(parameters, "red_min_s", config.red_min_s);
  update_param(parameters, "red_min_v", config.red_min_v);
  update_param(parameters, "red_max_h", config.red_max_h);
  update_param(parameters, "red_max_s", config.red_max_s);
  update_param(parameters, "red_max_v", config.red_max_v);

  core_.set_config(config);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::traffic_light
