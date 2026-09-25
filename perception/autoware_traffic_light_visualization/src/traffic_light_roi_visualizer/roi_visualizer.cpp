// Copyright 2020 Tier IV, Inc.
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

#include "roi_visualizer.hpp"

#include "shape_draw.hpp"

#include <opencv2/imgproc.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>  // for ROS 2 Jazzy or newer
#else
#include <cv_bridge/cv_bridge.h>  // for ROS 2 Humble or older
#endif
#include <opencv2/imgproc/imgproc_c.h>

#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{
namespace
{
/// Parsed form of a label string.
struct TrafficLightShapeInfo
{
  cv::Scalar color;                 //!< Color associated with "circle".
  std::vector<std::string> shapes;  //!< Shape names.
};

// The word a TrafficLightElement code contributes to a label. A code that is not listed yields an
// empty string, as the std::map this replaces did through operator[] - except that operator[] also
// inserted the empty entry into the map, which made the lookup mutate the node's state.
std::string state_to_label(int state)
{
  using tier4_perception_msgs::msg::TrafficLightElement;
  static const std::map<int, std::string> table{
    // color
    {TrafficLightElement::RED, "red"},
    {TrafficLightElement::AMBER, "yellow"},
    {TrafficLightElement::GREEN, "green"},
    {TrafficLightElement::WHITE, "white"},
    // shape
    {TrafficLightElement::CIRCLE, "circle"},
    {TrafficLightElement::LEFT_ARROW, "left"},
    {TrafficLightElement::RIGHT_ARROW, "right"},
    {TrafficLightElement::UP_ARROW, "straight"},
    {TrafficLightElement::DOWN_ARROW, "down"},
    {TrafficLightElement::UP_LEFT_ARROW, "straight_left"},
    {TrafficLightElement::UP_RIGHT_ARROW, "straight_right"},
    {TrafficLightElement::DOWN_LEFT_ARROW, "down_left"},
    {TrafficLightElement::DOWN_RIGHT_ARROW, "down_right"},
    {TrafficLightElement::CROSS, "cross"},
    // other
    {TrafficLightElement::UNKNOWN, "unknown"},
  };

  const auto found = table.find(state);
  return found == table.end() ? std::string{} : found->second;
}

/**
 * @brief Return RGB color from color string associated with "circle".
 * @param color Color string.
 * @return RGB color.
 */
cv::Scalar str_to_color(const std::string & color)
{
  if (color == "red") {
    return {254, 149, 149};
  } else if (color == "yellow") {
    return {254, 250, 149};
  } else if (color == "green") {
    return {149, 254, 161};
  } else {
    return {250, 250, 250};
  }
}

/**
 * @brief Extract color and shape names from label.
 * @param label String formatted as `<Color0>-<Shape0>,<Color1>-<Shape1>,...,<ColorN>-<ShapeN>`.
 * @return Extracted information includes a color associated with "circle" and shape names.
 */
TrafficLightShapeInfo extract_shape_info(const std::string & label)
{
  cv::Scalar color{255, 255, 255};
  std::vector<std::string> shapes;

  std::stringstream ss(label);
  std::string segment;
  while (std::getline(ss, segment, ',')) {
    size_t hyphen_pos = segment.find('-');
    if (hyphen_pos != std::string::npos) {
      auto shape = segment.substr(hyphen_pos + 1);
      if (shape == "circle") {
        const auto color_str = segment.substr(0, hyphen_pos);
        color = str_to_color(color_str);
      }
      shapes.emplace_back(shape);
    }
  }
  return {color, shapes};
}

bool get_classification_result(
  int id, const tier4_perception_msgs::msg::TrafficLightArray & traffic_signals,
  ClassificationResult & result)
{
  bool has_correspond_traffic_signal = false;
  for (const auto & traffic_signal : traffic_signals.signals) {
    if (id != traffic_signal.traffic_light_id) {
      continue;
    }
    has_correspond_traffic_signal = true;
    for (size_t i = 0; i < traffic_signal.elements.size(); i++) {
      auto element = traffic_signal.elements.at(i);
      // all lamp confidence are the same
      result.prob = element.confidence;
      result.label += (state_to_label(element.color) + "-" + state_to_label(element.shape));
      if (i < traffic_signal.elements.size() - 1) {
        result.label += ",";
      }
    }
  }
  return has_correspond_traffic_signal;
}

bool get_roi_from_id(
  int id, const tier4_perception_msgs::msg::TrafficLightRoiArray & rois,
  tier4_perception_msgs::msg::TrafficLightRoi & correspond_roi)
{
  for (const auto roi : rois.rois) {
    if (roi.traffic_light_id == id) {
      correspond_roi = roi;
      return true;
    }
  }
  return false;
}
}  // namespace

TrafficLightRoiVisualizer::TrafficLightRoiVisualizer(std::string shape_image_dir)
: shape_image_dir_(std::move(shape_image_dir))
{
}

bool TrafficLightRoiVisualizer::draw_roi_with_id(
  cv::Mat & image, const tier4_perception_msgs::msg::TrafficLightRoi & tl_roi,
  const cv::Scalar & color) const
{
  cv::rectangle(
    image, cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset),
    cv::Point(tl_roi.roi.x_offset + tl_roi.roi.width, tl_roi.roi.y_offset + tl_roi.roi.height),
    color, 3);
  cv::putText(
    image, std::to_string(tl_roi.traffic_light_id),
    cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset), cv::FONT_HERSHEY_COMPLEX, 1.0, color, 1,
    CV_AA);
  return true;
}

bool TrafficLightRoiVisualizer::draw_roi_with_label(
  cv::Mat & image, const tier4_perception_msgs::msg::TrafficLightRoi & tl_roi,
  const ClassificationResult & result) const
{
  const auto info = extract_shape_info(result.label);

  cv::rectangle(
    image, cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset),
    cv::Point(tl_roi.roi.x_offset + tl_roi.roi.width, tl_roi.roi.y_offset + tl_roi.roi.height),
    info.color, 2);

  constexpr int shape_img_size = 16;
  const auto position = cv::Point(tl_roi.roi.x_offset, tl_roi.roi.y_offset);

  visualization::draw_traffic_light_shape(
    image, shape_image_dir_, info.shapes, shape_img_size, position, info.color, result.prob);

  return true;
}

sensor_msgs::msg::Image::SharedPtr TrafficLightRoiVisualizer::visualize(
  const sensor_msgs::msg::Image & image,
  const tier4_perception_msgs::msg::TrafficLightRoiArray & rois,
  const tier4_perception_msgs::msg::TrafficLightArray & traffic_signals) const
{
  // Convert to RGB8 from whatever the camera sent, since the drawing below only handles RGB8.
  const auto cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);

  for (auto tl_roi : rois.rois) {
    ClassificationResult result;
    bool has_correspond_traffic_signal =
      get_classification_result(tl_roi.traffic_light_id, traffic_signals, result);

    if (!has_correspond_traffic_signal) {
      // does not have classification result
      draw_roi_with_id(cv_ptr->image, tl_roi, cv::Scalar(255, 255, 255));
    } else {
      // has classification result
      draw_roi_with_label(cv_ptr->image, tl_roi, result);
    }
  }

  return cv_ptr->toImageMsg();
}

sensor_msgs::msg::Image::SharedPtr TrafficLightRoiVisualizer::visualize_with_rough_rois(
  const sensor_msgs::msg::Image & image,
  const tier4_perception_msgs::msg::TrafficLightRoiArray & rois,
  const tier4_perception_msgs::msg::TrafficLightRoiArray & rough_rois,
  const tier4_perception_msgs::msg::TrafficLightArray & traffic_signals) const
{
  // Convert to RGB8 from whatever the camera sent, since the drawing below only handles RGB8.
  const auto cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);

  for (auto tl_rough_roi : rough_rois.rois) {
    // note: a signal will still be output even if it is undetected
    // Its position and size will be set as 0 and the color will be set as unknown
    // So a rough roi will always have correspond roi a correspond traffic signal
    ClassificationResult result;
    bool has_correspond_traffic_signal =
      get_classification_result(tl_rough_roi.traffic_light_id, traffic_signals, result);
    tier4_perception_msgs::msg::TrafficLightRoi tl_roi;
    bool has_correspond_roi = get_roi_from_id(tl_rough_roi.traffic_light_id, rois, tl_roi);

    draw_roi_with_id(cv_ptr->image, tl_rough_roi, extract_shape_info(result.label).color);

    if (has_correspond_roi && has_correspond_traffic_signal) {
      // has fine detection and classification results
      draw_roi_with_label(cv_ptr->image, tl_roi, result);
    } else if (has_correspond_roi && !has_correspond_traffic_signal) {
      // has fine detection result and does not have classification result
      draw_roi_with_id(cv_ptr->image, tl_roi, cv::Scalar(255, 255, 255));
    } else if (!has_correspond_roi && has_correspond_traffic_signal) {
      // does not have fine detection result and has classification result
      draw_roi_with_label(cv_ptr->image, tl_rough_roi, result);
    } else {
    }
  }

  return cv_ptr->toImageMsg();
}

}  // namespace autoware::traffic_light
