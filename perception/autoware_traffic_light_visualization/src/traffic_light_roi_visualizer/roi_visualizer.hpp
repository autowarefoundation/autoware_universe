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
#ifndef TRAFFIC_LIGHT_ROI_VISUALIZER__ROI_VISUALIZER_HPP_
#define TRAFFIC_LIGHT_ROI_VISUALIZER__ROI_VISUALIZER_HPP_

#include <opencv2/core.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <string>

namespace autoware::traffic_light
{
struct ClassificationResult
{
  float prob = 0.0;
  std::string label;
};

/**
 * @brief Draws the traffic light detection results onto a camera image.
 *
 * Knows nothing about topics or the package layout: it is handed the messages to draw and the
 * directory the shape icons were installed into, and hands back the image it drew, ready to
 * publish.
 */
class TrafficLightRoiVisualizer
{
public:
  /// @param shape_image_dir Directory holding the shape icons, resolved by the caller.
  explicit TrafficLightRoiVisualizer(std::string shape_image_dir);

  /**
   * @brief Draw the fine detection ROIs onto the camera image, one frame per ROI.
   * @param image Camera image, in any encoding cv_bridge can convert to RGB8.
   * @param rois Fine detection ROIs.
   * @param traffic_signals Classification results, matched to a ROI by traffic light id.
   * @return The RGB8 image with the ROIs drawn on it, ready to publish. Never null.
   * @throws cv_bridge::Exception if `image` cannot be converted to RGB8.
   */
  sensor_msgs::msg::Image::SharedPtr visualize(
    const sensor_msgs::msg::Image & image,
    const tier4_perception_msgs::msg::TrafficLightRoiArray & rois,
    const tier4_perception_msgs::msg::TrafficLightArray & traffic_signals) const;

  /**
   * @brief Draw the rough detection ROIs, and the fine detection ROI of each one that has it.
   *
   * Iterates over the rough ROIs, so a fine ROI whose id no rough ROI mentions is not drawn.
   * @param image Camera image, in any encoding cv_bridge can convert to RGB8.
   * @param rois Fine detection ROIs.
   * @param rough_rois Rough detection ROIs.
   * @param traffic_signals Classification results, matched to a ROI by traffic light id.
   * @return The RGB8 image with the ROIs drawn on it, ready to publish. Never null.
   * @throws cv_bridge::Exception if `image` cannot be converted to RGB8.
   */
  sensor_msgs::msg::Image::SharedPtr visualize_with_rough_rois(
    const sensor_msgs::msg::Image & image,
    const tier4_perception_msgs::msg::TrafficLightRoiArray & rois,
    const tier4_perception_msgs::msg::TrafficLightRoiArray & rough_rois,
    const tier4_perception_msgs::msg::TrafficLightArray & traffic_signals) const;

private:
  /// Draws the ROI in `color` and writes its traffic light id next to it.
  bool draw_roi_with_id(
    cv::Mat & image, const tier4_perception_msgs::msg::TrafficLightRoi & tl_roi,
    const cv::Scalar & color) const;

  /// Draws the ROI in the color of `result` and a label box with its shape and confidence.
  bool draw_roi_with_label(
    cv::Mat & image, const tier4_perception_msgs::msg::TrafficLightRoi & tl_roi,
    const ClassificationResult & result) const;

  /// Where the shape icons live. Held here so that the drawing needs no package lookup.
  std::string shape_image_dir_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_ROI_VISUALIZER__ROI_VISUALIZER_HPP_
