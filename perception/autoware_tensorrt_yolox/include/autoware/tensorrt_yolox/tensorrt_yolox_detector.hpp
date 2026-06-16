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

#ifndef AUTOWARE__TENSORRT_YOLOX__TENSORRT_YOLOX_DETECTOR_HPP_
#define AUTOWARE__TENSORRT_YOLOX__TENSORRT_YOLOX_DETECTOR_HPP_

#include "autoware/tensorrt_yolox/label.hpp"

#include <autoware/tensorrt_yolox/tensorrt_yolox.hpp>
#include <opencv2/opencv.hpp>
#include <tl_expected/expected.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_perception_msgs/msg/semantic.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// cspell: ignore Semseg semseg

namespace autoware::tensorrt_yolox
{
using Label = tier4_perception_msgs::msg::Semantic;

struct RoiOverlaySemsegLabel
{
  bool UNKNOWN;
  bool CAR;
  bool TRUCK;
  bool BUS;
  bool MOTORCYCLE;
  bool BICYCLE;
  bool PEDESTRIAN;
  bool ANIMAL;
  bool HAZARD;

  bool isOverlay(const uint8_t label) const
  {
    return (label == Label::UNKNOWN && UNKNOWN) || (label == Label::CAR && CAR) ||
           (label == Label::TRUCK && TRUCK) || (label == Label::BUS && BUS) ||
           (label == Label::ANIMAL && ANIMAL) || (label == Label::MOTORBIKE && MOTORCYCLE) ||
           (label == Label::BICYCLE && BICYCLE) || (label == Label::PEDESTRIAN && PEDESTRIAN) ||
           (label == Label::HAZARD && HAZARD);
  };
};  // struct RoiOverlaySemsegLabel

/**
 * @struct TrtYoloXDetectorConfig
 * @brief Configuration that does not change frame-by-frame. The detector is reconstructed when any
 * of these values change.
 */
struct TrtYoloXDetectorConfig
{
  // TensorRT engine
  std::string model_path;
  std::string precision;
  float score_threshold;
  float nms_threshold;
  std::string calibration_algorithm;
  int dla_core_id;
  bool quantize_first_layer;
  bool quantize_last_layer;
  bool profile_per_layer;
  double clip_value;
  bool preprocess_on_gpu;
  std::string calibration_image_list_path;
  uint8_t gpu_id;

  // label / remap files
  std::string label_path;
  std::string semseg_color_map_path;
  std::string roi_remap_path;
  std::string roi_to_semseg_remap_path;

  // behavior
  bool is_roi_overlap_semseg;
  bool is_publish_color_mask;
  float overlap_roi_score_threshold;
  RoiOverlaySemsegLabel roi_overlay_semseg_labels;
};

/**
 * @struct TrtYoloXDetectorResult
 * @brief Output of a single inference as ROS messages, ready to publish (headers already stamped).
 * mask/color_mask are present only when the model has a segmentation head (and color mask is
 * enabled), so they are optional.
 */
struct TrtYoloXDetectorResult
{
  tier4_perception_msgs::msg::DetectedObjectsWithFeature objects;
  sensor_msgs::msg::Image image;
  std::optional<sensor_msgs::msg::Image> mask;
  std::optional<sensor_msgs::msg::Image> color_mask;
};

/**
 * @class TrtYoloXDetector
 * @brief Frame-by-frame YOLOX detection pipeline decoupled from rclcpp::Node. It owns the TensorRT
 * inference engine and the label remapping, and converts an input image into the detection result.
 */
class TrtYoloXDetector
{
public:
  explicit TrtYoloXDetector(const TrtYoloXDetectorConfig & config);

  /**
   * @brief whether the underlying GPU has been initialized successfully
   */
  bool isGPUInitialized() const;

  /**
   * @brief run inference and post-process for a single image message
   * @param[in] image_msg input image message (BGR8 expected); its header is propagated to all
   * output messages
   * @return detection result on success, or an error message describing why the operation failed
   * (input image conversion failure or inference failure)
   */
  tl::expected<TrtYoloXDetectorResult, std::string> detect(
    const sensor_msgs::msg::Image & image_msg);

private:
  void setupLabel(
    const std::string & roi_label_path, const std::string & semseg_color_map_path,
    const std::string & roi_label_remap_path, const std::string & roi_to_semseg_remap_path);
  int mapRoiLabel2SegLabel(const int32_t roi_label_index);
  void overlapSegmentByRoi(
    const tensorrt_yolox::Object & object, cv::Mat & mask, const int width, const int height);
  void getColorizedMask(const cv::Mat & mask, cv::Mat & cmask);

  std::unique_ptr<tensorrt_yolox::TrtYoloX> trt_yolox_;
  TrtYoloXDetectorConfig config_;

  // using -1 to represent labels that be ignored
  static constexpr int unmapped_class_id_ = -1;
  std::vector<std::string> roi_class_name_list_;
  std::vector<int> roi_id_to_class_id_map_;
  std::vector<int> roi_id_to_semseg_id_map_;
  std::vector<autoware::tensorrt_yolox::Colormap> semseg_color_map_;
};

}  // namespace autoware::tensorrt_yolox

#endif  // AUTOWARE__TENSORRT_YOLOX__TENSORRT_YOLOX_DETECTOR_HPP_
