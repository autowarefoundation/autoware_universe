// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__TENSORRT_YOLOX__TENSORRT_YOLOX_NODE_HPP_
#define AUTOWARE__TENSORRT_YOLOX__TENSORRT_YOLOX_NODE_HPP_

#include "autoware/object_recognition_utils/object_recognition_utils.hpp"
#include "autoware/tensorrt_yolox/label.hpp"

#include <autoware/tensorrt_yolox/tensorrt_yolox.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_perception_msgs/msg/semantic.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_yolox
{
// cspell: ignore Semseg
using Label = tier4_perception_msgs::msg::Semantic;

class TrtYoloXNode : public rclcpp::Node
{
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
    bool isOverlay(const uint8_t label) const
    {
      return (label == Label::UNKNOWN && UNKNOWN) || (label == Label::CAR && CAR) ||
             (label == Label::TRUCK && TRUCK) || (label == Label::BUS && BUS) ||
             (label == Label::ANIMAL && ANIMAL) || (label == Label::MOTORBIKE && MOTORCYCLE) ||
             (label == Label::BICYCLE && BICYCLE) || (label == Label::PEDESTRIAN && PEDESTRIAN);
    };
  };  // struct RoiOverlaySemsegLabel

public:
  explicit TrtYoloXNode(const rclcpp::NodeOptions & node_options);

private:
  void onConnect();
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void overlapSegmentByRoi(
    const tensorrt_yolox::Object & object, cv::Mat & mask, const int width, const int height);
  int mapRoiLabel2SegLabel(const int32_t roi_label_index);
  void setupLabel(
    const std::string & roi_label_file_path,
    const std::string & segment_color_map_file_path,
    const std::string & roi_label_remap_file_path,
    const std::string & roi_segment_remap_path);
  void getColorizedMask(const cv::Mat & mask, cv::Mat & cmask);

  image_transport::Publisher image_pub_;
  image_transport::Publisher mask_pub_;
  image_transport::Publisher color_mask_pub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr objects_pub_;

  image_transport::Subscriber image_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tensorrt_yolox::TrtYoloX> trt_yolox_;

  bool is_roi_overlap_semseg_;
  bool is_publish_color_mask_;
  float overlap_roi_score_threshold_;

  // using -1 to represent labels that be ignored
  static constexpr int unmapped_class_id_ = -1;
  std::vector<std::string> roi_id_to_name_map_;
  std::vector<int> roi_id_to_class_id_map_;
  std::vector<int> roi_id_to_semseg_id_map_;

  std::vector<autoware::tensorrt_yolox::Colormap> semseg_color_map_;
  RoiOverlaySemsegLabel roi_overlay_semseg_labels_;

  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;
};

}  // namespace autoware::tensorrt_yolox

#endif  // AUTOWARE__TENSORRT_YOLOX__TENSORRT_YOLOX_NODE_HPP_
