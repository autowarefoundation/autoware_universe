// Copyright 2025 TIER IV, Inc.
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

#include "traffic_light_selector_node.hpp"

#include <sensor_msgs/msg/region_of_interest.hpp>

#include <map>
#include <memory>
#include <vector>

namespace autoware::traffic_light
{
using sensor_msgs::msg::RegionOfInterest;

TrafficLightSelectorNode::TrafficLightSelectorNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("traffic_light_selector_node", node_options),
  detected_rois_sub_(this, "input/detected_rois", rclcpp::QoS{1}.get_rmw_qos_profile()),
  rough_rois_sub_(this, "input/rough_rois", rclcpp::QoS{1}.get_rmw_qos_profile()),
  expected_rois_sub_(this, "input/expect_rois", rclcpp::QoS{1}.get_rmw_qos_profile()),
  camera_info_sub_(this, "input/camera_info", rclcpp::SensorDataQoS().get_rmw_qos_profile()),
  sync_(SyncPolicy(10), detected_rois_sub_, rough_rois_sub_, expected_rois_sub_, camera_info_sub_)
{
  {
    stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<autoware_utils::DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  max_iou_threshold_ = declare_parameter<double>("max_iou_threshold");
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  sync_.registerCallback(
    std::bind(&TrafficLightSelectorNode::objectsCallback, this, _1, _2, _3, _4));

  // Publisher
  pub_traffic_light_rois_ =
    create_publisher<TrafficLightRoiArray>("output/traffic_rois", rclcpp::QoS{1});
}

void TrafficLightSelectorNode::objectsCallback(
  const DetectedObjectsWithFeature::ConstSharedPtr & detected_traffic_light_msg,
  const TrafficLightRoiArray::ConstSharedPtr & rough_rois_msg,
  const TrafficLightRoiArray::ConstSharedPtr & expected_rois_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg)
{
  stop_watch_ptr_->toc("processing_time", true);

  const auto image_width = camera_info_msg->width;
  const auto image_height = camera_info_msg->height;

  std::map<uint8_t, RegionOfInterest> rough_rois_map;
  for (const auto & roi : rough_rois_msg->rois) {
    rough_rois_map[roi.traffic_light_id] = roi.roi;
  }

  std::vector<RegionOfInterest> detected_rois;
  for (const auto & detected_objects : detected_traffic_light_msg->feature_objects) {
    detected_rois.push_back(detected_objects.feature.roi);
  }

  // TODO(badai-nguyen): implement this function on CUDA or refactor the code
  TrafficLightRoiArray output;
  output.header = detected_traffic_light_msg->header;
  double max_matching_score = 0.0;
  int32_t det_roi_shift_x = 0;
  int32_t det_roi_shift_y = 0;

  for (const auto & expected_rois_msg_rois : expected_rois_msg->rois) {
    const auto traffic_light_id = expected_rois_msg_rois.traffic_light_id;
    const auto & rough_roi = rough_rois_map[traffic_light_id];
    const auto & expect_roi = expected_rois_msg_rois.roi;

    for (const auto & detected_roi : detected_rois) {
      if (!utils::isInsideRoughRoi(detected_roi, rough_roi)) {
        continue;
      }

      // shift expect roi because the location of detected roi is better that it
      int32_t shift_x, shift_y;
      const auto expect_roi_shifted =
        utils::getShiftedRoi(expect_roi, detected_roi, shift_x, shift_y);

      const double iou = utils::getIoU(expect_roi_shifted, detected_roi);
      if (iou > max_matching_score) {
        max_matching_score = iou;
        det_roi_shift_x = shift_x;
        det_roi_shift_y = shift_y;
      }
    }
  }

  for (const auto & expected_rois_msg_rois : expected_rois_msg->rois) {
    const auto traffic_light_id = expected_rois_msg_rois.traffic_light_id;
    const auto traffic_light_type = expected_rois_msg_rois.traffic_light_type;
    const auto & expect_roi = expected_rois_msg_rois.roi;

    // shift expect_roi by det_roi_shift_x, det_roi_shift_y
    RegionOfInterest expect_roi_shifted = expect_roi;
    expect_roi_shifted.x_offset = std::clamp(
      static_cast<int32_t>(expect_roi_shifted.x_offset) - det_roi_shift_x, 0,
      static_cast<int32_t>(image_width - expect_roi_shifted.width));
    expect_roi_shifted.y_offset = std::clamp(
      static_cast<int32_t>(expect_roi_shifted.y_offset) - det_roi_shift_y, 0,
      static_cast<int32_t>(image_height - expect_roi_shifted.height));

    // check max IoU after shift
    double max_iou = -1.0;
    RegionOfInterest max_iou_roi;
    for (const auto & detected_objects : detected_traffic_light_msg->feature_objects) {
      double iou = utils::getGenIoU(expect_roi_shifted, detected_objects.feature.roi);
      if (iou > max_iou) {
        max_iou = iou;
        max_iou_roi = detected_objects.feature.roi;
      }
    }

    if (max_iou > max_iou_threshold_) {
      TrafficLightRoi traffic_light_roi;
      traffic_light_roi.traffic_light_id = traffic_light_id;
      traffic_light_roi.traffic_light_type = traffic_light_type;
      traffic_light_roi.roi = max_iou_roi;
      output.rois.push_back(traffic_light_roi);
    } else {
      TrafficLightRoi traffic_light_roi;
      traffic_light_roi.traffic_light_id = traffic_light_id;
      traffic_light_roi.traffic_light_type = traffic_light_type;
      output.rois.push_back(traffic_light_roi);
    }
  }

  pub_traffic_light_rois_->publish(output);

  if (debug_publisher_) {
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - output.header.stamp).nanoseconds()))
        .count();
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }
}
}  // namespace autoware::traffic_light

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightSelectorNode)
