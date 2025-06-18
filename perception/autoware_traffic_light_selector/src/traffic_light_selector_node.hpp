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

#ifndef TRAFFIC_LIGHT_SELECTOR_NODE_HPP_
#define TRAFFIC_LIGHT_SELECTOR_NODE_HPP_

#include "traffic_light_selector_utils.hpp"

#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/detected_object_with_feature.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <map>
#include <memory>
#include <vector>

namespace autoware::traffic_light
{
using sensor_msgs::msg::RegionOfInterest;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;
using tier4_perception_msgs::msg::TrafficLightRoi;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

class TrafficLightSelectorNode : public rclcpp::Node
{
public:
  explicit TrafficLightSelectorNode(const rclcpp::NodeOptions & node_options);

private:
  // Subscriber
  message_filters::Subscriber<DetectedObjectsWithFeature> detected_rois_sub_;
  message_filters::Subscriber<TrafficLightRoiArray> rough_rois_sub_;
  message_filters::Subscriber<TrafficLightRoiArray> expected_rois_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
  typedef message_filters::sync_policies::ApproximateTime<
    DetectedObjectsWithFeature, TrafficLightRoiArray, TrafficLightRoiArray,
    sensor_msgs::msg::CameraInfo>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync sync_;

  // Publisher
  rclcpp::Publisher<TrafficLightRoiArray>::SharedPtr pub_traffic_light_rois_;
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;

  void objectsCallback(
    const DetectedObjectsWithFeature::ConstSharedPtr & detected_rois_msg,
    const TrafficLightRoiArray::ConstSharedPtr & rough_rois_msg,
    const TrafficLightRoiArray::ConstSharedPtr & expect_rois_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg);

  void evaluateWholeRois(
    const std::vector<RegionOfInterest> & detected_rois,
    const std::map<int64_t, RegionOfInterest> & expect_rois_shifted_map, double & total_max_iou,
    std::map<int64_t, RegionOfInterest> & total_max_iou_rois_map);
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_SELECTOR_NODE_HPP_
