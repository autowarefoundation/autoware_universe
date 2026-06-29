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

#include <memory>

namespace autoware::traffic_light
{

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

  const auto output = selector_.select(
    *detected_traffic_light_msg, *rough_rois_msg, *expected_rois_msg, camera_info_msg->width,
    camera_info_msg->height);

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
