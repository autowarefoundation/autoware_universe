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
#ifndef TRAFFIC_LIGHT_ROI_VISUALIZER__NODE_HPP_
#define TRAFFIC_LIGHT_ROI_VISUALIZER__NODE_HPP_

#include "roi_visualizer.hpp"

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <memory>

namespace autoware::traffic_light
{
class TrafficLightRoiVisualizerNode : public rclcpp::Node
{
public:
  explicit TrafficLightRoiVisualizerNode(const rclcpp::NodeOptions & options);
  void connect_cb();

  void image_roi_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
    const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_roi_msg,
    const tier4_perception_msgs::msg::TrafficLightArray::ConstSharedPtr &
      input_traffic_signals_msg);

  void image_rough_roi_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
    const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_roi_msg,
    const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_rough_roi_msg,
    const tier4_perception_msgs::msg::TrafficLightArray::ConstSharedPtr &
      input_traffic_signals_msg);

private:
  /// Sends the drawn image out through whichever publisher the parameters selected.
  void publish(const sensor_msgs::msg::Image::SharedPtr & drawn) const;

  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<tier4_perception_msgs::msg::TrafficLightRoiArray> roi_sub_;
  message_filters::Subscriber<tier4_perception_msgs::msg::TrafficLightRoiArray> rough_roi_sub_;
  message_filters::Subscriber<tier4_perception_msgs::msg::TrafficLightArray> traffic_signals_sub_;
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr simple_image_pub_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, tier4_perception_msgs::msg::TrafficLightRoiArray,
    tier4_perception_msgs::msg::TrafficLightArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, tier4_perception_msgs::msg::TrafficLightRoiArray,
    tier4_perception_msgs::msg::TrafficLightRoiArray, tier4_perception_msgs::msg::TrafficLightArray>
    SyncPolicyWithRoughRoi;
  typedef message_filters::Synchronizer<SyncPolicyWithRoughRoi> SyncWithRoughRoi;
  std::shared_ptr<SyncWithRoughRoi> sync_with_rough_roi_;

  bool use_high_accuracy_detection_;
  bool use_image_transport_;

  /// Does all of the drawing. Holds no ROS state.
  TrafficLightRoiVisualizer visualizer_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_ROI_VISUALIZER__NODE_HPP_
