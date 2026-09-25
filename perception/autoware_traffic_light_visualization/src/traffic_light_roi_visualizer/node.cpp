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

#include "node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>  // for ROS 2 Jazzy or newer
#else
#include <cv_bridge/cv_bridge.h>  // for ROS 2 Humble or older
#endif

#include <memory>
#include <string>

namespace autoware::traffic_light
{
TrafficLightRoiVisualizerNode::TrafficLightRoiVisualizerNode(const rclcpp::NodeOptions & options)
: Node("traffic_light_roi_visualizer_node", options),
  visualizer_(
    ament_index_cpp::get_package_share_directory("autoware_traffic_light_visualization") +
    "/images/")
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  using std::placeholders::_4;
  use_high_accuracy_detection_ = this->declare_parameter<bool>("use_high_accuracy_detection");
  use_image_transport_ = this->declare_parameter<bool>("use_image_transport");

  if (use_high_accuracy_detection_) {
    sync_with_rough_roi_.reset(new SyncWithRoughRoi(
      SyncPolicyWithRoughRoi(10), image_sub_, roi_sub_, rough_roi_sub_, traffic_signals_sub_));
    sync_with_rough_roi_->registerCallback(
      std::bind(&TrafficLightRoiVisualizerNode::image_rough_roi_callback, this, _1, _2, _3, _4));
  } else {
    sync_.reset(new Sync(SyncPolicy(10), image_sub_, roi_sub_, traffic_signals_sub_));
    sync_->registerCallback(
      std::bind(&TrafficLightRoiVisualizerNode::image_roi_callback, this, _1, _2, _3));
  }

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&TrafficLightRoiVisualizerNode::connect_cb, this));

  if (use_image_transport_) {
    image_pub_ = image_transport::create_publisher(
      this, "~/output/image", rclcpp::QoS{1}.get_rmw_qos_profile());
  } else {
    auto qos = rclcpp::QoS(1);
    simple_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/output/image", qos);
  }
}

void TrafficLightRoiVisualizerNode::connect_cb()
{
  int num_subscribers = 0;
  if (use_image_transport_) {
    num_subscribers = image_pub_.getNumSubscribers();
  } else {
    num_subscribers = simple_image_pub_->get_subscription_count();
  }
  if (num_subscribers == 0) {
    image_sub_.unsubscribe();
    traffic_signals_sub_.unsubscribe();
    roi_sub_.unsubscribe();
    if (use_high_accuracy_detection_) {
      rough_roi_sub_.unsubscribe();
    }
  } else if (!image_sub_.getSubscriber()) {
    image_sub_.subscribe(this, "~/input/image", "raw", rmw_qos_profile_sensor_data);
    roi_sub_.subscribe(this, "~/input/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
    traffic_signals_sub_.subscribe(
      this, "~/input/traffic_signals", rclcpp::QoS{1}.get_rmw_qos_profile());
    if (use_high_accuracy_detection_) {
      rough_roi_sub_.subscribe(this, "~/input/rough/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
    }
  }
}

void TrafficLightRoiVisualizerNode::publish(const sensor_msgs::msg::Image::SharedPtr & drawn) const
{
  if (use_image_transport_) {
    image_pub_.publish(drawn);
  } else {
    simple_image_pub_->publish(*drawn);
  }
}

void TrafficLightRoiVisualizerNode::image_roi_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_roi_msg,
  [[maybe_unused]] const tier4_perception_msgs::msg::TrafficLightArray::ConstSharedPtr &
    input_traffic_signals_msg)
{
  try {
    publish(visualizer_.visualize(*input_image_msg, *input_tl_roi_msg, *input_traffic_signals_msg));
  } catch (cv_bridge::Exception & e) {
    // Nothing was drawn, so there is nothing to publish for this image.
    RCLCPP_ERROR(
      get_logger(), "Could not convert from '%s' to 'rgb8'.", input_image_msg->encoding.c_str());
  }
}

void TrafficLightRoiVisualizerNode::image_rough_roi_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_roi_msg,
  const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_rough_roi_msg,
  const tier4_perception_msgs::msg::TrafficLightArray::ConstSharedPtr & input_traffic_signals_msg)
{
  try {
    publish(visualizer_.visualize_with_rough_rois(
      *input_image_msg, *input_tl_roi_msg, *input_tl_rough_roi_msg, *input_traffic_signals_msg));
  } catch (cv_bridge::Exception & e) {
    // Nothing was drawn, so there is nothing to publish for this image.
    RCLCPP_ERROR(
      get_logger(), "Could not convert from '%s' to 'rgb8'.", input_image_msg->encoding.c_str());
  }
}

}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightRoiVisualizerNode)
