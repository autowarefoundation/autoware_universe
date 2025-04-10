// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
// governing permissions and limitations under the License.

#include "topic_relay_controller_node.hpp"

#include <memory>
#include <string>

namespace autoware::topic_relay_controller
{
TopicRelayController::TopicRelayController(const rclcpp::NodeOptions & options)
: Node("topic_relay_controller", options), is_relaying_(true)
{
  // Parameter
  node_param_.topic = declare_parameter<std::string>("topic");
  node_param_.remap_topic = declare_parameter<std::string>("remap_topic");
  node_param_.qos_depth = declare_parameter<int>("qos_depth", 1);
  node_param_.transient_local = declare_parameter<bool>("transient_local", false);
  node_param_.best_effort = declare_parameter<bool>("best_effort", false);
  node_param_.is_transform = (node_param_.topic == "/tf" || node_param_.topic == "/tf_static");
  node_param_.enable_relay_control = declare_parameter<bool>("enable_relay_control");
  if (node_param_.enable_relay_control)
    node_param_.srv_name = declare_parameter<std::string>("srv_name");
  node_param_.enable_keep_publishing = declare_parameter<bool>("enable_keep_publishing");
  if (node_param_.enable_keep_publishing)
    node_param_.update_rate = declare_parameter<int>("update_rate");

  if (node_param_.is_transform) {
    node_param_.frame_id = declare_parameter<std::string>("frame_id");
    node_param_.child_frame_id = declare_parameter<std::string>("child_frame_id");
  } else {
    node_param_.topic_type = declare_parameter<std::string>("topic_type");
  }

  // Service
  if (node_param_.enable_relay_control) {
    srv_change_relay_control_ = create_service<tier4_system_msgs::srv::ChangeTopicRelayControl>(
      node_param_.srv_name,
      [this](
        const tier4_system_msgs::srv::ChangeTopicRelayControl::Request::SharedPtr request,
        tier4_system_msgs::srv::ChangeTopicRelayControl::Response::SharedPtr response) {
        is_relaying_ = request->relay_on;
        RCLCPP_INFO(get_logger(), "relay control: %s", is_relaying_ ? "ON" : "OFF");
        response->status.success = true;
      });
  }

  // Subscriber
  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();
  if (node_param_.qos_depth > 0) {
    size_t qos_depth = static_cast<size_t>(node_param_.qos_depth);
    qos.keep_last(qos_depth);
  } else {
    RCLCPP_ERROR(get_logger(), "qos_depth must be greater than 0");
    return;
  }

  if (node_param_.transient_local) {
    qos.transient_local();
  }
  if (node_param_.best_effort) {
    qos.best_effort();
  }

  if (node_param_.is_transform) {
    // Publisher
    pub_transform_ = this->create_publisher<tf2_msgs::msg::TFMessage>(node_param_.remap_topic, qos);

    sub_transform_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      node_param_.topic, qos, [this](tf2_msgs::msg::TFMessage::SharedPtr msg) {
        for (const auto & transform : msg->transforms) {
          if (
            transform.header.frame_id != node_param_.frame_id ||
            transform.child_frame_id != node_param_.child_frame_id || !is_relaying_)
            return;

          if (node_param_.enable_keep_publishing) {
            last_tf_topic_ = msg;
          } else {
            pub_transform_->publish(*msg);
          }
        }
      });
  } else {
    // Publisher
    pub_topic_ =
      this->create_generic_publisher(node_param_.remap_topic, node_param_.topic_type, qos);

    sub_topic_ = this->create_generic_subscription(
      node_param_.topic, node_param_.topic_type, qos,
      [this]([[maybe_unused]] std::shared_ptr<rclcpp::SerializedMessage> msg) {
        if (!is_relaying_) return;

        if (node_param_.enable_keep_publishing) {
          last_topic_ = msg;
        } else {
          pub_topic_->publish(*msg);
        }
      });
  }

  // Timer
  if (node_param_.enable_keep_publishing) {
    const auto update_period_ns = rclcpp::Rate(node_param_.update_rate).period();
    timer_ = rclcpp::create_timer(this, get_clock(), update_period_ns, [this]() {
      if (!is_relaying_) return;

      if (node_param_.is_transform) {
        if (last_tf_topic_) pub_transform_->publish(*last_tf_topic_);
      } else {
        if (last_topic_) pub_topic_->publish(*last_topic_);
      }
    });
  }
}
}  // namespace autoware::topic_relay_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::topic_relay_controller::TopicRelayController)
