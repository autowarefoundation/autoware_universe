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

#ifndef TOPIC_RELAY_CONTROLLER_NODE_HPP_
#define TOPIC_RELAY_CONTROLLER_NODE_HPP_

// ROS 2 core
#include <rclcpp/rclcpp.hpp>

#include <tf2_msgs/msg/tf_message.hpp>
#include <tier4_system_msgs/srv/change_topic_relay_control.hpp>

#include <memory>
#include <string>

namespace autoware::topic_relay_controller
{
struct NodeParam
{
  std::string topic;
  std::string remap_topic;
  std::string topic_type;
  int qos_depth;
  std::string frame_id;
  std::string child_frame_id;
  bool transient_local;
  bool best_effort;
  bool is_transform;
  bool enable_relay_control;
  std::string srv_name;
  bool enable_keep_publishing;
  int update_rate;
};

class TopicRelayController : public rclcpp::Node
{
public:
  explicit TopicRelayController(const rclcpp::NodeOptions & options);

private:
  // Parameter
  NodeParam node_param_;

  // Subscriber
  rclcpp::GenericSubscription::SharedPtr sub_topic_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_transform_;

  // Publisher
  rclcpp::GenericPublisher::SharedPtr pub_topic_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_transform_;

  // Service
  rclcpp::Service<tier4_system_msgs::srv::ChangeTopicRelayControl>::SharedPtr
    srv_change_relay_control_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  bool is_relaying_;
  tf2_msgs::msg::TFMessage::SharedPtr last_tf_topic_;
  std::shared_ptr<rclcpp::SerializedMessage> last_topic_;
};
}  // namespace autoware::topic_relay_controller

#endif  // TOPIC_RELAY_CONTROLLER_NODE_HPP_
