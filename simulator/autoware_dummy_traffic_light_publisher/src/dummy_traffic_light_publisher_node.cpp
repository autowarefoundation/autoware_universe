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

#include "autoware/dummy_traffic_light_publisher/dummy_traffic_light_publisher_node.hpp"

namespace autoware::dummy_traffic_light_publisher
{

DummyTrafficLightPublisherNode::DummyTrafficLightPublisherNode(const rclcpp::NodeOptions & options)
: Node("dummy_traffic_light_publisher", options)
{
  const auto mode_str = this->declare_parameter<std::string>("mode", "standalone");
  const auto mode = (mode_str == "standalone") ? Mode::Standalone : Mode::Empty;

  const auto publish_rate = this->declare_parameter<double>("publish_rate", 10.0);
  const auto green_duration = this->declare_parameter<double>("green_duration", 30.0);
  const auto yellow_duration = this->declare_parameter<double>("yellow_duration", 3.0);
  const auto red_duration = this->declare_parameter<double>("red_duration", 30.0);
  const auto passthrough_timeout = this->declare_parameter<double>("passthrough_timeout", 1.0);

  dummy_traffic_light_ = std::make_unique<DummyTrafficLight>(
    DummyTrafficLight::Config{mode, passthrough_timeout},
    std::make_unique<TrafficLightCycle>(green_duration, yellow_duration, red_duration));

  pub_ = this->create_publisher<autoware_perception_msgs::msg::TrafficLightGroupArray>(
    "~/output/traffic_signals", rclcpp::QoS(1));

  // Callback group not automatically added to executor, so take() works without callback interference
  manual_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = manual_group_;

  sub_vector_map_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    [](autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr) {}, sub_options);

  sub_input_ = this->create_subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>(
    "~/input/traffic_signals", rclcpp::QoS(1),
    [](autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr) {}, sub_options);

  const auto period = std::chrono::milliseconds(static_cast<int64_t>(1e3 / publish_rate));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period, std::bind(&DummyTrafficLightPublisherNode::onTimer, this));
}

void DummyTrafficLightPublisherNode::onTimer()
{
  const auto now = this->now();

  // update map if there is a new message
  try_update_vector_map();

  // update traffic light signals and publish
  {
    autoware_perception_msgs::msg::TrafficLightGroupArray msg;
    rclcpp::MessageInfo info;
    if (sub_input_->take(msg, info)) {
      dummy_traffic_light_->update_input_signals(msg, now);
    }
  }
  pub_->publish(dummy_traffic_light_->create_message(now));
}

void DummyTrafficLightPublisherNode::try_update_vector_map()
{
  autoware_map_msgs::msg::LaneletMapBin msg;
  rclcpp::MessageInfo info;
  if (sub_vector_map_->take(msg, info)) {
    dummy_traffic_light_->update_vector_map(msg);
    RCLCPP_INFO(
      this->get_logger(), "Received vector map with %zu traffic lights",
      dummy_traffic_light_->traffic_light_count());
  }
}

}  // namespace autoware::dummy_traffic_light_publisher
