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

#include "dummy_traffic_light_publisher_node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::dummy_traffic_light_publisher
{

Mode parse_mode(const std::string & mode_str)
{
  if (mode_str == "standalone") {
    return Mode::Standalone;
  }
  if (mode_str == "empty") {
    return Mode::Empty;
  }
  if (mode_str == "fixed") {
    return Mode::Fixed;
  }
  throw std::invalid_argument(
    "mode must be 'standalone', 'empty' or 'fixed', got: '" + mode_str + "'");
}

uint8_t parse_color(const std::string & color_str)
{
  using Element = autoware_perception_msgs::msg::TrafficLightElement;
  if (color_str == "green") {
    return Element::GREEN;
  }
  if (color_str == "yellow") {
    return Element::AMBER;
  }
  if (color_str == "red") {
    return Element::RED;
  }
  throw std::invalid_argument(
    "fixed_color must be 'green', 'yellow' or 'red', got: '" + color_str + "'");
}

void validate_positive(const std::string & name, double value)
{
  if (value <= 0.0) {
    throw std::invalid_argument(name + " must be positive, got: " + std::to_string(value));
  }
}

DummyTrafficLightPublisherNode::DummyTrafficLightPublisherNode(const rclcpp::NodeOptions & options)
: Node("dummy_traffic_light_publisher", options)
{
  // Parameters
  const auto mode = parse_mode(this->declare_parameter<std::string>("mode"));
  const auto publish_rate = this->declare_parameter<double>("publish_rate");
  validate_positive("publish_rate", publish_rate);
  const auto green_duration = this->declare_parameter<double>("green_duration");
  validate_positive("green_duration", green_duration);
  const auto yellow_duration = this->declare_parameter<double>("yellow_duration");
  validate_positive("yellow_duration", yellow_duration);
  const auto red_duration = this->declare_parameter<double>("red_duration");
  validate_positive("red_duration", red_duration);
  const auto passthrough_timeout = this->declare_parameter<double>("passthrough_timeout");
  validate_positive("passthrough_timeout", passthrough_timeout);
  const auto fixed_color = parse_color(this->declare_parameter<std::string>("fixed_color"));

  // Logic
  dummy_traffic_light_ = std::make_unique<DummyTrafficLight>(
    DummyTrafficLight::Config{mode, passthrough_timeout, fixed_color},
    std::make_unique<TrafficLightCycle>(green_duration, yellow_duration, red_duration));

  // Pub/Sub/Timer (subscriptions use take() in onTimer instead of callbacks)
  pub_ = this->create_publisher<autoware_perception_msgs::msg::TrafficLightGroupArray>(
    "~/output/traffic_signals", rclcpp::QoS(1));

  // use the take method in onTimer, so create dummy subscriptions with empty callbacks to get the
  // subscription objects.
  manual_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = manual_group_;

  sub_vector_map_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    [](autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr) {}, sub_options);

  sub_input_ = this->create_subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>(
    "~/input/traffic_signals", rclcpp::QoS(1),
    [](autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr) {}, sub_options);

  const auto period = rclcpp::Rate(publish_rate).period();
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
    try {
      lanelet::LaneletMapPtr lanelet_map = autoware::experimental::lanelet2_utils::remove_const(
        autoware::experimental::lanelet2_utils::from_autoware_map_msgs(msg));
      lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map);
      std::vector<lanelet::TrafficLightConstPtr> tl_reg_elems =
        lanelet::utils::query::trafficLights(all_lanelets);

      std::vector<int64_t> ids;
      ids.reserve(tl_reg_elems.size());
      for (const auto & tl_reg_elem : tl_reg_elems) {
        ids.push_back(tl_reg_elem->id());
      }
      dummy_traffic_light_->set_traffic_light_ids(ids);

      RCLCPP_INFO(
        this->get_logger(), "Received vector map with %zu traffic lights",
        dummy_traffic_light_->traffic_light_count());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse vector map: %s", e.what());
    }
  }
}

}  // namespace autoware::dummy_traffic_light_publisher
