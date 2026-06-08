// Copyright 2023 The Autoware Contributors
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

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/traffic_light_arbiter/traffic_light_arbiter.hpp>
#include <rclcpp/time.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::traffic_light
{
TrafficLightArbiter::TrafficLightArbiter(const rclcpp::NodeOptions & options)
: Node("traffic_light_arbiter", options)
{
  const auto external_delay_tolerance = this->declare_parameter<double>("external_delay_tolerance");
  const auto external_time_tolerance = this->declare_parameter<double>("external_time_tolerance");
  const auto perception_time_tolerance =
    this->declare_parameter<double>("perception_time_tolerance");

  // Parse source priority parameter
  SourcePriority source_priority;
  const std::string priority_str = this->declare_parameter<std::string>("source_priority");
  if (priority_str == "external") {
    source_priority = SourcePriority::EXTERNAL;
  } else if (priority_str == "perception") {
    source_priority = SourcePriority::PERCEPTION;
  } else if (priority_str == "confidence") {
    source_priority = SourcePriority::CONFIDENCE;
  } else {
    RCLCPP_WARN(
      get_logger(), "Unknown source_priority '%s', defaulting to 'confidence'",
      priority_str.c_str());
    source_priority = SourcePriority::CONFIDENCE;
  }

  const bool enable_signal_matching = this->declare_parameter<bool>("enable_signal_matching");

  core_ = std::make_unique<TrafficLightArbiterCore>(
    source_priority, enable_signal_matching, external_delay_tolerance, external_time_tolerance,
    perception_time_tolerance);

  map_sub_ = create_subscription<LaneletMapBin>(
    "~/sub/vector_map", rclcpp::QoS(1).transient_local(),
    std::bind(&TrafficLightArbiter::on_map, this, std::placeholders::_1));

  perception_tlr_sub_ = create_subscription<TrafficSignalArray>(
    "~/sub/perception_traffic_signals", rclcpp::QoS(1),
    std::bind(&TrafficLightArbiter::on_perception_msg, this, std::placeholders::_1));

  external_tlr_sub_ = create_subscription<TrafficSignalArray>(
    "~/sub/external_traffic_signals", rclcpp::QoS(1),
    std::bind(&TrafficLightArbiter::on_external_msg, this, std::placeholders::_1));

  pub_ = create_publisher<TrafficSignalArray>("~/pub/traffic_signals", rclcpp::QoS(1));
}

void TrafficLightArbiter::on_map(const LaneletMapBin::ConstSharedPtr msg)
{
  core_->set_map(autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));
}

void TrafficLightArbiter::on_perception_msg(const TrafficSignalArray::ConstSharedPtr msg)
{
  log_expired_external_signals(core_->ingest_perception(*msg));
  arbitrate_and_publish(msg->stamp);
}

void TrafficLightArbiter::on_external_msg(const TrafficSignalArray::ConstSharedPtr msg)
{
  const auto result = core_->ingest_external(*msg, this->now());
  if (!result.accepted) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Received outdated external traffic signal messages");
    return;
  }
  log_expired_external_signals(result.expired);
  arbitrate_and_publish(msg->stamp);
}

void TrafficLightArbiter::log_expired_external_signals(
  const std::vector<TrafficLightArbiterCore::ExpiredExternalSignal> & expired)
{
  for (const auto & entry : expired) {
    RCLCPP_DEBUG(
      get_logger(), "Removing expired external traffic light signal (ID: %lu, age: %.2f s)",
      entry.id, entry.age);
  }
}

void TrafficLightArbiter::arbitrate_and_publish(const builtin_interfaces::msg::Time & stamp)
{
  auto result = core_->arbitrate();

  if (!result.output.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Received traffic signal messages before a map");
    return;
  }

  for (const auto & id : result.off_map_signal_ids) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received a traffic signal not present in the current map (%lu)", id);
  }

  // Stamp inheritance is the Node's I/O contract with downstream consumers:
  // the published output carries the trigger msg's stamp for time alignment.
  result.output->stamp = stamp;
  pub_->publish(*result.output);

  if (rclcpp::Time(stamp) < result.latest_input_time) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Published traffic signal messages are not latest");
  }
}
}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightArbiter)
