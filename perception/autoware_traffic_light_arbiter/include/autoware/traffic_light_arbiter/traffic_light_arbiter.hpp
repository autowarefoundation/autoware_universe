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

#ifndef AUTOWARE__TRAFFIC_LIGHT_ARBITER__TRAFFIC_LIGHT_ARBITER_HPP_
#define AUTOWARE__TRAFFIC_LIGHT_ARBITER__TRAFFIC_LIGHT_ARBITER_HPP_

#include <autoware/traffic_light_arbiter/traffic_light_arbiter_core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <memory>
#include <vector>

namespace autoware::traffic_light
{

class TrafficLightArbiter : public rclcpp::Node
{
public:
  explicit TrafficLightArbiter(const rclcpp::NodeOptions & options);

private:
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using TrafficSignalArray = autoware_perception_msgs::msg::TrafficLightGroupArray;

  rclcpp::Subscription<LaneletMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<TrafficSignalArray>::SharedPtr perception_tlr_sub_;
  rclcpp::Subscription<TrafficSignalArray>::SharedPtr external_tlr_sub_;
  rclcpp::Publisher<TrafficSignalArray>::SharedPtr pub_;

  void on_map(const LaneletMapBin::ConstSharedPtr msg);
  void on_perception_msg(const TrafficSignalArray::ConstSharedPtr msg);
  void on_external_msg(const TrafficSignalArray::ConstSharedPtr msg);
  void arbitrate_and_publish(const builtin_interfaces::msg::Time & stamp);

  // Emits one DEBUG line per expired entry; called from the on_*_msg
  // handlers with the result of the corresponding ingest_*().
  void log_expired_external_signals(
    const std::vector<TrafficLightArbiterCore::ExpiredExternalSignal> & expired);

  std::unique_ptr<TrafficLightArbiterCore> core_;
};
}  // namespace autoware::traffic_light

#endif  // AUTOWARE__TRAFFIC_LIGHT_ARBITER__TRAFFIC_LIGHT_ARBITER_HPP_
