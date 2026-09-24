//  Copyright 2026 The Autoware Contributors
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

#ifndef SIMPLE_SWITCHER__SIMPLE_SWITCHER_NODE_HPP_
#define SIMPLE_SWITCHER__SIMPLE_SWITCHER_NODE_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tier4_system_msgs/msg/active_control_unit.hpp>

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::redundancy_switcher
{

class SimpleSwitcherNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit SimpleSwitcherNode(const rclcpp::NodeOptions & options);

public:
  static uint8_t encode_signals(bool stable, bool self_interrupted, bool faulted);

private:
  void on_manual_active(
    const std_srvs::srv::SetBool::Request & request, std_srvs::srv::SetBool::Response & response);
  void on_self_main();
  void on_self_sub();
  void on_reset();
  void on_priority_main(uint16_t priority);
  void on_priority_sub(uint16_t priority);
  void apply_priority_based_switching();
  void publish_status();

  int64_t main_ecu_id_{0};
  int64_t sub_ecu_id_{1};

  bool main_interrupted_{false};
  bool sub_interrupted_{false};
  uint16_t main_priority_{0};
  uint16_t sub_priority_{0};
  std::vector<uint8_t> active_ids_;
  std::string annotation_;
  std::mutex mutex_;

  AUTOWARE_PUBLISHER_PTR(tier4_system_msgs::msg::ActiveControlUnit) pub_active_;
  AUTOWARE_PUBLISHER_PTR(std_msgs::msg::UInt8) pub_signals_main_;
  AUTOWARE_PUBLISHER_PTR(std_msgs::msg::UInt8) pub_signals_sub_;
  AUTOWARE_PUBLISHER_PTR(std_msgs::msg::String) pub_annotation_main_;
  AUTOWARE_PUBLISHER_PTR(std_msgs::msg::String) pub_annotation_sub_;

  AUTOWARE_SERVICE_PTR(std_srvs::srv::SetBool) srv_manual_active_;
  AUTOWARE_SUBSCRIPTION_PTR(std_msgs::msg::Empty) sub_reset_;
  AUTOWARE_SUBSCRIPTION_PTR(std_msgs::msg::Empty) sub_self_main_;
  AUTOWARE_SUBSCRIPTION_PTR(std_msgs::msg::Empty) sub_self_sub_;
  AUTOWARE_SUBSCRIPTION_PTR(std_msgs::msg::UInt16) sub_priority_main_;
  AUTOWARE_SUBSCRIPTION_PTR(std_msgs::msg::UInt16) sub_priority_sub_;

  AUTOWARE_TIMER_PTR timer_;
};

}  // namespace autoware::redundancy_switcher

#endif  // SIMPLE_SWITCHER__SIMPLE_SWITCHER_NODE_HPP_
