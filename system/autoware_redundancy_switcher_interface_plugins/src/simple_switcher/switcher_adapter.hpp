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
#ifndef SIMPLE_SWITCHER__SWITCHER_ADAPTER_HPP_
#define SIMPLE_SWITCHER__SWITCHER_ADAPTER_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <redundancy_switcher_interface/plugin/event_gateway.hpp>
#include <redundancy_switcher_interface/plugin/i_adapter_plugin.hpp>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tier4_system_msgs/msg/active_control_unit.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace autoware::redundancy_switcher
{

class SimpleSwitcherAdapter : public IAdapterPlugin
{
public:
  SimpleSwitcherAdapter() = default;
  ~SimpleSwitcherAdapter() override = default;

  void initialize(
    autoware::agnocast_wrapper::Node * node, std::shared_ptr<EventGateway> gateway) override;
  void execute(const OutputCommand & command) override;

private:
  using EmptyMsg = std_msgs::msg::Empty;
  using StringMsg = std_msgs::msg::String;
  using UInt8Msg = std_msgs::msg::UInt8;
  using UInt16Msg = std_msgs::msg::UInt16;
  using ActiveControlUnitMsg = tier4_system_msgs::msg::ActiveControlUnit;

public:
  static SwitcherSignals decode_signals(uint8_t encoded);

private:
  void on_active_control_unit(const ActiveControlUnitMsg & msg);
  void on_switcher_signals(const UInt8Msg & msg);
  void on_switcher_annotation(const StringMsg & msg);

  autoware::agnocast_wrapper::Node * node_{nullptr};
  std::shared_ptr<EventGateway> gateway_;
  bool is_main_ecu_{true};

  AUTOWARE_PUBLISHER_PTR(EmptyMsg) pub_reset_;
  AUTOWARE_PUBLISHER_PTR(EmptyMsg) pub_self_main_;
  AUTOWARE_PUBLISHER_PTR(EmptyMsg) pub_self_sub_;
  AUTOWARE_PUBLISHER_PTR(UInt16Msg) pub_priority_;

  AUTOWARE_SUBSCRIPTION_PTR(ActiveControlUnitMsg) sub_active_control_unit_;
  AUTOWARE_SUBSCRIPTION_PTR(UInt8Msg) sub_switcher_signals_;
  AUTOWARE_SUBSCRIPTION_PTR(StringMsg) sub_switcher_annotation_;

  mutable std::mutex annotation_mutex_;
  std::string latest_annotation_{"simple_switcher: startup"};
};

}  // namespace autoware::redundancy_switcher
#endif  // SIMPLE_SWITCHER__SWITCHER_ADAPTER_HPP_
