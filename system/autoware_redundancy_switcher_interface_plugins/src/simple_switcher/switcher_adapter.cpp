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

#include "switcher_adapter.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <stdexcept>
#include <string>
#include <variant>
#include <redundancy_switcher_interface/detail/overloaded.hpp>

namespace autoware::redundancy_switcher
{

namespace
{
constexpr char kResetTopic[] = "/system/simple_switcher/request/reset";
constexpr char kSelfMainTopic[] = "/system/simple_switcher/request/self_interruption/main_ecu";
constexpr char kSelfSubTopic[] = "/system/simple_switcher/request/self_interruption/sub_ecu";
constexpr char kActiveTopic[] = "/system/simple_switcher/status/active_control_unit";
constexpr char kSignalsMainTopic[] = "/system/simple_switcher/status/switcher_signals/main_ecu";
constexpr char kSignalsSubTopic[] = "/system/simple_switcher/status/switcher_signals/sub_ecu";
constexpr char kAnnotationMainTopic[] = "/system/simple_switcher/status/switcher_annotation/main_ecu";
constexpr char kAnnotationSubTopic[] = "/system/simple_switcher/status/switcher_annotation/sub_ecu";
}  // namespace

void SimpleSwitcherAdapter::initialize(rclcpp::Node * node, std::shared_ptr<EventGateway> gateway)
{
  if (!node) throw std::invalid_argument("SimpleSwitcherAdapter: node is null");
  if (!gateway) throw std::invalid_argument("SimpleSwitcherAdapter: gateway is null");

  node_ = node;
  gateway_ = gateway;

  is_main_ecu_ = node_->has_parameter("is_main_ecu")
    ? node_->get_parameter("is_main_ecu").as_bool()
    : node_->declare_parameter<bool>("is_main_ecu", true);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  const auto signals_topic = is_main_ecu_ ? kSignalsMainTopic : kSignalsSubTopic;
  const auto annotation_topic = is_main_ecu_ ? kAnnotationMainTopic : kAnnotationSubTopic;

  pub_reset_ = node_->create_publisher<EmptyMsg>(kResetTopic, qos);
  pub_self_main_ = node_->create_publisher<EmptyMsg>(kSelfMainTopic, qos);
  pub_self_sub_ = node_->create_publisher<EmptyMsg>(kSelfSubTopic, qos);

  sub_active_control_unit_ = node_->create_subscription<ActiveControlUnitMsg>(
    kActiveTopic, qos, [this](const ActiveControlUnitMsg::ConstSharedPtr msg) {
      on_active_control_unit(*msg);
    });

  sub_switcher_signals_ = node_->create_subscription<UInt8Msg>(
    signals_topic, qos, [this](const UInt8Msg::ConstSharedPtr msg) {
      on_switcher_signals(*msg);
    });

  sub_switcher_annotation_ = node_->create_subscription<StringMsg>(
    annotation_topic, qos, [this](const StringMsg::ConstSharedPtr msg) {
      on_switcher_annotation(*msg);
    });

  RCLCPP_INFO(
    node_->get_logger(),
    "SimpleSwitcherAdapter initialized: is_main_ecu=%s, reset=%s, self_main=%s, self_sub=%s, "
    "signals=%s, annotation=%s",
    is_main_ecu_ ? "true" : "false", kResetTopic, kSelfMainTopic, kSelfSubTopic, signals_topic,
    annotation_topic);
}

void SimpleSwitcherAdapter::execute(const OutputCommand & command)
{
  std::visit(
    overloaded{
      [this](const ResetCommand &) {
        pub_reset_->publish(EmptyMsg{});
      },
      [this](const SelfInterruptionCommand &) {
        if (is_main_ecu_) {
          pub_self_main_->publish(EmptyMsg{});
        } else {
          pub_self_sub_->publish(EmptyMsg{});
        }
      },
      [](const auto &) {}},
    command);
}

SwitcherSignals SimpleSwitcherAdapter::decode_signals(uint8_t encoded)
{
  return SwitcherSignals{
    static_cast<bool>(encoded & 0x01),
    static_cast<bool>(encoded & 0x02),
    static_cast<bool>(encoded & 0x04)};
}

void SimpleSwitcherAdapter::on_active_control_unit(const ActiveControlUnitMsg & msg)
{
  gateway_->submit(InputEvent{SetActiveControlUnitEvent{
    Annotated<ActiveControlUnit>{ActiveControlUnit{msg.ids}, "simple_switcher topic"}}});
}

void SimpleSwitcherAdapter::on_switcher_signals(const UInt8Msg & msg)
{
  std::string annotation;
  {
    std::lock_guard<std::mutex> lock(annotation_mutex_);
    annotation = latest_annotation_;
  }

  gateway_->submit(InputEvent{SetSwitcherSignalsEvent{
    Annotated<SwitcherSignals>{decode_signals(msg.data), annotation}}});
}

void SimpleSwitcherAdapter::on_switcher_annotation(const StringMsg & msg)
{
  std::lock_guard<std::mutex> lock(annotation_mutex_);
  latest_annotation_ = msg.data;
}

}  // namespace autoware::redundancy_switcher

PLUGINLIB_EXPORT_CLASS(
  autoware::redundancy_switcher::SimpleSwitcherAdapter, autoware::redundancy_switcher::IAdapterPlugin)
