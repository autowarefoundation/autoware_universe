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

#include "simple_switcher_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <utility>

namespace autoware::redundancy_switcher
{

constexpr char kManualActiveService[] = "/system/simple_switcher/input/manual_active_control_unit";
constexpr char kResetRequestTopic[] = "/system/simple_switcher/request/reset";
constexpr char kSelfInterruptionMainTopic[] =
  "/system/simple_switcher/request/self_interruption/main_ecu";
constexpr char kSelfInterruptionSubTopic[] =
  "/system/simple_switcher/request/self_interruption/sub_ecu";
constexpr char kActiveStatusTopic[] = "/system/simple_switcher/status/active_control_unit";
constexpr char kSignalsStatusMainTopic[] =
  "/system/simple_switcher/status/switcher_signals/main_ecu";
constexpr char kSignalsStatusSubTopic[] = "/system/simple_switcher/status/switcher_signals/sub_ecu";
constexpr char kAnnotationStatusMainTopic[] =
  "/system/simple_switcher/status/switcher_annotation/main_ecu";
constexpr char kAnnotationStatusSubTopic[] =
  "/system/simple_switcher/status/switcher_annotation/sub_ecu";

SimpleSwitcherNode::SimpleSwitcherNode(const rclcpp::NodeOptions & options)
: Node("simple_switcher_node", options)
{
  main_ecu_id_ = declare_parameter<int64_t>("main_ecu_id", 0);
  sub_ecu_id_ = declare_parameter<int64_t>("sub_ecu_id", 1);
  const auto publish_period_ms = declare_parameter<int64_t>("publish_period_ms", 200);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  pub_active_ =
    create_publisher<tier4_system_msgs::msg::ActiveControlUnit>(kActiveStatusTopic, qos);
  pub_signals_main_ = create_publisher<std_msgs::msg::UInt8>(kSignalsStatusMainTopic, qos);
  pub_signals_sub_ = create_publisher<std_msgs::msg::UInt8>(kSignalsStatusSubTopic, qos);
  pub_annotation_main_ = create_publisher<std_msgs::msg::String>(kAnnotationStatusMainTopic, qos);
  pub_annotation_sub_ = create_publisher<std_msgs::msg::String>(kAnnotationStatusSubTopic, qos);

  srv_manual_active_ = create_service<std_srvs::srv::SetBool>(
    kManualActiveService, [this](
                            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      on_manual_active(*request, *response);
    });
  sub_reset_ = create_subscription<std_msgs::msg::Empty>(
    kResetRequestTopic, qos, [this](const std_msgs::msg::Empty::ConstSharedPtr) { on_reset(); });
  sub_self_main_ = create_subscription<std_msgs::msg::Empty>(
    kSelfInterruptionMainTopic, qos,
    [this](const std_msgs::msg::Empty::ConstSharedPtr) { on_self_main(); });
  sub_self_sub_ = create_subscription<std_msgs::msg::Empty>(
    kSelfInterruptionSubTopic, qos,
    [this](const std_msgs::msg::Empty::ConstSharedPtr) { on_self_sub(); });

  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ids_ = {static_cast<uint8_t>(main_ecu_id_)};
    annotation_ = "stable active=main";
  }

  timer_ =
    create_wall_timer(std::chrono::milliseconds(publish_period_ms), [this]() { publish_status(); });

  publish_status();

  RCLCPP_INFO(
    get_logger(), "simple_switcher_node started: main_ecu_id=%ld sub_ecu_id=%ld", main_ecu_id_,
    sub_ecu_id_);
}

uint8_t SimpleSwitcherNode::encode_signals(bool stable, bool self_interrupted, bool faulted)
{
  uint8_t v = 0;
  if (stable) v |= 0x01;
  if (self_interrupted) v |= 0x02;
  if (faulted) v |= 0x04;
  return v;
}

void SimpleSwitcherNode::on_manual_active(
  const std_srvs::srv::SetBool::Request & request, std_srvs::srv::SetBool::Response & response)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto requested =
      request.data ? static_cast<uint8_t>(main_ecu_id_) : static_cast<uint8_t>(sub_ecu_id_);

    active_ids_ = {requested};
    if (request.data) {
      main_interrupted_ = false;
      sub_interrupted_ = false;
      annotation_ = "manual override active=main";
      response.success = true;
      response.message = "active=main";
    } else {
      // Switching manually to Sub ECU implies Main ECU is treated as faulted.
      main_interrupted_ = true;
      sub_interrupted_ = false;
      annotation_ = "manual override active=sub main_fault=true";
      response.success = true;
      response.message = "active=sub main_fault=true";
    }
  }
  publish_status();
}

void SimpleSwitcherNode::on_self_main()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    main_interrupted_ = true;
    if (sub_interrupted_) {
      active_ids_.clear();
      annotation_ = "faulted both_self_interrupted";
    } else {
      active_ids_ = {static_cast<uint8_t>(sub_ecu_id_)};
      annotation_ = "self_interrupted by=main failover active=sub";
    }
  }
  publish_status();
}

void SimpleSwitcherNode::on_self_sub()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sub_interrupted_ = true;
    if (main_interrupted_) {
      active_ids_.clear();
      annotation_ = "faulted both_self_interrupted";
    } else {
      active_ids_ = {static_cast<uint8_t>(main_ecu_id_)};
      annotation_ = "self_interrupted by=sub failover active=main";
    }
  }
  publish_status();
}

void SimpleSwitcherNode::on_reset()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    main_interrupted_ = false;
    sub_interrupted_ = false;
    active_ids_ = {static_cast<uint8_t>(main_ecu_id_)};
    annotation_ = "stable active=main (reset)";
  }
  publish_status();
}

void SimpleSwitcherNode::publish_status()
{
  tier4_system_msgs::msg::ActiveControlUnit active_msg;
  std_msgs::msg::UInt8 signals_main_msg;
  std_msgs::msg::UInt8 signals_sub_msg;
  std_msgs::msg::String annotation_main_msg;
  std_msgs::msg::String annotation_sub_msg;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    const bool faulted = main_interrupted_ && sub_interrupted_;
    const bool main_self_interrupted = main_interrupted_ && !faulted;
    const bool sub_self_interrupted = sub_interrupted_ && !faulted;
    const bool main_stable = !main_self_interrupted && !faulted;
    const bool sub_stable = !sub_self_interrupted && !faulted;

    active_msg.ids = active_ids_;
    signals_main_msg.data = encode_signals(main_stable, main_self_interrupted, faulted);
    signals_sub_msg.data = encode_signals(sub_stable, sub_self_interrupted, faulted);

    if (faulted) {
      annotation_main_msg.data = "faulted both_self_interrupted";
      annotation_sub_msg.data = "faulted both_self_interrupted";
    } else if (main_interrupted_ && !sub_interrupted_) {
      annotation_main_msg.data = "self_interrupted by=main";
      annotation_sub_msg.data = annotation_;
    } else if (sub_interrupted_ && !main_interrupted_) {
      annotation_main_msg.data = annotation_;
      annotation_sub_msg.data = "self_interrupted by=sub";
    } else {
      annotation_main_msg.data = annotation_;
      annotation_sub_msg.data = annotation_;
    }
  }

  pub_active_->publish(active_msg);
  pub_signals_main_->publish(signals_main_msg);
  pub_signals_sub_->publish(signals_sub_msg);
  pub_annotation_main_->publish(annotation_main_msg);
  pub_annotation_sub_->publish(annotation_sub_msg);
}

}  // namespace autoware::redundancy_switcher

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::redundancy_switcher::SimpleSwitcherNode)
