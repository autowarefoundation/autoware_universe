//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "autonomous_mode_transition_flag_node.hpp"

#include <memory>
#include <string>

namespace autoware::operation_mode_transition_manager
{

AutonomousModeTransitionFlagNode::AutonomousModeTransitionFlagNode(
  const rclcpp::NodeOptions & options)
: Node("autonomous_mode_transition_flag_node", options)
{
  declare_parameter<double>("stable_check.duration");
  input_timeout_ = declare_parameter<double>("input_timeout");
  autonomous_mode_ = std::make_unique<AutonomousMode>(this);

  pub_transition_available_ =
    create_publisher<ModeChangeAvailable>("/system/command_mode/transition/available", 1);
  pub_transition_completed_ =
    create_publisher<ModeChangeAvailable>("/system/command_mode/transition/completed", 1);

  pub_driving_mode_available_ = create_publisher<DiagnosticArray>("/diagnostics", 1);
  pub_driving_mode_stable_ = create_publisher<DrivingModeFlag>("/system/driving_mode/stable", 1);
  sub_driving_mode_info_ = create_subscription<DrivingModeInfo>(
    "/system/driving_mode/info", rclcpp::QoS(1).transient_local(),
    [this](const DrivingModeInfo & msg) { on_driving_mode_info(msg); });

  pub_debug_ = create_publisher<ModeChangeBase::DebugInfo>("~/debug_info", 1);

  const auto period = rclcpp::Rate(declare_parameter<double>("frequency_hz")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void AutonomousModeTransitionFlagNode::on_timer()
{
  const auto publish = [](auto pub, rclcpp::Time stamp, bool value) {
    ModeChangeAvailable msg;
    msg.stamp = stamp;
    msg.available = value;
    pub->publish(msg);
  };

  const auto input = take_data();
  const auto stamp = get_clock()->now();
  // Before every input has been seen once there is nothing to judge on, and once a
  // publisher stalls its retained value must not keep the answer "yes" alive, so the
  // judgement also requires every input to still be fresh. Either way the answer to
  // "can autonomous run" is no.
  const bool has_fresh_data = has_all_data() && inputs_are_fresh(stamp);
  const bool is_available = has_fresh_data && autonomous_mode_->isModeChangeAvailable(input);
  const bool is_completed = has_fresh_data && autonomous_mode_->isModeChangeCompleted(input);

  publish(pub_transition_available_, stamp, is_available);
  publish(pub_transition_completed_, stamp, is_completed);
  publish_driving_mode_available(is_available);
  publish_driving_mode_stable(is_completed);

  ModeChangeBase::DebugInfo debug = autonomous_mode_->getDebugInfo();
  debug.stamp = stamp;
  pub_debug_->publish(debug);
}

InputData AutonomousModeTransitionFlagNode::take_data()
{
  // Each input keeps its last value: the polling subscribers return nothing on a
  // tick that saw no new message, and the checks read every field, so dropping
  // back to a default-constructed InputData would report a zero pose, an empty
  // trajectory and a zero command -- "not available" -- on every such tick.
  const auto kinematics = sub_kinematics_.take_data();
  if (kinematics) {
    input_data_.kinematics = *kinematics;
    has_kinematics_ = true;
  }

  const auto trajectory = sub_trajectory_.take_data();
  if (trajectory) {
    input_data_.trajectory = *trajectory;
    has_trajectory_ = true;
  }

  const auto control_cmd = sub_control_cmd_.take_data();
  if (control_cmd) {
    input_data_.control_cmd = *control_cmd;
    has_control_cmd_ = true;
  }

  const auto trajectory_follower_control_cmd = sub_trajectory_follower_control_cmd_.take_data();
  if (trajectory_follower_control_cmd) {
    input_data_.trajectory_follower_control_cmd = *trajectory_follower_control_cmd;
    has_trajectory_follower_control_cmd_ = true;
  }

  return input_data_;
}

bool AutonomousModeTransitionFlagNode::has_all_data() const
{
  return has_kinematics_ && has_trajectory_ && has_control_cmd_ &&
         has_trajectory_follower_control_cmd_;
}

bool AutonomousModeTransitionFlagNode::inputs_are_fresh(const rclcpp::Time & now) const
{
  // Mirrors OperationModeTransitionManager::subscribeData timeout handling, but here
  // it gates the retained values (this node keeps the last message across ticks) so a
  // stalled or exited publisher stops counting once its timestamp ages past the limit.
  const auto timed_out = [&](const char * name, const auto & stamp) {
    const bool stale = input_timeout_ < (now - rclcpp::Time(stamp)).seconds();
    if (stale) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Retained %s is timed out.", name);
    }
    return stale;
  };
  // has_all_data() guarantees each optional holds a value before this runs. Evaluate
  // every input (no short-circuit) so each stalled one is logged, then require all.
  const bool kinematics_ok = !timed_out("kinematics", input_data_.kinematics->header.stamp);
  const bool trajectory_ok = !timed_out("trajectory", input_data_.trajectory->header.stamp);
  const bool control_cmd_ok = !timed_out("control_cmd", input_data_.control_cmd->stamp);
  const bool trajectory_follower_control_cmd_ok = !timed_out(
    "trajectory_follower_control_cmd", input_data_.trajectory_follower_control_cmd->stamp);
  return kinematics_ok && trajectory_ok && control_cmd_ok && trajectory_follower_control_cmd_ok;
}

void AutonomousModeTransitionFlagNode::on_driving_mode_info(const DrivingModeInfo & msg)
{
  for (const auto & item : msg.items) {
    if (item.name == "autonomous") {
      driving_mode_id_ = item.mode;
      break;
    }
  }
}

void AutonomousModeTransitionFlagNode::publish_driving_mode_stable(bool flag) const
{
  if (!driving_mode_id_) {
    return;
  }

  tier4_system_msgs::msg::DrivingModeFlagItem item;
  item.mode = driving_mode_id_.value();
  item.flag = flag;

  DrivingModeFlag msg;
  msg.stamp = now();
  msg.items = {item};
  pub_driving_mode_stable_->publish(msg);
}

void AutonomousModeTransitionFlagNode::publish_driving_mode_available(bool flag) const
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  DiagnosticStatus status;
  status.level = flag ? DiagnosticStatus::OK : DiagnosticStatus::ERROR;
  status.name = std::string(get_name()) + ": mode_change_available";

  DiagnosticArray msg;
  msg.header.stamp = now();
  msg.status = {status};
  pub_driving_mode_available_->publish(msg);
}

}  // namespace autoware::operation_mode_transition_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::operation_mode_transition_manager::AutonomousModeTransitionFlagNode)
