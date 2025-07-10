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

#ifndef COMFORTABLE_STOP_HPP_
#define COMFORTABLE_STOP_HPP_

#include <autoware_command_mode_switcher/command_plugin.hpp>
#include <autoware_command_mode_types/modes.hpp>
#include <autoware_command_mode_types/sources.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_planning_msgs/msg/velocity_limit_clear_command.hpp>

#include <memory>

namespace autoware::command_mode_switcher
{

struct Params
{
  float min_acceleration;
  float max_jerk;
  float min_jerk;
};

// Plugins cannot be initialized in the constructor.
// cppcheck-suppress noConstructor
class ComfortableStopSwitcher : public CommandPlugin
{
public:
  uint16_t mode() const override { return autoware::command_mode_types::modes::comfortable_stop; }
  uint16_t source() const override { return autoware::command_mode_types::sources::main; }
  bool autoware_control() const override { return true; }
  void initialize() override;

  SourceState update_source_state(bool request) override;
  MrmState update_mrm_state() override;

private:
  void publish_velocity_limit();
  void publish_velocity_limit_clear_command();
  void publish_hazard_lights_command();
  bool is_stopped();

  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr pub_velocity_limit_;
  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimitClearCommand>::SharedPtr
    pub_velocity_limit_clear_command_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    pub_hazard_lights_command_;
  rclcpp::TimerBase::SharedPtr pub_hazard_lights_timer_;
  std::unique_ptr<autoware_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry>> sub_odom_;

  MrmState mrm_state_;
  bool enable_hazard_lights_;
  struct Params params_;
};

}  // namespace autoware::command_mode_switcher

#endif  // COMFORTABLE_STOP_HPP_
