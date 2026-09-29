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

#ifndef MRM_STEERING_HOLD_STOP_OPERATOR_NODE_HPP_
#define MRM_STEERING_HOLD_STOP_OPERATOR_NODE_HPP_

#include "deceleration_profile.hpp"
#include "steering_hold_stop_logic.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_system_msgs/msg/in_lane_stop_trigger.hpp>

#include <memory>
#include <optional>

namespace autoware::mrm_steering_hold_stop_operator
{

using autoware_control_msgs::msg::Control;
using autoware_internal_debug_msgs::msg::Float32MultiArrayStamped;
using autoware_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using tier4_system_msgs::msg::InLaneStopTrigger;

class MrmSteeringHoldStopOperator : public rclcpp::Node
{
public:
  explicit MrmSteeringHoldStopOperator(const rclcpp::NodeOptions & node_options);

private:
  void onTrigger(const InLaneStopTrigger::ConstSharedPtr msg);
  void onSteering(const SteeringReport::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onAcceleration(const AccelWithCovarianceStamped::ConstSharedPtr msg);

  void onTimer();

  MeasuredInputs collectInputs(const rclcpp::Time & current_time) const;
  void publishControl(const CommandOutput & command, const rclcpp::Time & current_time);
  void publishDebugStatus(
    const UpdateResult & result, const MeasuredInputs & inputs, const rclcpp::Time & current_time);
  void logWaitingInput(const MeasuredInputs & inputs);

  // Parameters
  double update_rate_{33.3};
  DecelerationProfiles profiles_;

  // Subscribers
  rclcpp::Subscription<InLaneStopTrigger>::SharedPtr sub_trigger_;
  rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<AccelWithCovarianceStamped>::SharedPtr sub_acceleration_;

  // Publishers
  rclcpp::Publisher<Control>::SharedPtr pub_control_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_debug_status_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Logic
  std::unique_ptr<SteeringHoldStopLogic> logic_;
  TriggerState trigger_state_;
  ProfileType trigger_profile_{InLaneStopTrigger::PROFILE_UNKNOWN};
  bool is_profile_fallback_{false};

  // Latest inputs and their receive times (node clock)
  SteeringReport::ConstSharedPtr steering_;
  Odometry::ConstSharedPtr odometry_;
  AccelWithCovarianceStamped::ConstSharedPtr acceleration_;
  std::optional<rclcpp::Time> steering_received_time_;
  std::optional<rclcpp::Time> odometry_received_time_;
  std::optional<rclcpp::Time> acceleration_received_time_;

  std::optional<rclcpp::Time> last_tick_time_;
};

}  // namespace autoware::mrm_steering_hold_stop_operator

#endif  // MRM_STEERING_HOLD_STOP_OPERATOR_NODE_HPP_
