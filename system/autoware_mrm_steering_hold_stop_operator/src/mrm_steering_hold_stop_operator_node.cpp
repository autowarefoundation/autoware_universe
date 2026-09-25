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

#include "mrm_steering_hold_stop_operator_node.hpp"

#include <memory>
#include <string>

namespace autoware::mrm_steering_hold_stop_operator
{

namespace
{
constexpr float kAgeNeverReceived = -1.0f;

float toAgeField(const std::optional<double> & age)
{
  return age.has_value() ? static_cast<float>(*age) : kAgeNeverReceived;
}
}  // namespace

MrmSteeringHoldStopOperator::MrmSteeringHoldStopOperator(const rclcpp::NodeOptions & node_options)
: Node("mrm_steering_hold_stop_operator", node_options)
{
  using std::placeholders::_1;

  // Parameters
  update_rate_ = declare_parameter<double>("update_rate");
  LogicParams logic_params;
  logic_params.input_freshness_timeout = declare_parameter<double>("input_freshness_timeout");
  logic_params.low_speed_threshold = declare_parameter<double>("low_speed_threshold");
  logic_ = std::make_unique<SteeringHoldStopLogic>(logic_params);

  const auto declare_targets = [this](const std::string & profile_name) {
    const std::string prefix = "profiles." + profile_name + ".";
    DecelerationTargets targets;
    targets.target_acceleration = declare_parameter<double>(prefix + "target_acceleration");
    targets.target_jerk = declare_parameter<double>(prefix + "target_jerk");
    return targets;
  };
  profiles_.moderate = declare_targets("moderate");
  profiles_.emergency = declare_targets("emergency");
  // Report the moderate targets until the first trigger arrives.
  trigger_state_.target_acceleration = profiles_.moderate.target_acceleration;
  trigger_state_.target_jerk = profiles_.moderate.target_jerk;

  // Subscribers
  // The trigger is published once per change; transient_local lets a (re)started node
  // receive the latest one.
  sub_trigger_ = create_subscription<InLaneStopTrigger>(
    "~/input/trigger", rclcpp::QoS{1}.reliable().transient_local(),
    std::bind(&MrmSteeringHoldStopOperator::onTrigger, this, _1));
  sub_steering_ = create_subscription<SteeringReport>(
    "~/input/steering_status", rclcpp::QoS{1},
    std::bind(&MrmSteeringHoldStopOperator::onSteering, this, _1));
  sub_odometry_ = create_subscription<Odometry>(
    "~/input/kinematic_state", rclcpp::QoS{1},
    std::bind(&MrmSteeringHoldStopOperator::onOdometry, this, _1));
  sub_acceleration_ = create_subscription<AccelWithCovarianceStamped>(
    "~/input/acceleration", rclcpp::QoS{1},
    std::bind(&MrmSteeringHoldStopOperator::onAcceleration, this, _1));

  // Publishers
  pub_control_ = create_publisher<Control>("~/output/control", rclcpp::QoS{1});
  pub_debug_status_ = create_publisher<Float32MultiArrayStamped>("~/debug/status", rclcpp::QoS{1});

  // Timer
  const auto period_ns = rclcpp::Rate(update_rate_).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&MrmSteeringHoldStopOperator::onTimer, this));
}

void MrmSteeringHoldStopOperator::onTrigger(const InLaneStopTrigger::ConstSharedPtr msg)
{
  const auto resolved = resolve_profile(profiles_, msg->profile);
  trigger_state_.active = msg->trigger;
  trigger_state_.target_acceleration = resolved.targets.target_acceleration;
  trigger_state_.target_jerk = resolved.targets.target_jerk;
  trigger_profile_ = msg->profile;
  is_profile_fallback_ = resolved.is_fallback;
}

void MrmSteeringHoldStopOperator::onSteering(const SteeringReport::ConstSharedPtr msg)
{
  steering_ = msg;
  steering_received_time_ = now();
}

void MrmSteeringHoldStopOperator::onOdometry(const Odometry::ConstSharedPtr msg)
{
  odometry_ = msg;
  odometry_received_time_ = now();
}

void MrmSteeringHoldStopOperator::onAcceleration(
  const AccelWithCovarianceStamped::ConstSharedPtr msg)
{
  acceleration_ = msg;
  acceleration_received_time_ = now();
}

void MrmSteeringHoldStopOperator::onTimer()
{
  const auto current_time = now();
  const double dt = last_tick_time_.has_value() ? (current_time - *last_tick_time_).seconds() : 0.0;
  last_tick_time_ = current_time;

  if (trigger_state_.active && is_profile_fallback_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Unknown in-lane stop profile %u requested. Decelerating with the moderate profile.",
      static_cast<unsigned int>(trigger_profile_));
  }

  const auto inputs = collectInputs(current_time);
  const auto result = logic_->update(trigger_state_, inputs, dt);

  if (result.command.has_value()) {
    publishControl(*result.command, current_time);
  } else {
    logWaitingInput(inputs);
  }
  publishDebugStatus(result, inputs, current_time);
}

MeasuredInputs MrmSteeringHoldStopOperator::collectInputs(const rclcpp::Time & current_time) const
{
  MeasuredInputs inputs;
  if (steering_ && steering_received_time_.has_value()) {
    inputs.steering_age = (current_time - *steering_received_time_).seconds();
    inputs.steering_tire_angle = steering_->steering_tire_angle;
  }
  if (odometry_ && odometry_received_time_.has_value()) {
    inputs.odom_age = (current_time - *odometry_received_time_).seconds();
    inputs.longitudinal_velocity = odometry_->twist.twist.linear.x;
  }
  if (acceleration_ && acceleration_received_time_.has_value()) {
    inputs.accel_age = (current_time - *acceleration_received_time_).seconds();
    inputs.longitudinal_acceleration = acceleration_->accel.accel.linear.x;
  }
  return inputs;
}

void MrmSteeringHoldStopOperator::publishControl(
  const CommandOutput & command, const rclcpp::Time & current_time)
{
  Control control;
  control.stamp = current_time;
  control.lateral.stamp = current_time;
  control.lateral.steering_tire_angle = static_cast<float>(command.steering_tire_angle);
  control.lateral.steering_tire_rotation_rate = 0.0f;
  control.longitudinal.stamp = current_time;
  control.longitudinal.velocity = static_cast<float>(command.velocity);
  control.longitudinal.acceleration = static_cast<float>(command.acceleration);
  control.longitudinal.jerk = static_cast<float>(command.jerk);
  pub_control_->publish(control);
}

void MrmSteeringHoldStopOperator::publishDebugStatus(
  const UpdateResult & result, const MeasuredInputs & inputs, const rclcpp::Time & current_time)
{
  Float32MultiArrayStamped status;
  status.stamp = current_time;
  status.data.reserve(11);
  status.data.push_back(static_cast<float>(result.state));
  status.data.push_back(trigger_state_.active ? 1.0f : 0.0f);
  status.data.push_back(toAgeField(inputs.steering_age));
  status.data.push_back(toAgeField(inputs.odom_age));
  status.data.push_back(toAgeField(inputs.accel_age));
  status.data.push_back(static_cast<float>(logic_->held_steering_angle()));
  status.data.push_back(
    result.command.has_value() ? static_cast<float>(result.command->velocity) : 0.0f);
  status.data.push_back(
    result.command.has_value() ? static_cast<float>(result.command->acceleration) : 0.0f);
  status.data.push_back(static_cast<float>(trigger_state_.target_acceleration));
  status.data.push_back(static_cast<float>(trigger_state_.target_jerk));
  status.data.push_back(static_cast<float>(trigger_profile_));
  pub_debug_status_->publish(status);
}

void MrmSteeringHoldStopOperator::logWaitingInput(const MeasuredInputs & inputs)
{
  const auto describe = [](const std::optional<double> & age) {
    return age.has_value() ? std::to_string(*age) + "s old" : std::string("never received");
  };
  RCLCPP_ERROR_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Not publishing control command: required input is stale (steering: %s, odometry: %s, "
    "acceleration: %s). The command gate timeout will escalate to an upper-level fail-safe.",
    describe(inputs.steering_age).c_str(), describe(inputs.odom_age).c_str(),
    describe(inputs.accel_age).c_str());
}

}  // namespace autoware::mrm_steering_hold_stop_operator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::mrm_steering_hold_stop_operator::MrmSteeringHoldStopOperator)
