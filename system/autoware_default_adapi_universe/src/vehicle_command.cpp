// Copyright 2025 The Autoware Contributors
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

#include "vehicle_command.hpp"

namespace autoware::default_adapi
{

VehicleCommandNode::VehicleCommandNode(const rclcpp::NodeOptions & options)
: Node("vehicle_command", options)
{
  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  adaptor.init_pub(pub_pedals_);
  adaptor.init_pub(pub_acceleration_);
  adaptor.init_pub(pub_velocity_);
  adaptor.init_pub(pub_steering_);
  adaptor.init_sub(sub_control_, this, &VehicleCommandNode::on_control);
  adaptor.init_sub(sub_actuation_, this, &VehicleCommandNode::on_actuation);
}

void VehicleCommandNode::on_control(const InternalControl::Message & msg)
{
  ExternalAcceleration::Message acceleration;
  acceleration.stamp = msg.stamp;
  acceleration.acceleration = msg.longitudinal.acceleration;
  pub_acceleration_->publish(acceleration);

  ExternalVelocity::Message velocity;
  velocity.stamp = msg.stamp;
  velocity.velocity = msg.longitudinal.velocity;
  pub_velocity_->publish(velocity);

  ExternalSteering::Message steering;
  steering.stamp = msg.stamp;
  steering.steering_tire_angle = msg.lateral.steering_tire_angle;
  steering.steering_tire_velocity = msg.lateral.steering_tire_rotation_rate;
  pub_steering_->publish(steering);
}

void VehicleCommandNode::on_actuation(const InternalActuation::Message & msg)
{
  ExternalPedals::Message pedals;
  pedals.stamp = msg.header.stamp;
  pedals.throttle = msg.actuation.accel_cmd;
  pedals.brake = msg.actuation.brake_cmd;
  pub_pedals_->publish(pedals);
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::VehicleCommandNode)
