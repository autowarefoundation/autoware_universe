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

#include "vehicle_metrics.hpp"

#include <limits>

namespace autoware::default_adapi
{

VehicleMetricsNode::VehicleMetricsNode(const rclcpp::NodeOptions & options)
: Node("vehicle_metrics", options)
{
  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  adaptor.init_pub(pub_metrics_);
  adaptor.init_sub(
    sub_energy_, [this](const EnergyStatus::Message::ConstSharedPtr msg) { energy_ = msg; });

  const auto period = rclcpp::Rate(declare_parameter<double>("update_rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void VehicleMetricsNode::on_timer()
{
  constexpr auto f_nan = std::numeric_limits<float>::quiet_NaN();

  VehicleMetrics::Message msg;
  msg.stamp = now();
  msg.energy = energy_ ? energy_->energy_level : f_nan;
  pub_metrics_->publish(msg);
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::VehicleMetricsNode)
