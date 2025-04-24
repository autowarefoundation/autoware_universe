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
  sub_energy_ = create_polling_subscription<EnergyStatus>(this);

  const auto on_timer = [this]() {
    constexpr double f_nan = std::numeric_limits<float>::quiet_NaN();
    const auto energy = sub_energy_->take_data();

    VehicleMetrics::Message msg;
    msg.stamp = now();
    msg.energy = energy ? energy->energy_level : f_nan;
    pub_metrics_->publish(msg);
  };
  const auto period = rclcpp::Rate(0.1).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, on_timer);
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::VehicleMetricsNode)
