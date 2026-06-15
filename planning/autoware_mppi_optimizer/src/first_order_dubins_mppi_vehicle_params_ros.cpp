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

#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params_ros.hpp"

#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params_conversion.hpp"

#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>

namespace autoware::mppi_optimizer
{

void declare_first_order_dubins_mppi_vehicle_dynamics_params(rclcpp::Node & node)
{
  const FirstOrderDubinsMppiVehicleParams defaults;
  node.declare_parameter("acc_time_constant", defaults.acc_time_constant);
  node.declare_parameter("steer_time_constant", defaults.steer_time_constant);
  node.declare_parameter("steer_rate_lim", defaults.steer_rate_lim);
  node.declare_parameter("vel_rate_lim", defaults.vel_rate_lim);
  node.declare_parameter("acc_time_delay", defaults.acc_time_delay);
  node.declare_parameter("steer_time_delay", defaults.steer_time_delay);
}

FirstOrderDubinsMppiVehicleParams get_first_order_dubins_mppi_vehicle_params(rclcpp::Node & node)
{
  FirstOrderDubinsMppiVehicleParams params =
    makeVehicleParams(autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo());
  params.acc_time_constant =
    static_cast<float>(node.get_parameter("acc_time_constant").as_double());
  params.steer_time_constant =
    static_cast<float>(node.get_parameter("steer_time_constant").as_double());
  params.steer_rate_lim = static_cast<float>(node.get_parameter("steer_rate_lim").as_double());
  params.vel_rate_lim = static_cast<float>(node.get_parameter("vel_rate_lim").as_double());
  params.acc_time_delay = static_cast<float>(node.get_parameter("acc_time_delay").as_double());
  params.steer_time_delay = static_cast<float>(node.get_parameter("steer_time_delay").as_double());
  return params;
}

}  // namespace autoware::mppi_optimizer
