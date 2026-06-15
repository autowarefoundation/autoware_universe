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

#ifndef AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_VEHICLE_PARAMS_ROS_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_VEHICLE_PARAMS_ROS_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params.hpp"

#include <rclcpp/rclcpp.hpp>

namespace autoware::mppi_optimizer
{

/**
 * Declare vehicle actuation parameters using the same names as simulator_model.param.yaml so a
 * vehicle description simulator config can be loaded on the same node.
 */
void declare_first_order_dubins_mppi_vehicle_dynamics_params(rclcpp::Node & node);

/** Geometry from vehicle_info plus actuation dynamics from the parameter server. */
FirstOrderDubinsMppiVehicleParams get_first_order_dubins_mppi_vehicle_params(rclcpp::Node & node);

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_VEHICLE_PARAMS_ROS_HPP_
