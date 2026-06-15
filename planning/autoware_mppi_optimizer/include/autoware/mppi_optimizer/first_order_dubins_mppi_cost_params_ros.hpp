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

#ifndef AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_COST_PARAMS_ROS_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_COST_PARAMS_ROS_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::mppi_optimizer
{

/** Declare ROS parameters with defaults from FirstOrderDubinsBicycleCostParams. */
void declare_first_order_dubins_mppi_cost_params(
  rclcpp::Node & node, const std::string & prefix = "");

/** Read declared MPPI cost parameters from a node. */
FirstOrderDubinsMppiCostParams get_first_order_dubins_mppi_cost_params(
  const rclcpp::Node & node, const std::string & prefix = "");

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_COST_PARAMS_ROS_HPP_
