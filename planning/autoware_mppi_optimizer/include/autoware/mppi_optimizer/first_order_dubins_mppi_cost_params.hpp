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

#ifndef AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_COST_PARAMS_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_COST_PARAMS_HPP_

namespace autoware::mppi_optimizer
{

/** Host-side cost weights; defaults match first_order_dubins_bicycle_cost.cuh and
 * mppi_optimizer.param.yaml */
struct FirstOrderDubinsMppiCostParams
{
  float desired_speed{3.0F};
  float speed_coeff{500.0F};
  float track_coeff{1000.0F};
  float heading_coeff{500.0F};
  float crash_coeff{100000.0F};
  float boundary_threshold{1.5F};
  float boundary_threshold_left{-1.0F};
  float boundary_threshold_right{-1.0F};
  float accel_cmd_coeff{0.0F};
  float steer_cmd_coeff{0.0F};
  float lateral_acceleration_coeff{300.0F};
  float lateral_jerk_coeff{300.0F};
  float longitudinal_jerk_coeff{10.0F};
  float obstacle_collision_margin{0.5F};
  float goal_pos_coeff{1000.0F};
  float goal_speed_coeff{0.0F};
  float goal_yaw_coeff{500.0F};
  float goal_terminal_scale{10.0F};
};

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_COST_PARAMS_HPP_
