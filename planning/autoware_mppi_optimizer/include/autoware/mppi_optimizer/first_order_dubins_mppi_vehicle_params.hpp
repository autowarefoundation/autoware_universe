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

#ifndef AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_VEHICLE_PARAMS_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_VEHICLE_PARAMS_HPP_

namespace autoware::mppi_optimizer
{

struct FirstOrderDubinsMppiVehicleParams
{
  /** Oriented-box footprint for obstacle collision (rear axle = state x,y). */
  float ego_length{0.825F};
  float ego_width{0.42F};
  float ego_axle_to_box_center{0.2F};

  /** Bicycle dynamics and control limits. */
  float wheel_base{0.32F};
  float max_steer_angle{0.45F};
  float min_accel{-6.0F};
  float max_accel{4.0F};
};

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_VEHICLE_PARAMS_HPP_
