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

#ifndef AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_VEHICLE_PARAMS_CONVERSION_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_VEHICLE_PARAMS_CONVERSION_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

namespace autoware::mppi_optimizer
{

using autoware::vehicle_info_utils::VehicleInfo;

inline FirstOrderDubinsMppiVehicleParams makeVehicleParams(const VehicleInfo & vehicle_info)
{
  FirstOrderDubinsMppiVehicleParams params;
  params.ego_length = static_cast<float>(vehicle_info.vehicle_length_m);
  params.ego_width = static_cast<float>(vehicle_info.vehicle_width_m);
  // Rear-axle (base_link) to geometric center of the vehicle bounding box.
  params.ego_axle_to_box_center =
    static_cast<float>(0.5F * vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m);
  params.wheel_base = static_cast<float>(vehicle_info.wheel_base_m);
  params.max_steer_angle = static_cast<float>(vehicle_info.max_steer_angle_rad);
  return params;
}

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_VEHICLE_PARAMS_CONVERSION_HPP_
