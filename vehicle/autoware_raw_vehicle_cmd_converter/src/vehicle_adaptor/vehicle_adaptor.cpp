//  Copyright 2024 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "autoware_raw_vehicle_cmd_converter/vehicle_adaptor/vehicle_adaptor.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::raw_vehicle_cmd_converter
{
void VehicleAdaptor::set_understeer_ff_params(
  const double k_us, const double max_correction, const double wheelbase)
{
  k_us_ = k_us;
  max_correction_ = max_correction;
  wheelbase_ = wheelbase;
}

Control VehicleAdaptor::compensate(
  const Control & input_control_cmd, const Odometry & odometry,
  [[maybe_unused]] const AccelWithCovarianceStamped & accel, [[maybe_unused]] const double steering,
  const OperationModeState & operation_mode,
  [[maybe_unused]] const ControlHorizon & control_horizon)
{
  Control output_control_cmd = input_control_cmd;

  if (operation_mode.mode != OperationModeState::AUTONOMOUS) {
    return output_control_cmd;
  }

  // NOTE: kinematic bicycle prediction of the commanded lateral acceleration
  // ay_cmd = vx² · tan(δ_cmd) / L
  // The understeer correction adds Δδ = K_us · ay_cmd, then clamps to a safe magnitude.
  const double vx = odometry.twist.twist.linear.x;
  const double delta_in = input_control_cmd.lateral.steering_tire_angle;
  const double ay_cmd = vx * vx * std::tan(delta_in) / wheelbase_;
  const double delta_correction = std::clamp(k_us_ * ay_cmd, -max_correction_, max_correction_);
  output_control_cmd.lateral.steering_tire_angle = delta_in + delta_correction;

  return output_control_cmd;
}
}  // namespace autoware::raw_vehicle_cmd_converter
