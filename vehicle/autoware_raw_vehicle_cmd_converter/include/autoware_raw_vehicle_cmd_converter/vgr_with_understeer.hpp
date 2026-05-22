//  Copyright 2026 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE_RAW_VEHICLE_CMD_CONVERTER__VGR_WITH_UNDERSTEER_HPP_
#define AUTOWARE_RAW_VEHICLE_CMD_CONVERTER__VGR_WITH_UNDERSTEER_HPP_

#include "autoware_raw_vehicle_cmd_converter/vgr.hpp"

namespace autoware::raw_vehicle_cmd_converter
{
// Gear ratio that folds the steady-state understeer effect into the mechanical
// VGR. The kinematic bicycle requires an extra Δδ = K_us · ay to track a curve
// at non-zero lateral acceleration. Pulling δ/L out and treating tan(δ) ≈ δ,
// this becomes a multiplicative factor on the gear ratio:
//   N(v, δ_w) = N_mech(v, δ_w) · (1 + K_us · v² / L)
class VGRWithUndersteer
{
public:
  VGRWithUndersteer() = default;
  void setCoefficients(double a, double b, double c);
  void setUndersteerParams(double k_us, double wheelbase);
  double calculateUndersteerRatio(double vel) const;
  double calculateVariableGearRatio(double vel, double steer_wheel) const;
  double calculateSteeringTireState(double vel, double steer_wheel) const;

private:
  VGR vgr_;
  double k_us_{0.0};
  double wheelbase_{1.0};
};
}  // namespace autoware::raw_vehicle_cmd_converter

#endif  // AUTOWARE_RAW_VEHICLE_CMD_CONVERTER__VGR_WITH_UNDERSTEER_HPP_
