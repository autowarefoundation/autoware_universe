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

#ifndef AUTOWARE_RAW_VEHICLE_CMD_CONVERTER__UNDERSTEER_COMPENSATION_HPP_
#define AUTOWARE_RAW_VEHICLE_CMD_CONVERTER__UNDERSTEER_COMPENSATION_HPP_

namespace autoware::raw_vehicle_cmd_converter
{
// Steady-state understeer compensation *without* the mechanical VGR. The
// kinematic bicycle requires an extra Δδ = K_us · ay to track a curve at
// non-zero lateral acceleration. Pulling δ/L out and treating tan(δ) ≈ δ, this
// becomes a multiplicative factor on the (tire-angle) command:
//   δ_cmd = δ · (1 + K_us · v² / L)
// Unlike VGRWithUndersteerCompensation the output stays a tire angle (no gear
// ratio is applied); use this when the tire-to-wheel gear ratio is handled
// elsewhere (e.g. by the vehicle firmware).
class UndersteerCompensation
{
public:
  UndersteerCompensation() = default;
  void setUndersteerParams(double k_us, double wheelbase);
  double calculateUndersteerRatio(double vel) const;
  double calculateSteeringTireState(double vel, double steer_status) const;

private:
  double k_us_{0.0};
  double wheelbase_{1.0};
};
}  // namespace autoware::raw_vehicle_cmd_converter

#endif  // AUTOWARE_RAW_VEHICLE_CMD_CONVERTER__UNDERSTEER_COMPENSATION_HPP_
